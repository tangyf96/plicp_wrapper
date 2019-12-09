#include  <plicp/plicp_node.hpp>

namespace icp_tools
{
Plicp::Plicp(lcm::LCM* plcm): pLCM(plcm)
{
    std::cout << "Starting Plicp" << std::endl;
    // 接受激光数据
    pLCM->subscribe("lidar_scan", &Plicp::lcmScanCallback, this);
    pLCM->subscribe("odom_cali", &Plicp::handleOdom, this);

    std::cout << "init subscriber" << std::endl;
    init();
}

Plicp::~Plicp()
{ 
  std::cout << "Destroying PlicpWrapper" << std::endl;
}

void Plicp::init()
{
    filename_ =  "/home/yifan/data.txt";
    std::string odomfile = "/home/yifan/odom.txt";
    // 数据
    // 打开保存的文件
    ofile_.open(filename_.c_str());
    if (!ofile_)
    {
        std::cout << "Fail to open file!" << std::endl;
        return;
    }
    else
        ofile_ << "delta_x , delta_y, delta_theta, timeStamp" << std::endl;
    
    odomFile_.open(odomfile.c_str());
    if (!odomFile_)
    {
      std::cout << "Failed to open odom file!" << std::endl;
      return;
    }
    else
      odomFile_ << "delta_x , delta_y, delta_theta, timeStamp" << std::endl;

    // Init parameters
    max_iterations_ =  30;
    max_correspondence_dist_ = 0.08; // 0.1
    max_angular_correction_deg_= 1.5;
    max_linear_correction_ = 0.02;
    epsilon_xy_ = 0.000001;
    epsilon_theta_ = 0.000001;
    outliers_maxPerc_ = 0.95;

    // Advanced parameters
    sigma_ = 0.01;
    use_corr_tricks_ = 1;
    restart_ = 0;
    restart_threshold_mean_error_ = 0.01;
    restart_dt_ = 1.0;
    restart_dtheta_ = 0.1;
    clustering_threshold_ = 0.25;
    orientation_neighbourhood_ = 10;
    use_point_to_line_distance_ = 1;
    do_alpha_test_ = 1;
    do_alpha_test_thresholdDeg_ = 2.0;
    outliers_adaptive_order_ = 0.7;
    outliers_adaptive_mul_ = 2.0;
    do_visibility_test_ = 0;
    outliers_remove_doubles_ = 1;
    do_compute_covariance_ = 0;
    debug_verify_trick_ = 0;
    use_ml_weights_ = 0;
    use_sigma_weights_ = 0;

    output_.valid = false;
    // set params for input
    set_params();
}

void Plicp::set_params()
{
    // Init parameters
    input_.max_iterations = max_iterations_;
    input_.max_correspondence_dist = max_correspondence_dist_;
    input_.max_angular_correction_deg = max_angular_correction_deg_;
    input_.max_linear_correction = max_linear_correction_;
    input_.epsilon_xy = epsilon_xy_;
    input_.epsilon_theta = epsilon_theta_;
    input_.outliers_maxPerc = outliers_maxPerc_;
    // Advanced parameters
    input_.sigma = sigma_;
    input_.use_corr_tricks = use_corr_tricks_;
    input_.restart = restart_;
    input_.restart_threshold_mean_error = restart_threshold_mean_error_;
    input_.restart_dt = restart_dt_;
    input_.restart_dtheta = restart_dtheta_;
    input_.clustering_threshold = clustering_threshold_;
    input_.orientation_neighbourhood = orientation_neighbourhood_;
    input_.use_point_to_line_distance = use_point_to_line_distance_;
    input_.do_alpha_test = do_alpha_test_;
    input_.do_alpha_test_thresholdDeg = do_alpha_test_thresholdDeg_;
    input_.outliers_adaptive_order = outliers_adaptive_order_;
    input_.outliers_adaptive_mult = outliers_adaptive_mul_;
    input_.do_visibility_test = do_visibility_test_;
    input_.outliers_remove_doubles = outliers_remove_doubles_;
    input_.do_compute_covariance = do_compute_covariance_;
    input_.debug_verify_tricks = debug_verify_trick_;
    input_.use_ml_weights = use_ml_weights_;
    input_.use_sigma_weights = use_sigma_weights_;
}

void Plicp::lcmScanCallback(const lcm::ReceiveBuffer *rbuf, 
                                                            const std::string &channel, 
                                                            const lcm_visualization_msgs::Marker *pLidarScan)
{
  std::cout << "Get message" << std::endl;
  if (!initialized_)
  {
    input_.min_reading = 0.1;
    input_.max_reading = 8.0;
    laserScanToLDP(pLidarScan, prev_ldp_scan_);
    last_icp_time_ = pLidarScan->header.stamp;
    initialized_ = true;
  }
  else
  {
    LDP curr_ldp_scan;
    laserScanToLDP(pLidarScan, curr_ldp_scan);
    processScan(curr_ldp_scan, pLidarScan->header.stamp);
  }
  if (output_.valid)
  {
      // std::cout << "save data" << std::endl;
      ofile_ << output_.x[0] << ", " 
                  << output_.x[1] << ", "
                  << output_.x[2] << ", "
                  <<  curTime << std::endl;
  }
}

void Plicp::laserScanToLDP(const lcm_visualization_msgs::Marker *pLidarScan, LDP& ldp)
{
  size_t n = pLidarScan->points.size();
  ldp = ld_alloc_new(n);

  double min_theta = DBL_MAX, max_theta = DBL_MIN;
  for (size_t i = 0; i < n; ++i)
  {
    // calculate position in laser frame
    double x = pLidarScan->points[i].x;
    double y = pLidarScan->points[i].y;
    double range = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double theta = std::atan2(y, x);
    if (range > min_reading_ && range < max_reading_)
    {
      // fill in laser scan data
      ldp->valid[i] = 1;
      ldp->readings[i] = range;
      ldp->theta[i] = theta;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
      ldp->theta[i] = theta;
    }

    ldp->cluster[i]  = -1;
    if (min_theta > theta)
      min_theta = theta;
    if (max_theta < theta)
      max_theta = theta;
  }
  ldp->min_theta = min_theta;
  ldp->max_theta = max_theta;

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->estimate[0] = 0.0;
  ldp->estimate[1] = 0.0;
  ldp->estimate[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void Plicp::processScan(LDP& curr_ldp_scan, const lcm_std_msgs::Time time)
{
    curTime = time;

    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    prev_ldp_scan_->odometry[0] = 0.0;
    prev_ldp_scan_->odometry[1] = 0.0;
    prev_ldp_scan_->odometry[2] = 0.0;

    prev_ldp_scan_->estimate[0] = 0.0;
    prev_ldp_scan_->estimate[1] = 0.0;
    prev_ldp_scan_->estimate[2] = 0.0;

    prev_ldp_scan_->true_pose[0] = 0.0;
    prev_ldp_scan_->true_pose[1] = 0.0;
    prev_ldp_scan_->true_pose[2] = 0.0;

    // first_guess 为 [0, 0, 0]
    input_.first_guess[0] = 0.0;
    input_.first_guess[1] = 0.0;
    input_.first_guess[2] = 0.0;

    input_.laser_ref  = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;
    sm_icp(&input_, &output_);
}

void Plicp::handleOdom(const lcm::ReceiveBuffer* rbuf,
                  const std::string& channel,
                  const lcm_nav_msgs::Odometry* odom_cali)
{
  double time_stamp = odom_cali->header.stamp.sec + double(odom_cali->header.stamp.nsec)*1e-9;
  //计算delta_x, delta_y, delta_yaw
  double dSl = odom_cali->pose.pose.position.x;
  double dSr = odom_cali->pose.pose.position.y;
  double b = 0.2385292521;// 轮距b(m)
  double dS = (dSr + dSl) / 2;// ∆s
  double dYaw = (dSr - dSl) / b;// ∆θ, robot姿态变化量

  double cy = std::cos(dYaw / 2);// cos(∆θ/2)
  double sy = std::sin(dYaw / 2);// sin(∆θ/2)
  double dX = dS * cy;// ∆x = ∆s * cos(∆θ/2)
  double dY = dS * sy;// ∆y = ∆s * sin(∆θ/2)
  
  odomFile_ << dX << ", " 
            << dY << ", "
            << dYaw << ", "
            <<  time_stamp << std::endl;
}

}