#include  <plicp/plicp_node.hpp>

namespace icp_tools
{
Plicp::Plicp(ros::NodeHandle nh): nh_(nh)
{
    ROS_INFO("Starting LaserScanMatcher");
    init();
}

Plicp::~Plicp()
{ 
  ROS_INFO("Destroying PlicpWrapper");
}

void Plicp::init()
{
    // 接受激光数据
    scan_subsciber_ =  nh_.subscribe("/scan", 1, &Plicp::scanCallback, this);
    // lcm->subscribe("lidar_scan", &Plicp::lcmScanCallback, this);

    // Init parameters
    max_iterations_ = 10;
    max_correspondence_dist_ = 0.3;
    max_angular_correction_deg_= 45.0;
    max_linear_correction_ = 0.5;
    epsilon_xy_ = 0.000001;
    epsilon_theta_ = 0.000001;
    outliers_maxPerc_ = 0.9;
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
    do_alpha_test_ = 0;
    do_alpha_test_thresholdDeg_ = 20.0;
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

void Plicp::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgs)
{
    std::cout << "Get Scan Message" << std::endl;
    if (!initialized_)
    {
        createCache(msgs);    // caches the sin and cos of all angles
        laserScanToLDP(msgs, prev_ldp_scan_);
        last_icp_time_ = msgs->header.stamp;
        initialized_ = true;
    }

    LDP curr_ldp_scan;
    laserScanToLDP(msgs, curr_ldp_scan);
    processScan(curr_ldp_scan, msgs->header.stamp);
}

void Plicp::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }
  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void Plicp::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
  
  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}

void Plicp::processScan(LDP& curr_ldp_scan, const ros::Time& time)
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

    input_.laser_ref  = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // first_guess 为 [0, 0, 0]
    input_.first_guess[0] = prev_ldp_scan_->true_pose[0];
    input_.first_guess[1] = prev_ldp_scan_->true_pose[1];
    input_.first_guess[2] = prev_ldp_scan_->true_pose[2];

    sm_icp(&input_, &output_);
}

}