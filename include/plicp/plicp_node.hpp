#ifndef PLICP_NODE_HPP
#define PLICP_NODE_HPP

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains

#include <lcm/lcm-cpp.hpp>
#include <lcm_ros/time.h>
#include <lcm_ros/transform_datatypes.h>
#include <lcm_nav_msgs/Path.hpp>
#include <lcm_nav_msgs/OccupancyGrid.hpp>
#include <lcm_nav_msgs/Odometry.hpp>  //话题robot_pose_icp, odom_cali数据
#include <lcm_visualization_msgs/Marker.hpp>

#include <math.h>
#include <fstream>
#include <vector>
#include <algorithm> 
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Core>

namespace icp_tools
{

class Plicp
{
    public:
        Plicp();
        Plicp(lcm::LCM* plcm);
        ~Plicp();
        
        void init();
        void processScan(LDP& curr_ldp_scan, const lcm_std_msgs::Time stamp);
        void set_params();
        void lcmScanCallback(const lcm::ReceiveBuffer *rbuf, 
                                                            const std::string &channel, 
                                                            const lcm_visualization_msgs::Marker *pLidarScan);
        void laserScanToLDP(const lcm_visualization_msgs::Marker *pLidarScan, LDP& ldp);
        void handleOdom(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lcm_nav_msgs::Odometry* odom_cali);
        void set_first_guess(sm_params* params);

        bool valid_odometry(const double* delta_odom);
        void copy_d(const double*from, int n, double*to);


    public:
        sm_result output_;
        int cnt_ = 0;

    private:
        std::ofstream ofile_;
        std::ofstream odomFile_;
        std::ifstream inOdomFile_;
        std::mutex mutexOdom_;
        bool initOdom_ = true;
        double max_rotation_deg_;
        double odom_laser_T[3];
        Eigen::Matrix3d T_odom_laser_;

        // lcm
        lcm::LCM* pLCM;
        lcm_std_msgs::Time last_icp_time_;
        double min_reading_ = 0.0001;
        double max_reading_ =  10.0;

        // csm
        sm_params input_;
        LDP prev_ldp_scan_;
        double last_trans_[3];
        double last_odom_[3];

        // PLICP parameters
        int max_iterations_;
        double max_correspondence_dist_ ;
        double max_angular_correction_deg_;
        double max_linear_correction_ ;
        double epsilon_xy_;
        double epsilon_theta_;
        double outliers_maxPerc_ ;

        // Advanced parameters
        double sigma_;
        int use_corr_tricks_;
        int restart_;
        double restart_threshold_mean_error_ ;
        double restart_dt_ ;
        double restart_dtheta_;
        double clustering_threshold_ ;
        int orientation_neighbourhood_;
        int use_point_to_line_distance_;
        int do_alpha_test_;
        double do_alpha_test_thresholdDeg_;
        double outliers_adaptive_order_;
        double outliers_adaptive_mul_;
        int do_visibility_test_;
        int outliers_remove_doubles_ ;
        int do_compute_covariance_ ;
        int debug_verify_trick_;
        int use_ml_weights_;
        int use_sigma_weights_;

        // other parameters
        bool initialized_ = false;
};
}
#endif