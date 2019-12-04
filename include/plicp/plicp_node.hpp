#ifndef PLICP_NODE_HPP
#define PLICP_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains

#include <lcm/lcm-cpp.hpp>

#include <fstream>
#include <vector>

namespace icp_tools
{

class Plicp
{
    public:
        Plicp();
        Plicp(ros::NodeHandle nh);
        ~Plicp();
        
        void init();
        void processScan(LDP& curr_ldp_scan, const ros::Time& time);
        void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgs);
        void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        void set_params();

    public:
        sm_result output_;
        ros::Time curTime;

    private:
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber scan_subsciber_;
        ros::Time last_icp_time_;

        // csm
        sm_params input_;
        LDP prev_ldp_scan_;

        std::vector<double> a_cos_;
        std::vector<double> a_sin_;

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