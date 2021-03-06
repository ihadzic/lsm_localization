/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

/* This package contains modifications that are copyright by Nokia
 *
 * Licensed under the BSD 3-Clause "New" or "Revised" License
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <laser_scan_matcher/scan_constructor.h>
#undef min
#undef max

#define MAX_ODOM_HISTORY 1024
#define MAX_ODOM_AGE 2.0

namespace scan_tools
{

class LaserScanMatcher
{
  public:

    LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~LaserScanMatcher();

  private:

    // **** ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber map_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber initialpose_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_
    tf::Transform base_to_footprint_;
    tf::Transform footprint_to_base_;

    ros::Publisher  pose_publisher_;
    ros::Publisher  pose_stamped_publisher_;
    ros::Publisher  pose_with_covariance_publisher_;
    ros::Publisher  pose_with_covariance_stamped_publisher_;
    ros::Publisher  predicted_pose_publisher_;
    ros::Publisher  measured_pose_publisher_;
    ros::Publisher  constructed_scan_publisher_;

    ros::Publisher  debug_odom_delta_publisher_;
    ros::Publisher  debug_laser_delta_publisher_;
    ros::Publisher  debug_odom_reference_publisher_;
    ros::Publisher  debug_odom_current_publisher_;

    // **** parameters

    std::string base_frame_;
    std::string fixed_frame_;
    std::string odom_frame_;
    std::string initialpose_topic_;
    int scan_downsample_rate_;
    double max_variance_trans_;
    double max_variance_rot_;
    double max_allowed_range_;
    double max_pose_delta_yaw_;
    double map_occupancy_threshold_;
    bool use_map_;
    bool initialpose_valid_;
    bool publish_constructed_scan_;
    bool publish_base_tf_;
    bool publish_odom_tf_;
    bool publish_pose_;
    bool publish_pose_with_covariance_;
    bool publish_pose_stamped_;
    bool publish_pose_with_covariance_stamped_;
    bool publish_predicted_pose_;
    bool publish_measured_pose_;
    bool publish_debug_;
    bool debug_csm_;
    std::vector<double> position_covariance_;
    std::vector<double> orientation_covariance_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;

    bool use_odom_;
    bool no_odom_fusing_;

    // **** state variables

    boost::mutex mutex_;

    bool initialized_;
    bool have_map_;
    bool received_odom_;
    ScanConstructor scan_constructor_;
    double observed_range_min_;
    double observed_range_max_;
    double observed_angle_min_;
    double observed_angle_max_;
    double observed_angle_inc_;
    double observed_scan_time_;
    double observed_time_inc_;
    std::string observed_scan_frame_;
    int skipped_poses_;

    tf::Transform predicted_pose_;
    tf::Transform measured_pose_;
    tf::Transform predicted_pose_in_pcl_;
    tf::Transform initial_pose_;
    tf::Transform f2pcl_;  // fixed-to-point-cloud-local tf
    tf::Transform pcl2f_;  // (and its inverse)

    tf::Transform f2b_;    // fixed-to-base tf (pose of base frame in fixed frame)
    tf::Transform f2b_kf_; // pose of the last keyframe scan in fixed frame

    ros::Time last_icp_time_;

    nav_msgs::Odometry current_odom_msg_;
    nav_msgs::Odometry reference_odom_msg_;

    std::vector<double> constructed_intensities_;
    std::vector<double> constructed_ranges_;
    std::deque<nav_msgs::Odometry> odom_history_;

    // covariance tracking
    gsl_matrix *Sigma_odom_;
    gsl_matrix *Sigma_odom_trans_;
    gsl_matrix *Sigma_measured_;
    gsl_matrix *B_odom_;
    gsl_matrix *Sigma_u_;
    gsl_matrix *I1_;
    gsl_matrix *I2_;
    gsl_permutation *P2_;
    gsl_matrix *trans_sigma_;
    gsl_matrix *kalman_gain_;
    gsl_matrix *kalman_gain_comp_;
    gsl_vector *xvec_;
    gsl_vector *yvec_;
    double theta_odom_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // **** methods

    void initParams();
    void resetState();
    double syncOdom(const ros::Time& time);
    void addOdomToHistory(const nav_msgs::Odometry::ConstPtr& o);
    double getOdomDeltaT(const nav_msgs::Odometry::ConstPtr& o);
    void setTransSigmaMatrix(const double yaw);

    nav_msgs::Odometry* earliestOdomAfter(const ros::Time& time);
    nav_msgs::Odometry* latestOdomBefore(const ros::Time& time);
    tf::Vector3 fusePoses(const tf::Transform& pose_delta);
    int processScan(LDP& curr_ldp_scan, const ros::Time& time);
    int processScan(LDP& curr_ldp_scan, LDP& ref_ldp_scan, const ros::Time& time);
    void doPredictPose(double delta_t);
    void doPublishScanRate(const ros::Time& time);
    void doPublishOdomRate(const ros::Time& time);
    void doPublishDebugTF(const ros::Time& time, const tf::Transform& odom_delta, const ros::Publisher& publisher, const std::string& frame);

    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);
    void constructedScanToLDP(LDP& ldp);
    void constructScan(const ros::Time& timestamp);

    void mapCallback (const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

    void initialposeCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    bool getBaseToLaserTf (const std::string& frame_id);
    bool getBaseToFootprintTf (const std::string& frame_id);

    bool newKeyframeNeeded(const tf::Transform& d);

    void getPrediction(double& pr_ch_x, double& pr_ch_y,
                       double& pr_ch_a, double dt);

    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
};

} // namespace scan_tools

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
