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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

#include <algorithm>      // Now safe to use std::max and std::min
#include <complex>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <deque>
#include <laser_scan_matcher/scan_constructor.h>


#define MAX_ODOM_HISTORY 1024
#define MAX_ODOM_AGE 2.0


namespace scan_tools
{

class LaserScanMatcher : public rclcpp::Node
{
  public:

    explicit LaserScanMatcher();
    ~LaserScanMatcher();

  private:

    // **** ros
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2::Transform base_to_laser_; // static, cached
    tf2::Transform laser_to_base_; // static, cached, calculated from base_to_laser_
    tf2::Transform base_to_footprint_;
    tf2::Transform footprint_to_base_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_with_covariance_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_stamped_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr predicted_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr measured_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr constructed_scan_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_delta_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_reference_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_current_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_laser_delta_publisher_;


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

    std::mutex mutex_;

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

    tf2::Transform predicted_pose_;
    tf2::Transform measured_pose_;
    tf2::Transform predicted_pose_in_pcl_;
    tf2::Transform initial_pose_;
    tf2::Transform f2pcl_;   // fixed-to-point-cloud-local tf
    tf2::Transform pcl2f_;   // inverse

    tf2::Transform f2b_;     // fixed-to-base tf
    tf2::Transform f2b_kf_;  // last keyframe pose

    rclcpp::Time last_icp_time_;

    nav_msgs::msg::Odometry current_odom_msg_;
    nav_msgs::msg::Odometry reference_odom_msg_;

    std::vector<double> constructed_intensities_;
    std::vector<double> constructed_ranges_;
    std::deque<nav_msgs::msg::Odometry> odom_history_;

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

    double syncOdom(const rclcpp::Time& time);
    void addOdomToHistory(const nav_msgs::msg::Odometry::SharedPtr& o);
    double getOdomDeltaT(const nav_msgs::msg::Odometry::SharedPtr& o);
    void setTransSigmaMatrix(const double yaw);

    nav_msgs::msg::Odometry* earliestOdomAfter(const rclcpp::Time& time);
    nav_msgs::msg::Odometry* latestOdomBefore(const rclcpp::Time& time);

    tf2::Vector3 fusePoses(const tf2::Transform& pose_delta);

    int processScan(LDP& curr_ldp_scan, const rclcpp::Time& time);
    int processScan(LDP& curr_ldp_scan, LDP& ref_ldp_scan, const rclcpp::Time& time);

    void doPredictPose(double delta_t);
    void doPublishScanRate(const rclcpp::Time& time);
    void doPublishOdomRate(const rclcpp::Time& time);

    void doPublishDebugTF(
      const rclcpp::Time& time,
      const tf2::Transform& transform,
      const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher,
      const std::string& frame
    );

    void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg, LDP& ldp);
    void constructedScanToLDP(LDP& ldp);
    void constructScan(const rclcpp::Time& time);

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg);
    void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& pose_msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr& odom_msg);

    bool getBaseToLaserTf(const std::string& frame_id);
    bool getBaseToFootprintTf(const std::string& frame_id);

    bool newKeyframeNeeded(const tf2::Transform& d);

    void getPrediction(double& pr_ch_x, double& pr_ch_y, double& pr_ch_a, double dt);
    void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);
};

geometry_msgs::msg::TransformStamped createTransformStamped(
    const tf2::Transform& transform,
    const rclcpp::Time& stamp,
    const std::string& parent_frame,
    const std::string& child_frame
);
geometry_msgs::msg::Pose transformToPose(const tf2::Transform& tf);
geometry_msgs::msg::PoseStamped transformToPoseStamped(
    const tf2::Transform& tf,
    const std::string& frame_id,
    const rclcpp::Time& stamp
    );   
geometry_msgs::msg::PoseWithCovariance transformToPoseWithCovariance(const tf2::Transform& tf);
geometry_msgs::msg::PoseWithCovarianceStamped transformToPoseWithCovarianceStamped(
    const tf2::Transform& tf,
    const std::string& frame_id,
    const rclcpp::Time& stamp
    );




} // namespace scan_tools

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
