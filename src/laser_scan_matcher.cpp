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

#include <mutex>
#include <laser_scan_matcher/laser_scan_matcher.h>

namespace scan_tools
{

LaserScanMatcher::LaserScanMatcher() : rclcpp::Node("laser_scan_matcher"),
  initialized_(false),
  received_odom_(false),
  have_map_(false),
  initialpose_valid_(false),
  theta_odom_(0.0),
  skipped_poses_(0)
{
  RCLCPP_INFO(this->get_logger(), "Starting LaserScanMatcher");

  // **** init parameters

  initParams();
  if (debug_csm_)
    sm_debug_write(1);

  // **** state variables
  scan_counter_ = 0; 

  // pre-allocate all matrices for speed
  Sigma_odom_ = gsl_matrix_calloc(3, 3);
  Sigma_odom_trans_ = gsl_matrix_alloc(3, 3);
  Sigma_measured_ = gsl_matrix_calloc(3, 3);
  B_odom_ = gsl_matrix_calloc(3, 5);
  gsl_matrix_set(B_odom_, 2, 4, 1.0);
  Sigma_u_ = gsl_matrix_calloc(5, 5);
  trans_sigma_ = gsl_matrix_calloc(3, 3);
  gsl_matrix_set(trans_sigma_, 2, 2, 1);
  kalman_gain_ = gsl_matrix_alloc(3, 3);
  kalman_gain_comp_ = gsl_matrix_alloc(3, 3);
  // interim results
  I1_ = gsl_matrix_alloc(5, 3);
  I2_ = gsl_matrix_alloc(3, 3);
  P2_ = gsl_permutation_alloc (3);
  // pose vectors for kalman filter
  xvec_ = gsl_vector_alloc(3);
  yvec_ = gsl_vector_alloc(3);

  resetState();
  pcl2f_.setIdentity();
  f2pcl_.setIdentity();
  measured_pose_.setIdentity();
  predicted_pose_.setIdentity();
  odom_history_.resize(MAX_ODOM_HISTORY);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // **** publishers
  if (publish_constructed_scan_ && use_map_)
  {
    constructed_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "lsm_localization/constructed_scan", 5);
  }

  if (publish_pose_)
  {
    pose_publisher_  = this->create_publisher<geometry_msgs::msg::Pose2D>(
      "lsm_localization/pose2D", 5);
  }

  if (publish_pose_stamped_)
  {
    pose_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "lsm_localization/pose_stamped", 5);
  }

  if (publish_pose_with_covariance_)
  {
    pose_with_covariance_publisher_  = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>(
      "lsm_localization/pose_with_covariance", 5);
  }

  if (publish_pose_with_covariance_stamped_)
  {
    pose_with_covariance_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "lsm_localization/pose", 5);
  }

  if (publish_predicted_pose_)
  {
    predicted_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "lsm_localization/predicted_pose", 5);
  }

  if (publish_measured_pose_)
  {
    measured_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("lsm_localization/measured_pose", 5);
  }

  if (publish_debug_)
  {
    debug_odom_delta_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "lsm_localization/debug/odom_delta", 5);
    debug_laser_delta_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "lsm_localization/debug/laser_delta", 5);
    debug_odom_reference_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "lsm_localization/debug/odom_reference", 5);
    debug_odom_current_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "lsm_localization/debug/odom_current", 5);
  }

  // *** subscribers

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>( "odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(msg);
  });

  initialpose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>( "pose_topic", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        this->initialposeCallback(msg);
  });


  if (use_map_)
  {
    RCLCPP_INFO(this->get_logger(), "using map-based matching");
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>( "map", 1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->mapCallback(msg);
    });

  }
  else
    RCLCPP_INFO(this->get_logger(), "using frame-to-frame matching");

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>( "scan", 1, 
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->scanCallback(msg);
  });
}

LaserScanMatcher::~LaserScanMatcher()
{
  RCLCPP_INFO(this->get_logger(), "Destroying LaserScanMatcher");
  gsl_matrix_free(Sigma_odom_);
  gsl_matrix_free(Sigma_odom_trans_);
  gsl_matrix_free(Sigma_measured_);
  gsl_matrix_free(B_odom_);
  gsl_matrix_free(I1_);
  gsl_matrix_free(I2_);
  gsl_permutation_free(P2_);
  gsl_matrix_free(trans_sigma_);
  gsl_matrix_free(kalman_gain_);
  gsl_matrix_free(kalman_gain_comp_);
  gsl_vector_free(xvec_);
  gsl_vector_free(yvec_);
}

void LaserScanMatcher::resetState()
{
  RCLCPP_INFO(this->get_logger(), "LaserScanMatcher state cleared");

  // **** state variables

  constructed_intensities_.clear();
  constructed_ranges_.clear();
  initialpose_valid_ = false;
  initial_pose_.setIdentity();
  predicted_pose_in_pcl_.setIdentity();

  f2b_.setIdentity();
  f2b_kf_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;
}

void LaserScanMatcher::initParams()
{
  if (!this->get_parameter ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!this->get_parameter ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";
  if (!this->get_parameter ("initialpose_topic_name", initialpose_topic_))
    initialpose_topic_ = "initialpose";
  if (!this->get_parameter("scan_downsample_rate", scan_downsample_rate_))
    scan_downsample_rate_ = 1;
  if (scan_downsample_rate_ <= 0)
    scan_downsample_rate_ = 1;

  // **** keyframe params: when to generate the keyframe scan
  // if either is set to 0, reduces to frame-to-frame matching

  if (!this->get_parameter ("kf_dist_linear", kf_dist_linear_))
    kf_dist_linear_ = 0.10;
  if (!this->get_parameter ("kf_dist_angular", kf_dist_angular_))
    kf_dist_angular_ = 10.0 * (M_PI / 180.0);

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  if (!this->get_parameter ("no_odom_fusing", no_odom_fusing_))
    no_odom_fusing_ = false;

  // **** Parameters that control map-to-scan matching
  // use_map must be true for map-to-scan matching to be used
  // other parameters control how the initial-pose scan is constructed
  if (!this->get_parameter ("use_map", use_map_))
    use_map_ = true;
  if (!this->get_parameter ("map_occupancy_threshold", map_occupancy_threshold_))
    map_occupancy_threshold_ = 10.0;
  // min and max range for constructing the scan that initial pose
  // would see
  if (!this->get_parameter("max_allowed_range", max_allowed_range_))
    max_allowed_range_ = -1;
  if (!this->get_parameter ("max_variance_trans", max_variance_trans_))
    max_variance_trans_ = 1e-5;
  if (!this->get_parameter ("max_variance_rot", max_variance_rot_))
    max_variance_rot_ = 1e-5;
  if (!this->get_parameter ("max_pose_delta_yaw", max_pose_delta_yaw_))
    max_pose_delta_yaw_ = 0.785; // 45 degrees

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)
  if (!this->get_parameter ("publish_base_tf", publish_base_tf_))
    publish_base_tf_ = false;
  if (!this->get_parameter ("publish_odom_tf", publish_odom_tf_))
    publish_odom_tf_ = true;
  if (!this->get_parameter ("publish_pose", publish_pose_))
    publish_pose_ = true;
  if (!this->get_parameter ("publish_constructed_scan", publish_constructed_scan_))
    publish_constructed_scan_ = false;
  if (!this->get_parameter ("publish_pose_stamped", publish_pose_stamped_))
    publish_pose_stamped_ = false;
  if (!this->get_parameter ("publish_pose_with_covariance", publish_pose_with_covariance_))
    publish_pose_with_covariance_ = false;
  if (!this->get_parameter ("publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped_))
    publish_pose_with_covariance_stamped_ = true;
  if (!this->get_parameter ("publish_predicted_pose", publish_predicted_pose_))
    publish_predicted_pose_ = false;
  if (!this->get_parameter ("publish_measured_pose", publish_measured_pose_))
    publish_measured_pose_ = false;
  if (!this->get_parameter ("publish_debug", publish_debug_))
    publish_debug_ = false;
  if (!this->get_parameter ("debug_csm", debug_csm_))
    debug_csm_ = false;

  if (!this->get_parameter("position_covariance", position_covariance_))
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  }

  if (!this->get_parameter("orientation_covariance", orientation_covariance_))
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!this->get_parameter ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!this->get_parameter ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!this->get_parameter ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!this->get_parameter ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!this->get_parameter ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!this->get_parameter ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!this->get_parameter ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!this->get_parameter ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!this->get_parameter ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!this->get_parameter ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!this->get_parameter ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!this->get_parameter ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!this->get_parameter ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!this->get_parameter ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!this->get_parameter ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!this->get_parameter ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!this->get_parameter ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!this->get_parameter ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!this->get_parameter ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!this->get_parameter ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!this->get_parameter ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!this->get_parameter ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // Compute the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  input_.do_compute_covariance = 1;

  // Checks that find_correspondences_tricks gives the right answer
  if (!this->get_parameter ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!this->get_parameter ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!this->get_parameter ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

nav_msgs::msg::Odometry* LaserScanMatcher::latestOdomBefore(const rclcpp::Time& time)
{
    for (auto o=odom_history_.begin(); o < odom_history_.end(); o++) {
        if (rclcpp::Time(o->header.stamp) <= time)
            return &(*o);
    }
    return NULL;
}

nav_msgs::msg::Odometry* LaserScanMatcher::earliestOdomAfter(const rclcpp::Time& time)
{
    for (auto o=odom_history_.begin(); o < odom_history_.end(); o++) {
        if (rclcpp::Time(o->header.stamp) == time) {
            // found exact match, that's it!
            return &(*o);
        } else if (rclcpp::Time(o->header.stamp) < time) {
            if (o==odom_history_.begin()) {
                // beginning of queue is already before (i.e. nothing after)
                return NULL;
            } else {
                // found latest before, so earliest after is one before that
                return &(*(o-1));
            }
        }
    }
    return &(*odom_history_.end());
}

void LaserScanMatcher::addOdomToHistory(const nav_msgs::msg::Odometry::SharedPtr& o)
{
    if (odom_history_.size() >= MAX_ODOM_HISTORY)
        odom_history_.pop_back();
    odom_history_.push_front(*o);
}

double LaserScanMatcher::syncOdom(const rclcpp::Time& time)
{
    nav_msgs::msg::Odometry* latest_before = latestOdomBefore(time);
    if (!latest_before) {
        RCLCPP_WARN(this->get_logger(), "missing latest odom before t=%.3f (is odom topic alive)",
                 time.seconds());
        return -1.0;
    }
    nav_msgs::msg::Odometry o = *latest_before;
    o.header.stamp = time;
    tf2::Quaternion q;
    // extrapolate because we don't have the second point
    double delta_t = (time - latest_before->header.stamp).seconds();
    if (delta_t > MAX_ODOM_AGE)
        return -1.0;
    o.pose.pose.position.x = latest_before->pose.pose.position.x +
        delta_t * latest_before->twist.twist.linear.x;
    o.pose.pose.position.y = latest_before->pose.pose.position.y +
        delta_t * latest_before->twist.twist.linear.y;
    o.pose.pose.position.z = latest_before->pose.pose.position.z +
        delta_t * latest_before->twist.twist.linear.z;
    tf2::Quaternion delta_q;
    delta_q.setRPY(delta_t * latest_before->twist.twist.angular.x,
                   delta_t * latest_before->twist.twist.angular.y,
                   delta_t * latest_before->twist.twist.angular.z);
    tf2::Quaternion qb(latest_before->pose.pose.orientation.x,
                      latest_before->pose.pose.orientation.y,
                      latest_before->pose.pose.orientation.z,
                      latest_before->pose.pose.orientation.w);
    q = delta_q * qb;
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    o.pose.pose.orientation.w = q.w();
    current_odom_msg_ = o;
    return delta_t;
}

double LaserScanMatcher::getOdomDeltaT(const nav_msgs::msg::Odometry::SharedPtr& o)
{
  if (odom_history_.empty())
    return -1.0;

  rclcpp::Time t1(o->header.stamp);
  rclcpp::Time t0(odom_history_.front().header.stamp);
  return (t1 - t0).seconds();
}

void LaserScanMatcher::doPredictPose(double delta_t)
{
  tf2::Transform current_odom_tf;
  tf2::Transform reference_odom_tf;
  createTfFromXYTheta(current_odom_msg_.pose.pose.position.x,
                      current_odom_msg_.pose.pose.position.y,
                      tf2::getYaw(current_odom_msg_.pose.pose.orientation),
                      current_odom_tf);
  createTfFromXYTheta(reference_odom_msg_.pose.pose.position.x,
                      reference_odom_msg_.pose.pose.position.y,
                      tf2::getYaw(reference_odom_msg_.pose.pose.orientation),
                      reference_odom_tf);
  tf2::Transform delta_odom_tf = reference_odom_tf.inverse() * current_odom_tf;

  // apply calculated delta to the reference pose
  predicted_pose_ =
    initial_pose_ * base_to_footprint_ * delta_odom_tf * footprint_to_base_;

  // construct input covariance (see the paper)
  // matrix is pre-cleared, so we don't have to touch what's always zero
  gsl_matrix_set(Sigma_u_, 0, 0, current_odom_msg_.twist.covariance[0]);
  gsl_matrix_set(Sigma_u_, 0, 1, current_odom_msg_.twist.covariance[1]);
  gsl_matrix_set(Sigma_u_, 1, 0, current_odom_msg_.twist.covariance[6]);
  gsl_matrix_set(Sigma_u_, 1, 1, current_odom_msg_.twist.covariance[7]);
  gsl_matrix_set(Sigma_u_, 2, 2, gsl_matrix_get(Sigma_odom_, 2, 2));
  gsl_matrix_set(Sigma_u_, 2, 3,
                 gsl_matrix_get(Sigma_odom_, 2, 2) *
                 current_odom_msg_.twist.twist.linear.x *
                 current_odom_msg_.twist.twist.linear.y);
  gsl_matrix_set(Sigma_u_, 3, 2,
                 gsl_matrix_get(Sigma_odom_, 2, 2) *
                 current_odom_msg_.twist.twist.linear.x *
                 current_odom_msg_.twist.twist.linear.y);
  gsl_matrix_set(Sigma_u_, 3, 3, gsl_matrix_get(Sigma_odom_, 2, 2));
  gsl_matrix_set(Sigma_u_, 4, 4, current_odom_msg_.twist.covariance[35]);

  // construct B-matrix
  theta_odom_ += delta_t * current_odom_msg_.twist.twist.angular.z;
  double cos0 = cos(theta_odom_);
  double sin0 = sin(theta_odom_);
  double cos90 = cos(theta_odom_ + M_PI/2.0);
  double sin90 = sin(theta_odom_ + M_PI/2.0);
  gsl_matrix_set(B_odom_, 0, 0,  cos0);
  gsl_matrix_set(B_odom_, 0, 1, -sin0);
  gsl_matrix_set(B_odom_, 1, 0,  sin0);
  gsl_matrix_set(B_odom_, 1, 1,  cos0);
  gsl_matrix_set(B_odom_, 0, 2,  cos90);
  gsl_matrix_set(B_odom_, 0, 3, -sin90);
  gsl_matrix_set(B_odom_, 1, 2,  sin90);
  gsl_matrix_set(B_odom_, 1, 3,  cos90);
  // I1 = delta_t * Sigma_u * B_odom^T + 0 * I1
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, delta_t, Sigma_u_, B_odom_, 0.0, I1_);
  // Sigma_odom = delta_t * B_odom * I1 + 1 * Sigma_odom (use beta = 1.0 to accumulate)
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, delta_t, B_odom_, I1_, 1.0, Sigma_odom_);

  if (publish_debug_) {
    doPublishDebugTF(current_odom_msg_.header.stamp, delta_odom_tf, debug_odom_delta_publisher_, "");
    doPublishDebugTF(current_odom_msg_.header.stamp, reference_odom_tf, debug_odom_reference_publisher_, "odom");
    doPublishDebugTF(current_odom_msg_.header.stamp, current_odom_tf, debug_odom_current_publisher_, "odom");
  }
}

void LaserScanMatcher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr& odom_msg)
{
  std::scoped_lock lock(mutex_);
  double delta_t = getOdomDeltaT(odom_msg);
  addOdomToHistory(odom_msg);
  if (!received_odom_)
  {
    if (!getBaseToFootprintTf(odom_msg->child_frame_id)) {
      RCLCPP_WARN(this->get_logger(), "skipping odom");
      return;
    }
    reference_odom_msg_ = *odom_msg;
    current_odom_msg_ = *odom_msg;
    gsl_matrix_set_zero(Sigma_odom_);
    theta_odom_ = 0.0;
    received_odom_ = true;
    odom_frame_ = odom_msg->header.frame_id;
    return;
  }

  // if we got here, we have what we need to do the integration
  assert (delta_t >= 0.0);
  current_odom_msg_ = *odom_msg;
  doPredictPose(delta_t);
  doPublishOdomRate(odom_msg->header.stamp);
}

void LaserScanMatcher::setTransSigmaMatrix(double yaw)
{
  gsl_matrix_set(trans_sigma_, 0, 0, cos(yaw));
  gsl_matrix_set(trans_sigma_, 0, 1, -sin(yaw));
  gsl_matrix_set(trans_sigma_, 1, 0, sin(yaw));
  gsl_matrix_set(trans_sigma_, 1, 1, cos(yaw));
}

void LaserScanMatcher::constructScan(const rclcpp::Time& time)
{
  ScanConstructor::scan_params_t params;
  params.range_max = observed_range_max_;
  params.range_min = observed_range_min_;
  params.angle_max = observed_angle_max_;
  params.angle_min = observed_angle_min_;
  params.angle_inc = observed_angle_inc_;

  params.max_allowed_range = max_allowed_range_;

  predicted_pose_in_pcl_ = pcl2f_ * predicted_pose_;
  tf2::Transform predicted_laser_pose_in_pcl = predicted_pose_in_pcl_ * base_to_laser_;
  double laser_x = predicted_laser_pose_in_pcl.getOrigin().getX();
  double laser_y = predicted_laser_pose_in_pcl.getOrigin().getY();
  double laser_yaw = tf2::getYaw(predicted_laser_pose_in_pcl.getRotation());
  RCLCPP_DEBUG(this->get_logger(), "%s: laser_x=%f, laser_y=%f laser_yaw (deg)=%f", __func__,
            laser_x, laser_y, 180.0 * laser_yaw / M_PI);
  RCLCPP_DEBUG(this->get_logger(), "%s: range=[%f,%f], angle=[%f,%f]@%f", __func__,
            params.range_min, params.range_max,
            params.angle_min, params.angle_max, params.angle_inc);

  constructed_ranges_ =
    scan_constructor_.constructScan(laser_x, laser_y, laser_yaw, params);

  constructed_intensities_.clear();
  constructed_intensities_.reserve(constructed_intensities_.size());

  for (const auto r : constructed_ranges_)
    constructed_intensities_.push_back(r > 0.0? 100.0:0.0);

  if (publish_constructed_scan_) {
    sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
    scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->range_min = params.range_min;
    scan_msg->range_max = params.range_max;
    scan_msg->angle_min = params.angle_min;
    scan_msg->angle_max = params.angle_max;
    scan_msg->angle_increment = params.angle_inc;
    scan_msg->scan_time = observed_scan_time_;
    scan_msg->time_increment = observed_time_inc_;
    scan_msg->header.stamp = time;
    scan_msg->header.frame_id = observed_scan_frame_;
    scan_msg->ranges.assign(
      constructed_ranges_.begin(), constructed_ranges_.end());
    scan_msg->intensities.assign(
      constructed_intensities_.begin(), constructed_intensities_.end());;

    constructed_scan_publisher_->publish(*scan_msg);
  }
}

void LaserScanMatcher::initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& pose_msg)
{
  std::scoped_lock lock(mutex_);
  tf2::Transform pose;

  if (!have_map_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "map not loaded, cannot process initial pose");
    return;
  }
  if (!received_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "odom never received, cannot process initial pose");
    return;
  }
  if (syncOdom(pose_msg->header.stamp) < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "cannot sync odometry for initial pose timestamp");
    return;
  }
  reference_odom_msg_ = current_odom_msg_;
  gsl_matrix_set_zero(Sigma_odom_);
  theta_odom_ = 0.0;

  // convert the input pose (typically in 'map' frame into lsm_fixed frame)
  pose.setOrigin(tf2::Vector3(pose_msg->pose.pose.position.x,
                             pose_msg->pose.pose.position.y,
                             pose_msg->pose.pose.position.z));
  pose.setRotation(tf2::Quaternion(pose_msg->pose.pose.orientation.x,
                                  pose_msg->pose.pose.orientation.y,
                                  pose_msg->pose.pose.orientation.z,
                                  pose_msg->pose.pose.orientation.w));
  if (pose_msg->header.frame_id != fixed_frame_)
    RCLCPP_WARN(this->get_logger(), "incorrect frame for initial pose: '%s' vs. '%s'",
             pose_msg->header.frame_id.c_str(), fixed_frame_.c_str());

  // new initial pose came in, set the predicted pose to be equal
  // to it and set the reference odom to be the current odom
  initial_pose_ = pose;
  predicted_pose_in_pcl_ = pcl2f_ * initial_pose_;
  initialpose_valid_ = true;
  // reflect the incoming initial pose if set up to publish
  // on compatible topic
  if (publish_pose_with_covariance_stamped_) {
    pose_with_covariance_stamped_publisher_->publish(*pose_msg);
  }
}

void LaserScanMatcher::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
{
  std::scoped_lock lock(mutex_);

  ScanConstructor::map_params_t map_params;

  map_params.map_occupancy_threshold = map_occupancy_threshold_;
  map_params.map_res = map_msg->info.resolution;
  map_params.map_width = map_msg->info.width;
  map_params.map_height = map_msg->info.height;
  f2pcl_.setOrigin(tf2::Vector3(map_msg->info.origin.position.x,
                               map_msg->info.origin.position.y,
                               map_msg->info.origin.position.z));
  f2pcl_.setRotation(tf2::Quaternion(map_msg->info.origin.orientation.x,
                                    map_msg->info.origin.orientation.y,
                                    map_msg->info.origin.orientation.z,
                                    map_msg->info.origin.orientation.w));
  pcl2f_ = f2pcl_.inverse();

  ScanConstructor::grid_t map_grid;
  map_grid.resize(map_params.map_height);

  int i = 0;
  for (auto row=map_grid.begin(); row < map_grid.end(); row++) {
      row->resize(map_params.map_width);
      for (auto element=row->begin(); element < row->end(); element++)
          *element = map_msg->data[i++];
  }
  RCLCPP_INFO(this->get_logger(), "got map: %dx%d@%f",
           map_params.map_width, map_params.map_height, map_params.map_res);

  scan_constructor_ = ScanConstructor(std::move(map_grid), map_params);

  have_map_ = true;
}

void LaserScanMatcher::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg)
{
  std::scoped_lock lock(mutex_);

  // **** if first scan, cache the tf from base to the scanner
  if (!initialized_)
  {
    RCLCPP_INFO(this->get_logger(), "first scan observed, initializing");

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      RCLCPP_WARN(this->get_logger(), "Skipping scan");
      return;
    }
    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;

    laserScanToLDP(scan_msg, prev_ldp_scan_);
    last_icp_time_ = scan_msg->header.stamp;
    observed_range_min_ = scan_msg->range_min;
    observed_range_max_ = scan_msg->range_max;
    observed_angle_min_ = scan_msg->angle_min;
    observed_angle_max_ = scan_msg->angle_max;
    observed_angle_inc_ = scan_msg->angle_increment;
    observed_scan_time_ = scan_msg->scan_time;
    observed_time_inc_ = scan_msg->time_increment;
    observed_scan_frame_ = scan_msg->header.frame_id;
    initialized_ = true;
  }
  
  if (++scan_counter_ % scan_downsample_rate_ != 0)
      return;
  rclcpp::Time start = this->get_clock()->now();
  LDP curr_ldp_scan;
  int r = 0;
  double delta_t;
  if ((delta_t = syncOdom(scan_msg->header.stamp)) < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "odometry sync failed, skipping scan");
    return;
  }
  // run prediction once again here, so that we end up using
  // synced up odometry
  doPredictPose(delta_t);
  if (use_map_) {
    if (initialpose_valid_) {
      // if the reference frame comes from the map, replace it
      LDP ref_pose_ldp_scan;
      constructScan(scan_msg->header.stamp);
      constructedScanToLDP(ref_pose_ldp_scan);
      laserScanToLDP(scan_msg, curr_ldp_scan);
      r = processScan(curr_ldp_scan, ref_pose_ldp_scan, scan_msg->header.stamp);
      if (r) {
        // if localization was successful, use the estimated pose
        // as the reference for next time

        initial_pose_ = f2b_;
        predicted_pose_in_pcl_ = pcl2f_ * initial_pose_;
        initialpose_valid_ = true;
        reference_odom_msg_ = current_odom_msg_;
        gsl_matrix_set_zero(Sigma_odom_);
        theta_odom_ = 0.0;
      }
    } else
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 20000,
              "initial pose not received yet, scan processing skipped");
  } else {
    laserScanToLDP(scan_msg, curr_ldp_scan);
    r = processScan(curr_ldp_scan, scan_msg->header.stamp);
  }
  if (r) {
    rclcpp::Time end = this->get_clock()->now();
    rclcpp::Duration duration = end - start;
    int64_t duration_ms = duration.nanoseconds() / 1e6;

    RCLCPP_DEBUG(this->get_logger(), ":complete scan processing total duration: %ld ms", static_cast<long>(duration_ms));
  }
}

void LaserScanMatcher::doPublishDebugTF(
    const rclcpp::Time& time,
    const tf2::Transform& transform,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
    const std::string& frame
)
{
    auto pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    
    pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(
        transformToPoseStamped(transform, frame, time));

    publisher->publish(*pose_stamped_msg);
}

void LaserScanMatcher::doPublishOdomRate(const rclcpp::Time& time)
{
  if (publish_predicted_pose_) {
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_with_covariance_stamped_msg;
    pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg->header.stamp    = time;
    pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

    pose_with_covariance_stamped_msg->pose.pose = transformToPose(predicted_pose_);

    pose_with_covariance_stamped_msg->pose.covariance = {
      gsl_matrix_get(Sigma_odom_trans_, 0, 0), gsl_matrix_get(Sigma_odom_trans_, 0, 1), 0, 0, 0, 0,
      gsl_matrix_get(Sigma_odom_trans_, 1, 0), gsl_matrix_get(Sigma_odom_trans_, 1, 1), 0, 0, 0, 0,
      0, 0, static_cast<double>(position_covariance_[2]), 0, 0, 0,
      0, 0, 0, static_cast<double>(orientation_covariance_[0]), 0, 0,
      0, 0, 0, 0, static_cast<double>(orientation_covariance_[1]), 0,
      0, 0, 0, 0, 0, gsl_matrix_get(Sigma_odom_trans_, 2, 2)
    };
    predicted_pose_publisher_->publish(*pose_with_covariance_stamped_msg);
  }
}

void LaserScanMatcher::doPublishScanRate(const rclcpp::Time& time)
{
  if (publish_pose_) {
    // unstamped Pose2D message
    auto pose_msg = std::make_shared<geometry_msgs::msg::Pose2D>();
    pose_msg->x = f2b_.getOrigin().getX();
    pose_msg->y = f2b_.getOrigin().getY();
    pose_msg->theta = tf2::getYaw(f2b_.getRotation());
    pose_publisher_->publish(*pose_msg);
  }
  if (publish_pose_stamped_) {
    // stamped Pose message
    auto pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    
    pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>(
        transformToPoseStamped(f2b_, fixed_frame_, time));

    pose_stamped_publisher_->publish(*pose_stamped_msg);
  }
  if (publish_pose_with_covariance_) {
    // unstamped PoseWithCovariance message
    auto pose_with_covariance_msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();

    pose_with_covariance_msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>(
        transformToPoseWithCovariance(f2b_));

    pose_with_covariance_msg->covariance = {
      gsl_matrix_get(output_.cov_x_m, 0, 0), gsl_matrix_get(output_.cov_x_m, 0, 1), 0, 0, 0, 0,
      gsl_matrix_get(output_.cov_x_m, 1, 0), gsl_matrix_get(output_.cov_x_m, 1, 1), 0, 0, 0, 0,
      0, 0, static_cast<double>(position_covariance_[2]), 0, 0, 0,
      0, 0, 0, static_cast<double>(orientation_covariance_[0]), 0, 0,
      0, 0, 0, 0, static_cast<double>(orientation_covariance_[1]), 0,
      0, 0, 0, 0, 0, gsl_matrix_get(output_.cov_x_m, 2, 2)
    };
    pose_with_covariance_publisher_->publish(*pose_with_covariance_msg);
  }
  if (publish_pose_with_covariance_stamped_) {
    // stamped Pose message
    auto pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
        transformToPoseWithCovarianceStamped(f2b_, fixed_frame_, time));

    pose_with_covariance_stamped_msg->pose.covariance = {
      gsl_matrix_get(output_.cov_x_m, 0, 0), gsl_matrix_get(output_.cov_x_m, 0, 1), 0, 0, 0, 0,
      gsl_matrix_get(output_.cov_x_m, 1, 0), gsl_matrix_get(output_.cov_x_m, 1, 1), 0, 0, 0, 0,
      0, 0, static_cast<double>(position_covariance_[2]), 0, 0, 0,
      0, 0, 0, static_cast<double>(orientation_covariance_[0]), 0, 0,
      0, 0, 0, 0, static_cast<double>(orientation_covariance_[1]), 0,
      0, 0, 0, 0, 0, gsl_matrix_get(output_.cov_x_m, 2, 2)
    };
    pose_with_covariance_stamped_publisher_->publish(*pose_with_covariance_stamped_msg);
  }

  if (publish_measured_pose_) {
    auto pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
        transformToPoseWithCovarianceStamped(f2b_, fixed_frame_, time));

    pose_with_covariance_stamped_msg->pose.covariance = {
      gsl_matrix_get(Sigma_measured_, 0, 0), gsl_matrix_get(Sigma_measured_, 0, 1), 0, 0, 0, 0,
      gsl_matrix_get(Sigma_measured_, 1, 0), gsl_matrix_get(Sigma_measured_, 1, 1), 0, 0, 0, 0,
      0, 0, static_cast<double>(position_covariance_[2]), 0, 0, 0,
      0, 0, 0, static_cast<double>(orientation_covariance_[0]), 0, 0,
      0, 0, 0, 0, static_cast<double>(orientation_covariance_[1]), 0,
      0, 0, 0, 0, 0, gsl_matrix_get(Sigma_measured_, 2, 2)
    };
    measured_pose_publisher_->publish(*pose_with_covariance_stamped_msg);
  }

  if (publish_base_tf_) {
    auto transform_msg = createTransformStamped(f2b_, time, fixed_frame_, base_frame_);
    tf_broadcaster_->sendTransform (transform_msg);
  }

  if (publish_odom_tf_) {
    tf2::Transform current_odom_tf;
    tf2::Transform m2o;
    createTfFromXYTheta(current_odom_msg_.pose.pose.position.x,
			current_odom_msg_.pose.pose.position.y,
			tf2::getYaw(current_odom_msg_.pose.pose.orientation),
			current_odom_tf);
    m2o = f2b_ * (current_odom_tf * footprint_to_base_).inverse();
    auto transform_msg = createTransformStamped(m2o, time, fixed_frame_, odom_frame_);
    tf_broadcaster_->sendTransform (transform_msg);
  }

}

tf2::Vector3 LaserScanMatcher::fusePoses(const tf2::Transform& pose_delta)
{
  int s;

  measured_pose_ = f2pcl_ * predicted_pose_in_pcl_ * pose_delta;
  gsl_matrix_memcpy(Sigma_measured_, output_.cov_x_m);
  gsl_matrix_add(output_.cov_x_m, Sigma_odom_trans_);
  // in gsl, matrix inversion goes via LU decomposition
  gsl_linalg_LU_decomp(output_.cov_x_m, P2_, &s);
  gsl_linalg_LU_invert(output_.cov_x_m, P2_, I2_);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
    1.0, Sigma_odom_trans_, I2_, 0.0, kalman_gain_);
  gsl_matrix_set_identity(kalman_gain_comp_);
  gsl_matrix_sub(kalman_gain_comp_, kalman_gain_);

  // Sigma = (I-K)Sigma_odom
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
    1.0, kalman_gain_comp_, Sigma_odom_trans_, 0.0, output_.cov_x_m);
  double measured_pose_yaw = tf2::getYaw(measured_pose_.getRotation());
  double predicted_pose_yaw = tf2::getYaw(predicted_pose_.getRotation());
  // handle discontinuity around +/- PI
  if (measured_pose_yaw < -M_PI/2.0 && predicted_pose_yaw > M_PI/2.0)
    measured_pose_yaw += 2*M_PI;
  else if (measured_pose_yaw > M_PI/2.0  && predicted_pose_yaw < -M_PI/2.0)
    predicted_pose_yaw += 2*M_PI;
  // y = K*measured
  gsl_vector_set(xvec_, 0, measured_pose_.getOrigin().getX());
  gsl_vector_set(xvec_, 1, measured_pose_.getOrigin().getY());
  gsl_vector_set(xvec_, 2, measured_pose_yaw);
  gsl_blas_dgemv(CblasNoTrans, 1.0, kalman_gain_, xvec_, 0.0, yvec_);
  // y += (I-K)*predicted
  gsl_vector_set(xvec_, 0, predicted_pose_.getOrigin().getX());
  gsl_vector_set(xvec_, 1, predicted_pose_.getOrigin().getY());
  gsl_vector_set(xvec_, 2, predicted_pose_yaw);
  gsl_blas_dgemv(CblasNoTrans, 1.0, kalman_gain_comp_, xvec_, 1.0, yvec_);

  RCLCPP_DEBUG(this->get_logger(), "kalman_gain:\n%e %e %e\n%e %e %e\n%e %e %e",
            gsl_matrix_get(kalman_gain_, 0, 0),
            gsl_matrix_get(kalman_gain_, 0, 1),
            gsl_matrix_get(kalman_gain_, 0, 2),
            gsl_matrix_get(kalman_gain_, 1, 0),
            gsl_matrix_get(kalman_gain_, 1, 1),
            gsl_matrix_get(kalman_gain_, 1, 2),
            gsl_matrix_get(kalman_gain_, 2, 0),
            gsl_matrix_get(kalman_gain_, 2, 1),
            gsl_matrix_get(kalman_gain_, 2, 2));
  return tf2::Vector3(
    gsl_vector_get(yvec_, 0),
    gsl_vector_get(yvec_, 1),
    gsl_vector_get(yvec_, 2));
}

int LaserScanMatcher::processScan(LDP& curr_ldp_scan, LDP& ref_ldp_scan, const rclcpp::Time& time)
{
  rclcpp::Time start = this->get_clock()->now();

  ref_ldp_scan->odometry[0] = 0.0;
  ref_ldp_scan->odometry[1] = 0.0;
  ref_ldp_scan->odometry[2] = 0.0;
  ref_ldp_scan->estimate[0] = 0.0;
  ref_ldp_scan->estimate[1] = 0.0;
  ref_ldp_scan->estimate[2] = 0.0;
  ref_ldp_scan->true_pose[0] = 0.0;
  ref_ldp_scan->true_pose[1] = 0.0;
  ref_ldp_scan->true_pose[2] = 0.0;

  input_.laser_ref  = ref_ldp_scan;
  input_.laser_sens = curr_ldp_scan;
  input_.first_guess[0] = 0.0;
  input_.first_guess[1] = 0.0;
  input_.first_guess[2] = 0.0;

  // clear old covariance gsl matrices
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM
  try {
    sm_icp(&input_, &output_);
  } catch(int e) {
    if (e == GSL_EDOM) {
      RCLCPP_ERROR(this->get_logger(), "sm_icp failed, probably singular matrix");
      output_.valid = false;
    } else
      throw e;
  }
  if (output_.valid) {
    if (gsl_matrix_get(output_.cov_x_m, 0, 0) < max_variance_trans_ &&
	gsl_matrix_get(output_.cov_x_m, 1, 1) < max_variance_trans_ &&
	gsl_matrix_get(output_.cov_x_m, 2, 2) < max_variance_rot_) {
      // the correction of the laser's position, in the laser frame
      RCLCPP_DEBUG(this->get_logger(), "found correlation transform: x=%f, y=%f, yaw=%f",
               output_.x[0], output_.x[1], 180.0 * output_.x[2] / M_PI);
      RCLCPP_DEBUG(this->get_logger(), "variances: %f, %f, %f",
		gsl_matrix_get(output_.cov_x_m, 0, 0),
		gsl_matrix_get(output_.cov_x_m, 1, 1),
		gsl_matrix_get(output_.cov_x_m, 2, 2));
      tf2::Transform pose_delta_laser;
      createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2],
                          pose_delta_laser);
      // predicted pose is pcl-to-base_link
      // delta is crrection of predicted pose, right two
      // operands get us corrected pcl-to-base_link
      // multiply by map-to-pcl on the left to get
      // map-to-base_link, which is the pose we are looking for
      if (output_.x[2] > max_pose_delta_yaw_ ||
          output_.x[2] < -max_pose_delta_yaw_) {
        RCLCPP_WARN(this->get_logger(), "laser pose delta max yaw exceeded %f", output_.x[2]);
        ld_free(curr_ldp_scan);
        ld_free(ref_ldp_scan);
        return 0;
      }
      tf2::Transform pose_delta =
        base_to_laser_ * pose_delta_laser * laser_to_base_;
      
      if (publish_debug_)
        doPublishDebugTF(time, pose_delta, debug_laser_delta_publisher_, "");

      // Sigma_odom is the covariance of odometry-delta
      // we need covariance in the map frame, so apply transforms
      setTransSigmaMatrix(tf2::getYaw((initial_pose_ * base_to_laser_).getRotation()));
      gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0,
		     Sigma_odom_, trans_sigma_, 0.0, I2_);
      gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, trans_sigma_,
		     I2_, 0.0, Sigma_odom_trans_);
      gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0,
		     output_.cov_x_m, trans_sigma_, 0.0, I2_);
      gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, trans_sigma_,
		     I2_, 0.0, output_.cov_x_m);
      if (!no_odom_fusing_) {
	tf2::Vector3 pv = fusePoses(pose_delta);
	createTfFromXYTheta(pv.getX(), pv.getY(), pv.getZ(), f2b_);
      } else {
	f2b_ = f2pcl_ * predicted_pose_in_pcl_ * pose_delta;
      }
      doPublishScanRate(time);
      rclcpp::Time end = this->get_clock()->now();
      rclcpp::Duration duration = end - start;
      int64_t duration_ms = duration.nanoseconds() / 1e6;
      RCLCPP_DEBUG(this->get_logger(), "scan matcher duration: %ld ms, iterations: %d",
            static_cast<long>(duration_ms), output_.iterations);
      ld_free(curr_ldp_scan);
      ld_free(ref_ldp_scan);
      return 1;
    } else {
      skipped_poses_ += 1;
      RCLCPP_WARN_THROTTLE( this->get_logger(), *this->get_clock(), 10000,  
            "Variance of measured pose exceeded limit, not publishing. (%d poses skipped)",
            skipped_poses_);
      ld_free(curr_ldp_scan);
      ld_free(ref_ldp_scan);
      return 0;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Error in scan matching");
    ld_free(curr_ldp_scan);
    ld_free(ref_ldp_scan);
    return 0;
  }
}

int LaserScanMatcher::processScan(LDP& curr_ldp_scan, const rclcpp::Time& time)
{
  rclcpp::Time start = this->get_clock()->now();
  int success = 0;

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

  // **** estimated change since last scan

  double dt = (time - last_icp_time_).seconds();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the fixed frame

  tf2::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf2::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM
  sm_icp(&input_, &output_);
  tf2::Transform corr_ch;

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf2::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;

    doPublishScanRate(time);
    success = 1;
  }
  else
  {
    corr_ch.setIdentity();
    RCLCPP_WARN(this->get_logger(), "Error in scan matching");
  }

  // **** swap old and new

  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
  }
  else
  {
    ld_free(curr_ldp_scan);
  }

  last_icp_time_ = time;

  // **** statistics

  rclcpp::Time end = this->get_clock()->now();
  rclcpp::Duration duration = end - start;
  int64_t duration_ms = duration.nanoseconds() / 1e6;
  RCLCPP_INFO(this->get_logger(), "Scan matcher total duration: %ld ms", static_cast<long>(duration_ms));

  return success;
}

bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform& d)
{
  if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_) return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg, LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max &&
        (max_allowed_range_ <= 0 || r <= max_allowed_range_))
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

void LaserScanMatcher::constructedScanToLDP(LDP& ldp)
{
  int n = constructed_intensities_.size();
  ldp = ld_alloc_new(n);

  for (int i = 0; i < n; i++) {
    if (constructed_ranges_[i] > 0.0) {
      ldp->valid[i] = 1;
      ldp->readings[i] = constructed_ranges_[i];
    } else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;
    }
    ldp->theta[i] = observed_angle_min_ + i * observed_angle_inc_;
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

bool LaserScanMatcher::getBaseToLaserTf(const std::string& frame_id)
{
  rclcpp::Time t = this->get_clock()->now();

  try {
    if (!tf_buffer_->canTransform(base_frame_, frame_id, t, rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available yet", frame_id.c_str(), base_frame_.c_str());
      return false;
    }

    geometry_msgs::msg::TransformStamped transform_msg =
      tf_buffer_->lookupTransform(base_frame_, frame_id, t);

    tf2::Transform base_to_laser_tf;
    tf2::fromMsg(transform_msg.transform, base_to_laser_tf);

    base_to_laser_ = base_to_laser_tf;
    laser_to_base_ = base_to_laser_.inverse();
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get initial transform from base to laser frame: %s", ex.what());
    return false;
  }

  return true;
}

bool LaserScanMatcher::getBaseToFootprintTf(const std::string& frame_id)
{
  rclcpp::Time t = this->get_clock()->now();

  try {
    // Check if the transform is available
    if (!tf_buffer_->canTransform(frame_id, base_frame_, t, rclcpp::Duration::from_seconds(1.0))) {
      RCLCPP_WARN(this->get_logger(), "Transform from %s to %s not available yet", frame_id.c_str(), base_frame_.c_str());
      return false;
    }

    // Lookup the transform
    geometry_msgs::msg::TransformStamped transform_msg =
      tf_buffer_->lookupTransform(frame_id, base_frame_, t);

    // Convert to tf2::Transform
    tf2::Transform footprint_to_base_tf;
    tf2::fromMsg(transform_msg.transform, footprint_to_base_tf);

    footprint_to_base_ = footprint_to_base_tf;
    base_to_footprint_ = footprint_to_base_.inverse();
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get initial transform from footprint to base frame: %s", ex.what());
    return false;
  }

  return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void LaserScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y,
                                     double& pr_ch_a, double dt)
{
  std::scoped_lock lock(mutex_);

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  if (received_odom_)
  {
    pr_ch_x = current_odom_msg_.pose.pose.position.x -
              reference_odom_msg_.pose.pose.position.x;

    pr_ch_y = current_odom_msg_.pose.pose.position.y -
              reference_odom_msg_.pose.pose.position.y;

    pr_ch_a = tf2::getYaw(current_odom_msg_.pose.pose.orientation) -
              tf2::getYaw(reference_odom_msg_.pose.pose.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    reference_odom_msg_ = current_odom_msg_;
    gsl_matrix_set_zero(Sigma_odom_);
    theta_odom_ = 0.0;
  }

}

void LaserScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t)
{
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

geometry_msgs::msg::TransformStamped createTransformStamped(
const tf2::Transform& transform,
const rclcpp::Time& stamp,
const std::string& parent_frame,
const std::string& child_frame)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;

    msg.transform.translation.x = transform.getOrigin().x();
    msg.transform.translation.y = transform.getOrigin().y();
    msg.transform.translation.z = transform.getOrigin().z();

    msg.transform.rotation.x = transform.getRotation().x();
    msg.transform.rotation.y = transform.getRotation().y();
    msg.transform.rotation.z = transform.getRotation().z();
    msg.transform.rotation.w = transform.getRotation().w();

    return msg;
}

geometry_msgs::msg::Pose transformToPose(const tf2::Transform& tf)
{
  geometry_msgs::msg::Pose pose;

  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();

  pose.orientation.x = tf.getRotation().x();
  pose.orientation.y = tf.getRotation().y();
  pose.orientation.z = tf.getRotation().z();
  pose.orientation.w = tf.getRotation().w();

  return pose;
}

geometry_msgs::msg::PoseStamped transformToPoseStamped(
    const tf2::Transform& tf,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped pose_stamped;

  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = stamp;

  pose_stamped.pose.position.x = tf.getOrigin().x();
  pose_stamped.pose.position.y = tf.getOrigin().y();
  pose_stamped.pose.position.z = tf.getOrigin().z();

  pose_stamped.pose.orientation.x = tf.getRotation().x();
  pose_stamped.pose.orientation.y = tf.getRotation().y();
  pose_stamped.pose.orientation.z = tf.getRotation().z();
  pose_stamped.pose.orientation.w = tf.getRotation().w();

  return pose_stamped;
}

geometry_msgs::msg::PoseWithCovariance transformToPoseWithCovariance(const tf2::Transform& tf)
{
  geometry_msgs::msg::PoseWithCovariance pose_cov;

  pose_cov.pose.position.x = tf.getOrigin().x();
  pose_cov.pose.position.y = tf.getOrigin().y();
  pose_cov.pose.position.z = tf.getOrigin().z();

  pose_cov.pose.orientation.x = tf.getRotation().x();
  pose_cov.pose.orientation.y = tf.getRotation().y();
  pose_cov.pose.orientation.z = tf.getRotation().z();
  pose_cov.pose.orientation.w = tf.getRotation().w();

  return pose_cov;
}

geometry_msgs::msg::PoseWithCovarianceStamped transformToPoseWithCovarianceStamped(
    const tf2::Transform& tf,
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_stamped;

  pose_cov_stamped.header.frame_id = frame_id;
  pose_cov_stamped.header.stamp = stamp;

  pose_cov_stamped.pose.pose.position.x = tf.getOrigin().x();
  pose_cov_stamped.pose.pose.position.y = tf.getOrigin().y();
  pose_cov_stamped.pose.pose.position.z = tf.getOrigin().z();

  pose_cov_stamped.pose.pose.orientation.x = tf.getRotation().x();
  pose_cov_stamped.pose.pose.orientation.y = tf.getRotation().y();
  pose_cov_stamped.pose.pose.orientation.z = tf.getRotation().z();
  pose_cov_stamped.pose.pose.orientation.w = tf.getRotation().w();

  return pose_cov_stamped;
}

} // namespace scan_tools
