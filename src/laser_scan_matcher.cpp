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

#include <laser_scan_matcher/laser_scan_matcher.h>
#include <boost/assign.hpp>

namespace scan_tools
{

LaserScanMatcher::LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_(false),
  received_imu_(false),
  received_odom_(false),
  received_vel_(false),
  have_map_(false),
  initialpose_valid_(false),
  map_res_(0.0),
  map_width_(0),
  map_height_(0),
  theta_odom_(0.0)
{
  ROS_INFO("Starting LaserScanMatcher");

  // **** init parameters

  initParams();

  // **** state variables

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

  // **** publishers
  if (publish_constructed_scan_ && use_map_)
  {
    constructed_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(
      "lsm/constructed_scan", 5);
  }

  if (publish_pose_)
  {
    pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
      "lsm/pose2D", 5);
  }

  if (publish_pose_stamped_)
  {
    pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "lsm/pose_stamped", 5);
  }

  if (publish_pose_with_covariance_)
  {
    pose_with_covariance_publisher_  = nh_.advertise<geometry_msgs::PoseWithCovariance>(
      "lsm/pose_with_covariance", 5);
  }

  if (publish_pose_with_covariance_stamped_)
  {
    pose_with_covariance_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "lsm/pose", 5);
  }

  if (publish_predicted_pose_)
  {
    if (!input_.do_compute_covariance) {
      ROS_WARN("publishing predicted pose requires 'do_compute_covariance' option");
    } else if (!use_odom_) {
      ROS_WARN("publishing predicted pose requires 'use_odom' option");
    } else {
      predicted_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "lsm/predicted_pose", 5);
    }
  }

  if (publish_measured_pose_)
  {
    if (!input_.do_compute_covariance) {
      ROS_WARN("publishing measured pose requires 'do_compute_covariance' option");
    } else {
      measured_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
       "lsm/measured_pose", 5);
    }
  }

  if (publish_debug_)
  {
    debug_odom_delta_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "lsm/debug/odom_delta", 5);
    debug_laser_delta_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "lsm/debug/laser_delta", 5);
    debug_odom_reference_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "lsm/debug/odom_reference", 5);
    debug_odom_current_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "lsm/debug/odom_current", 5);
  }

  // *** subscribers

  initialpose_subscriber_ = nh_.subscribe(
    initialpose_topic_, 1, &LaserScanMatcher::initialposeCallback, this);

  if (use_map_)
  {
    ROS_INFO("using map-based matching");
    map_subscriber_ = nh_.subscribe(
      "map", 1, &LaserScanMatcher::mapCallback, this);
  }
  else
    ROS_INFO("using frame-to-frame matching");

  scan_subscriber_ = nh_.subscribe(
    "scan", 1, &LaserScanMatcher::scanCallback, this);

  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      "imu/data", 1, &LaserScanMatcher::imuCallback, this);
  }
  if (use_odom_)
  {
    odom_subscriber_ = nh_.subscribe(
      "/odom", 1, &LaserScanMatcher::odomCallback, this);
  }
  if (use_vel_)
  {
    if (stamped_vel_)
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velStmpCallback, this);
    else
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velCallback, this);
  }
}

LaserScanMatcher::~LaserScanMatcher()
{
  ROS_INFO("Destroying LaserScanMatcher");
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
  ROS_INFO("LaserScanMatcher state cleared");

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
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";
  if (!nh_private_.getParam ("default_scan_frame", default_scan_frame_))
    default_scan_frame_ = "base_scan";

  if (!nh_private_.getParam ("initialpose_topic_name", initialpose_topic_))
    initialpose_topic_ = "initialpose";

  if (!nh_private_.getParam("scan_downsample_rate", scan_downsample_rate_))
    scan_downsample_rate_ = 1;
  if (scan_downsample_rate_ <= 0)
    scan_downsample_rate_ = 1;

  // **** keyframe params: when to generate the keyframe scan
  // if either is set to 0, reduces to frame-to-frame matching

  if (!nh_private_.getParam ("kf_dist_linear", kf_dist_linear_))
    kf_dist_linear_ = 0.10;
  if (!nh_private_.getParam ("kf_dist_angular", kf_dist_angular_))
    kf_dist_angular_ = 10.0 * (M_PI / 180.0);

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /imu topic
  // 2) odom - [x, y, theta] from wheel odometry - /odom topic
  // 3) vel - [x, y, theta] from velocity predictor - see alpha-beta predictors - /vel topic
  // If more than one is enabled, priority is imu > odom > vel

  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = true;
  if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = true;
  if (!nh_private_.getParam ("no_odom_fusing", no_odom_fusing_))
    no_odom_fusing_ = false;
  if (!nh_private_.getParam ("use_vel", use_vel_))
    use_vel_ = false;

  // **** Parameters that control map-to-scan matching
  // use_map must be true for map-to-scan matching to be used
  // other parameters control how the initial-pose scan is constructed
  if (!nh_private_.getParam ("use_map", use_map_))
    use_map_ = true;
  if (!nh_private_.getParam ("map_occupancy_threshold", map_occupancy_threshold_))
    map_occupancy_threshold_ = 10.0;
  // min and max range for constructing the scan that initial pose
  // would see
  if (!nh_private_.getParam("max_allowed_range", max_allowed_range_))
    max_allowed_range_ = -1;
  if (!nh_private_.getParam ("default_range_min", default_range_min_))
    default_range_min_ = 0.5;
  if (!nh_private_.getParam ("default_range_max", default_range_min_))
    default_range_max_ = 16.0;
  if (!nh_private_.getParam ("default_angle_min", default_angle_min_))
    default_angle_min_ = 0.0;
  if (!nh_private_.getParam ("default_angle_inc", default_angle_inc_))
    default_angle_inc_ = M_PI / 180.0;
  if (!nh_private_.getParam ("default_angle_max", default_angle_max_))
    default_angle_max_ = 2 * M_PI - default_angle_inc_;
  if (!nh_private_.getParam ("default_scan_time", default_scan_time_))
    default_scan_time_ = 0.2;
  if (!nh_private_.getParam ("default_time_inc", default_time_inc_))
    default_time_inc_ = default_scan_time_ * default_angle_inc_ / (default_angle_max_ - default_angle_min_);

  if (!nh_private_.getParam ("max_variance_trans", max_variance_trans_))
    max_variance_trans_ = 1e-5;

  if (!nh_private_.getParam ("max_variance_rot", max_variance_rot_))
    max_variance_rot_ = 1e-5;

  if (!nh_private_.getParam ("max_pose_delta_yaw", max_pose_delta_yaw_))
    max_pose_delta_yaw_ = 0.707; // sqrt(2)/2 or 45 degrees

  // **** Are velocity input messages stamped?
  // if false, will subscribe to Twist msgs on /vel
  // if true, will subscribe to TwistStamped msgs on /vel
  if (!nh_private_.getParam ("stamped_vel", stamped_vel_))
    stamped_vel_ = false;

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;
  if (!nh_private_.getParam ("publish_constructed_scan", publish_constructed_scan_))
    publish_constructed_scan_ = false;
  if (!nh_private_.getParam ("publish_pose_stamped", publish_pose_stamped_))
    publish_pose_stamped_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance", publish_pose_with_covariance_))
    publish_pose_with_covariance_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped_))
    publish_pose_with_covariance_stamped_ = false;
  if (!nh_private_.getParam ("publish_predicted_pose", publish_predicted_pose_))
    publish_predicted_pose_ = false;
  if (!nh_private_.getParam ("publish_measured_pose", publish_measured_pose_))
    publish_measured_pose_ = false;
  if (!nh_private_.getParam ("publish_debug", publish_debug_))
    publish_debug_ = false;

  if (!nh_private_.getParam("position_covariance", position_covariance_))
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  }

  if (!nh_private_.getParam("orientation_covariance", orientation_covariance_))
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

void LaserScanMatcher::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_imu_msg_ = *imu_msg;
  if (!received_imu_)
  {
    reference_imu_msg_ = *imu_msg;
    received_imu_ = true;
  }
}

nav_msgs::Odometry* LaserScanMatcher::latestOdomBefore(const ros::Time& time)
{
    for (auto o=odom_history_.begin(); o < odom_history_.end(); o++) {
        if (o->header.stamp <= time)
            return &(*o);
    }
    return NULL;
}

nav_msgs::Odometry* LaserScanMatcher::earliestOdomAfter(const ros::Time& time)
{
    for (auto o=odom_history_.begin(); o < odom_history_.end(); o++) {
        if (o->header.stamp == time) {
            // found exact match, that's it!
            return &(*o);
        } else if (o->header.stamp < time) {
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

void LaserScanMatcher::addOdomToHistory(const nav_msgs::Odometry::ConstPtr& o)
{
    if (odom_history_.size() >= MAX_ODOM_HISTORY)
        odom_history_.pop_back();
    odom_history_.push_front(*o);
}

double LaserScanMatcher::syncOdom(const ros::Time& time)
{
    nav_msgs::Odometry* latest_before = latestOdomBefore(time);
    if (!latest_before) {
        ROS_WARN("missing latest odom before t=%.3f (is odom topic alive)",
                 time.toSec());
        return -1.0;
    }
    nav_msgs::Odometry o = *latest_before;
    o.header.stamp = time;
    tf::Quaternion q;
    // extrapolate because we don't have the second point
    double delta_t = (time - latest_before->header.stamp).toSec();
    o.pose.pose.position.x = latest_before->pose.pose.position.x +
        delta_t * latest_before->twist.twist.linear.x;
    o.pose.pose.position.y = latest_before->pose.pose.position.y +
        delta_t * latest_before->twist.twist.linear.y;
    o.pose.pose.position.z = latest_before->pose.pose.position.z +
        delta_t * latest_before->twist.twist.linear.z;
    tf::Quaternion delta_q;
    delta_q.setRPY(delta_t * latest_before->twist.twist.angular.x,
                   delta_t * latest_before->twist.twist.angular.y,
                   delta_t * latest_before->twist.twist.angular.z);
    tf::Quaternion qb(latest_before->pose.pose.orientation.x,
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

double LaserScanMatcher::getOdomDeltaT(const nav_msgs::Odometry::ConstPtr& o)
{
  if (odom_history_.empty())
    return -1.0;
  return (o->header.stamp - odom_history_.front().header.stamp).toSec();
}

void LaserScanMatcher::doPredictPose(double delta_t)
{
  tf::Transform current_odom_tf;
  tf::Transform reference_odom_tf;
  createTfFromXYTheta(current_odom_msg_.pose.pose.position.x,
                      current_odom_msg_.pose.pose.position.y,
                      tf::getYaw(current_odom_msg_.pose.pose.orientation),
                      current_odom_tf);
  createTfFromXYTheta(reference_odom_msg_.pose.pose.position.x,
                      reference_odom_msg_.pose.pose.position.y,
                      tf::getYaw(reference_odom_msg_.pose.pose.orientation),
                      reference_odom_tf);
  tf::Transform delta_odom_tf = reference_odom_tf.inverse() * current_odom_tf;

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

void LaserScanMatcher::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);
  double delta_t = getOdomDeltaT(odom_msg);
  addOdomToHistory(odom_msg);
  if (!received_odom_)
  {
    if (!getBaseToFootprintTf(odom_msg->child_frame_id)) {
      ROS_WARN("skipping odom");
      return;
    }
    reference_odom_msg_ = *odom_msg;
    current_odom_msg_ = *odom_msg;
    gsl_matrix_set_zero(Sigma_odom_);
    theta_odom_ = 0.0;
    received_odom_ = true;
    return;
  }

  // if we got here, we have what we need to do the integration
  assert (delta_t >= 0.0);
  current_odom_msg_ = *odom_msg;
  doPredictPose(delta_t);
  doPublishOdomRate(odom_msg->header.stamp);
}

void LaserScanMatcher::velCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = *twist_msg;

  received_vel_ = true;
}

void LaserScanMatcher::velStmpCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = twist_msg->twist;

  received_vel_ = true;
}

void LaserScanMatcher::setTransSigmaMatrix(double yaw)
{
  gsl_matrix_set(trans_sigma_, 0, 0, cos(yaw));
  gsl_matrix_set(trans_sigma_, 0, 1, -sin(yaw));
  gsl_matrix_set(trans_sigma_, 1, 0, sin(yaw));
  gsl_matrix_set(trans_sigma_, 1, 1, cos(yaw));
}

void LaserScanMatcher::beamAddIncidentAngle(double dx, double dy, double laser_yaw, std::vector<double>& angles)
{
  double theta = atan2(dy, dx) - laser_yaw;
  double angle_max = (initialized_) ? observed_angle_max_ : default_angle_max_;
  double angle_min = (initialized_) ? observed_angle_min_ : default_angle_min_;
  // ensure that theta is in -pi to pi range
  while (theta < -M_PI) theta += 2 * M_PI;
  while (theta >= M_PI) theta -= 2 * M_PI;
  // if LIDAR range uses only positive angles, make the angle compatible with it
  if (angle_min >= 0 && theta < 0.0) theta += 2 * M_PI;
  if (theta < angle_min) return;
  if (theta > angle_max) return;
  angles.push_back(theta);
}

void LaserScanMatcher::constructScan(const ros::Time& time)
{
  double range_max = (initialized_) ? observed_range_max_ : default_range_max_;
  double range_min = (initialized_) ? observed_range_min_ : default_range_min_;
  double angle_max = (initialized_) ? observed_angle_max_ : default_angle_max_;
  double angle_min = (initialized_) ? observed_angle_min_ : default_angle_min_;
  double angle_inc = (initialized_) ? observed_angle_inc_ : default_angle_inc_;

  if (use_odom_) {
    predicted_pose_in_pcl_ = pcl2f_ * predicted_pose_;
  } else {
    predicted_pose_in_pcl_ = pcl2f_ * initial_pose_;
  }
  tf::Transform predicted_laser_pose_in_pcl = predicted_pose_in_pcl_ * base_to_laser_;
  double laser_x = predicted_laser_pose_in_pcl.getOrigin().getX();
  double laser_y = predicted_laser_pose_in_pcl.getOrigin().getY();
  double laser_yaw = tf::getYaw(predicted_laser_pose_in_pcl.getRotation());
  ROS_DEBUG("%s: laser_x=%f, laser_y=%f laser_yaw (deg)=%f", __func__,
            laser_x, laser_y, 180.0 * laser_yaw / M_PI);
  ROS_DEBUG("%s: range=[%f,%f], angle=[%f,%f]@%f", __func__,
            range_min, range_max, angle_min, angle_max, angle_inc);
  // indices into the map for the region of interest
  // x0, y0: lower-left corner index
  // w, h  : width and height in indices
  int x0 = (int)((laser_x - range_max) / map_res_);
  if (x0 < 0) x0 = 0;
  int y0 = (int)((laser_y - range_max) / map_res_);
  if (y0 < 0) y0 = 0;
  int w = 2 * (int)(range_max / map_res_);
  int h = w;
  if (x0 + w > map_width_) w = map_width_ - x0;
  if (y0 + h > map_height_) h = map_height_ - y0;

  // clear intensity array
  int num_angles = (int)round((angle_max - angle_min + angle_inc) / angle_inc);
  ROS_DEBUG("%s: x0=%d, y0=%d, w=%d, h=%d, num_angles=%d", __func__,
            x0, y0, w, h, num_angles);
  constructed_intensities_.clear();
  constructed_intensities_.reserve(num_angles);
  constructed_ranges_.clear();
  constructed_ranges_.reserve(num_angles);
  for (int i = 0; i < num_angles; i++) {
    constructed_intensities_.push_back(0.0);
    constructed_ranges_.push_back(0.0);
  }
  // scan all points in the area of interest, convert to polar and
  // remember the point if it is currently the closest for the given point
  for (int y = y0; y < y0 + h; y++) {
    for (int x = x0; x < x0 + w; x++) {
      if (map_grid_[y][x] > map_occupancy_threshold_) {
        // add .5 to reference the middle of the pixel
        double delta_x = (x + .5) * map_res_ - laser_x;
        double delta_y = (y + .5) * map_res_ - laser_y;
        double rho = sqrt(delta_x * delta_x + delta_y * delta_y);
        // is pixel in range at all?
        if (rho < range_min) continue;
        if (rho > range_max) continue;
        if (max_allowed_range_ > 0 && rho > max_allowed_range_) continue;
        // go to polar cordinates relative to the scanner heading
        // calculate incident angles for all four corners of the pixel
        std::vector<double> incident_angles;
        beamAddIncidentAngle(delta_x - .5 * map_res_, delta_y - .5 * map_res_,
                             laser_yaw, incident_angles);
        beamAddIncidentAngle(delta_x + .5 * map_res_, delta_y - .5 * map_res_,
                             laser_yaw, incident_angles);
        beamAddIncidentAngle(delta_x - .5 * map_res_, delta_y + .5 * map_res_,
                             laser_yaw, incident_angles);
        beamAddIncidentAngle(delta_x + .5 * map_res_, delta_y + .5 * map_res_,
                             laser_yaw, incident_angles);
        if (incident_angles.empty()) continue;
        double min_theta = *std::min_element(incident_angles.begin(),
                                             incident_angles.end());
        double max_theta = *std::max_element(incident_angles.begin(),
                                             incident_angles.end());
        // calculate view angle with under which this pixel is visible
        double delta_theta = max_theta - min_theta;
        double start_theta = min_theta;
        // handle wrap-around the circle
        if (delta_theta > M_PI) {
          start_theta = max_theta;
          delta_theta = 2 * M_PI - delta_theta;
        }
        int theta_index = (int)((start_theta - angle_min) / angle_inc) % num_angles;
        for (int i = 0; i <= (int)(delta_theta / angle_inc); i++) {
          // either no point ever recorded for this angle, so take it
          // or the current point is closer than previously recorded point
          if ((constructed_intensities_[theta_index] == 0.0 &&
               constructed_ranges_[theta_index] == 0.0) ||
              rho < constructed_ranges_[theta_index]) {
            // intensity can be anything, range is whatever rho says
            constructed_intensities_[theta_index] = 100.0;
            constructed_ranges_[theta_index] = rho;
          }
          theta_index = (theta_index + 1) % num_angles;
        }
      }
    }
  }
  if (publish_constructed_scan_) {
    sensor_msgs::LaserScan::Ptr scan_msg;
    scan_msg = boost::make_shared<sensor_msgs::LaserScan>();
    scan_msg->range_min = range_min;
    scan_msg->range_max = range_max;
    scan_msg->angle_min = angle_min;
    scan_msg->angle_max = angle_max;
    scan_msg->angle_increment = angle_inc;
    scan_msg->scan_time = (initialized_) ? observed_scan_time_ : default_scan_time_;
    scan_msg->time_increment = (initialized_) ? observed_time_inc_ : default_time_inc_;
    scan_msg->header.stamp = time;
    scan_msg->header.frame_id = (initialized_) ?  observed_scan_frame_ : default_scan_frame_;
    scan_msg->ranges.resize(num_angles);
    scan_msg->intensities.resize(num_angles);
    for (int i = 0; i < num_angles; i++) {
      scan_msg->ranges[i] = constructed_ranges_[i];
      scan_msg->intensities[i] = constructed_intensities_[i];
    }
    constructed_scan_publisher_.publish(scan_msg);
  }
}

void LaserScanMatcher::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  boost::mutex::scoped_lock(mutex_);
  tf::Transform pose;

  if (!have_map_) {
    ROS_WARN("map not loaded, cannot process initial pose");
    return;
  }
  if (use_odom_ && !received_odom_) {
    ROS_WARN("odom never received, cannot process initial pose");
    return;
  }
  if (syncOdom(pose_msg->header.stamp) < 0.0) {
    ROS_WARN("cannot sync odometry for initial pose timestamp");
    return;
  }
  reference_odom_msg_ = current_odom_msg_;
  gsl_matrix_set_zero(Sigma_odom_);
  theta_odom_ = 0.0;

  // convert the input pose (typically in 'map' frame into lsm_fixed frame)
  pose.setOrigin(tf::Vector3(pose_msg->pose.pose.position.x,
                             pose_msg->pose.pose.position.y,
                             pose_msg->pose.pose.position.z));
  pose.setRotation(tf::Quaternion(pose_msg->pose.pose.orientation.x,
                                  pose_msg->pose.pose.orientation.y,
                                  pose_msg->pose.pose.orientation.z,
                                  pose_msg->pose.pose.orientation.w));
  if (pose_msg->header.frame_id != fixed_frame_)
    ROS_WARN("incorrect frame for initial pose: '%s' vs. '%s'",
             pose_msg->header.frame_id.c_str(), fixed_frame_.c_str());

  // new initial pose came in, set the predicted pose to be equal
  // to it and set the reference odom to be the current odom
  initial_pose_ = pose;
  predicted_pose_in_pcl_ = pcl2f_ * initial_pose_;
  initialpose_valid_ = true;
  // reflect the incoming initial pose if set up to publish
  // on compatible topic
  if (publish_pose_with_covariance_stamped_) {
    pose_with_covariance_stamped_publisher_.publish(pose_msg);
  }
}

void LaserScanMatcher::mapCallback (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  boost::mutex::scoped_lock(mutex_);

  map_res_ = map_msg->info.resolution;
  map_width_ = map_msg->info.width;
  map_height_ = map_msg->info.height;
  f2pcl_.setOrigin(tf::Vector3(map_msg->info.origin.position.x,
                               map_msg->info.origin.position.y,
                               map_msg->info.origin.position.z));
  f2pcl_.setRotation(tf::Quaternion(map_msg->info.origin.orientation.x,
                                    map_msg->info.origin.orientation.y,
                                    map_msg->info.origin.orientation.z,
                                    map_msg->info.origin.orientation.w));
  pcl2f_ = f2pcl_.inverse();
  map_grid_.resize(map_height_);
  int i = 0;
  for (auto row=map_grid_.begin(); row < map_grid_.end(); row++) {
      row->resize(map_width_);
      for (auto element=row->begin(); element < row->end(); element++)
          *element = map_msg->data[i++];
  }
  ROS_INFO("got map: %dx%d@%f", map_width_, map_height_, map_res_);
  have_map_ = true;
}

void LaserScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  boost::mutex::scoped_lock(mutex_);

  // **** if first scan, cache the tf from base to the scanner
  if (!initialized_)
  {
    ROS_INFO("first scan observed, initializing");

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("Skipping scan");
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

  if (scan_msg->header.seq % scan_downsample_rate_ != 0)
    return;
  ros::WallTime start = ros::WallTime::now();
  LDP curr_ldp_scan;
  int r = 0;
  double delta_t;
  if ((delta_t = syncOdom(scan_msg->header.stamp)) < 0.0) {
    ROS_WARN("odometry sync failed, skipping scan");
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
      ROS_INFO_THROTTLE(2,
        "initial pose not received yet, scan processing skipped"
      );
  } else {
    laserScanToLDP(scan_msg, curr_ldp_scan);
    r = processScan(curr_ldp_scan, scan_msg->header.stamp);
  }
  if (r) {
    double dur = (ros::WallTime::now() - start).toSec() * 1e3;
    ROS_DEBUG("complete scan processing total duration: %.1f ms", dur);
  }
}

void LaserScanMatcher::doPublishDebugTF(const ros::Time& time, const tf::Transform& transform, const ros::Publisher &publisher, const std::string& frame)
{
  geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
  pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();
  pose_stamped_msg->header.stamp = time;
  pose_stamped_msg->header.frame_id = frame;
  tf::poseTFToMsg(transform, pose_stamped_msg->pose);
  publisher.publish(pose_stamped_msg);
}

void LaserScanMatcher::doPublishOdomRate(const ros::Time& time)
{
  if (publish_predicted_pose_ && input_.do_compute_covariance && use_odom_) {
    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
    pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg->header.stamp    = time;
    pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

    tf::poseTFToMsg(predicted_pose_, pose_with_covariance_stamped_msg->pose.pose);

    pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
      (gsl_matrix_get(Sigma_odom_trans_, 0, 0)) (gsl_matrix_get(Sigma_odom_trans_, 0, 1))  (0)  (0)  (0)  (0)
      (gsl_matrix_get(Sigma_odom_trans_, 1, 0)) (gsl_matrix_get(Sigma_odom_trans_, 1, 1))  (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
      (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(Sigma_odom_trans_, 2, 2));
    predicted_pose_publisher_.publish(pose_with_covariance_stamped_msg);
  }
}

void LaserScanMatcher::doPublishScanRate(const ros::Time& time)
{
  if (publish_pose_) {
    // unstamped Pose2D message
    geometry_msgs::Pose2D::Ptr pose_msg;
    pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
    pose_msg->x = f2b_.getOrigin().getX();
    pose_msg->y = f2b_.getOrigin().getY();
    pose_msg->theta = tf::getYaw(f2b_.getRotation());
    pose_publisher_.publish(pose_msg);
  }
  if (publish_pose_stamped_) {
    // stamped Pose message
    geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
    pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();

    pose_stamped_msg->header.stamp    = time;
    pose_stamped_msg->header.frame_id = fixed_frame_;

    tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);

    pose_stamped_publisher_.publish(pose_stamped_msg);
  }
  if (publish_pose_with_covariance_) {
    // unstamped PoseWithCovariance message
    geometry_msgs::PoseWithCovariance::Ptr pose_with_covariance_msg;
    pose_with_covariance_msg = boost::make_shared<geometry_msgs::PoseWithCovariance>();
    tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

    if (input_.do_compute_covariance) {
      pose_with_covariance_msg->covariance = boost::assign::list_of
        (gsl_matrix_get(output_.cov_x_m, 0, 0)) (gsl_matrix_get(output_.cov_x_m, 0, 1))  (0)  (0)  (0)  (0)
        (gsl_matrix_get(output_.cov_x_m, 1, 0)) (gsl_matrix_get(output_.cov_x_m, 1, 1))  (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
        (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 2, 2));
    } else {
      pose_with_covariance_msg->covariance = boost::assign::list_of
        (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
    }

    pose_with_covariance_publisher_.publish(pose_with_covariance_msg);
  }
  if (publish_pose_with_covariance_stamped_) {
    // stamped Pose message
    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
    pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg->header.stamp    = time;
    pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

    tf::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

    if (input_.do_compute_covariance) {
      pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
        (gsl_matrix_get(output_.cov_x_m, 0, 0)) (gsl_matrix_get(output_.cov_x_m, 0, 1))  (0)  (0)  (0)  (0)
        (gsl_matrix_get(output_.cov_x_m, 1, 0)) (gsl_matrix_get(output_.cov_x_m, 1, 1))  (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
        (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 2, 2));
    } else {
      pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
        (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
    }

    pose_with_covariance_stamped_publisher_.publish(pose_with_covariance_stamped_msg);
  }

  if (publish_measured_pose_ && input_.do_compute_covariance) {
    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
    pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

    pose_with_covariance_stamped_msg->header.stamp    = time;
    pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

    tf::poseTFToMsg(measured_pose_, pose_with_covariance_stamped_msg->pose.pose);

    pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
      (gsl_matrix_get(Sigma_measured_, 0, 0)) (gsl_matrix_get(Sigma_measured_, 0, 1))  (0)  (0)  (0)  (0)
      (gsl_matrix_get(Sigma_measured_, 1, 0)) (gsl_matrix_get(Sigma_measured_, 1, 1))  (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
      (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(Sigma_measured_, 2, 2));
    measured_pose_publisher_.publish(pose_with_covariance_stamped_msg);
  }

  if (publish_tf_) {
    tf::StampedTransform transform_msg (f2b_, time, fixed_frame_, base_frame_);
    tf_broadcaster_.sendTransform (transform_msg);
  }

}

tf::Vector3 LaserScanMatcher::fusePoses(const tf::Transform& pose_delta)
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
  // y = K*measured
  gsl_vector_set(xvec_, 0, measured_pose_.getOrigin().getX());
  gsl_vector_set(xvec_, 1, measured_pose_.getOrigin().getY());
  gsl_vector_set(xvec_, 2, tf::getYaw(measured_pose_.getRotation()));
  gsl_blas_dgemv(CblasNoTrans, 1.0, kalman_gain_, xvec_, 0.0, yvec_);
  // y += (I-K)*predicted
  gsl_vector_set(xvec_, 0, predicted_pose_.getOrigin().getX());
  gsl_vector_set(xvec_, 1, predicted_pose_.getOrigin().getY());
  gsl_vector_set(xvec_, 2, tf::getYaw(predicted_pose_.getRotation()));
  gsl_blas_dgemv(CblasNoTrans, 1.0, kalman_gain_comp_, xvec_, 1.0, yvec_);

  ROS_DEBUG("kalman_gain:\n%e %e %e\n%e %e %e\n%e %e %e",
            gsl_matrix_get(kalman_gain_, 0, 0),
            gsl_matrix_get(kalman_gain_, 0, 1),
            gsl_matrix_get(kalman_gain_, 0, 2),
            gsl_matrix_get(kalman_gain_, 1, 0),
            gsl_matrix_get(kalman_gain_, 1, 1),
            gsl_matrix_get(kalman_gain_, 1, 2),
            gsl_matrix_get(kalman_gain_, 2, 0),
            gsl_matrix_get(kalman_gain_, 2, 1),
            gsl_matrix_get(kalman_gain_, 2, 2));
  return tf::Vector3(
    gsl_vector_get(yvec_, 0),
    gsl_vector_get(yvec_, 1),
    gsl_vector_get(yvec_, 2));
}

int LaserScanMatcher::processScan(LDP& curr_ldp_scan, LDP& ref_ldp_scan, const ros::Time& time)
{
  ros::WallTime start = ros::WallTime::now();

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
      ROS_ERROR("sm_icp failed, probably singular matrix");
      output_.valid = false;
    } else
      throw e;
  }
  if (output_.valid) {
    if (!input_.do_compute_covariance ||
        (gsl_matrix_get(output_.cov_x_m, 0, 0) < max_variance_trans_ &&
         gsl_matrix_get(output_.cov_x_m, 1, 1) < max_variance_trans_ &&
         gsl_matrix_get(output_.cov_x_m, 2, 2) < max_variance_rot_)) {
      // the correction of the laser's position, in the laser frame
      ROS_DEBUG("found correlation transform: x=%f, y=%f, yaw=%f",
               output_.x[0], output_.x[1], 180.0 * output_.x[2] / M_PI);
      if (input_.do_compute_covariance)
        ROS_DEBUG("variances: %f, %f, %f",
                 gsl_matrix_get(output_.cov_x_m, 0, 0),
                 gsl_matrix_get(output_.cov_x_m, 1, 1),
                 gsl_matrix_get(output_.cov_x_m, 2, 2));
      tf::Transform pose_delta_laser;
      createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2],
                          pose_delta_laser);
      // predicted pose is pcl-to-base_link
      // delta is crrection of predicted pose, right two
      // operands get us corrected pcl-to-base_link
      // multiply by map-to-pcl on the left to get
      // map-to-base_link, which is the pose we are looking for
      if (output_.x[2] > max_pose_delta_yaw_ ||
          output_.x[2] < -max_pose_delta_yaw_) {
        ROS_WARN("laser pose delta max yaw exceeded %f", output_.x[2]);
        ld_free(curr_ldp_scan);
        ld_free(ref_ldp_scan);
        return 0;
      }
      tf::Transform pose_delta =
        base_to_laser_ * pose_delta_laser * laser_to_base_;
      if (publish_debug_)
        doPublishDebugTF(time, pose_delta, debug_laser_delta_publisher_, "");
      if (input_.do_compute_covariance) {
        // Sigma_odom is the covariance of odometry-delta
        // we need covariance in the map frame, so apply transforms
        setTransSigmaMatrix(
          tf::getYaw((initial_pose_ * base_to_footprint_ * base_to_laser_).getRotation())
        );
        gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0,
                       Sigma_odom_, trans_sigma_, 0.0, I2_);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, trans_sigma_,
                       I2_, 0.0, Sigma_odom_trans_);
        if (!no_odom_fusing_) {
          tf::Vector3 pv = fusePoses(pose_delta);
          createTfFromXYTheta(pv.getX(), pv.getY(), pv.getZ(), f2b_);
        } else {
          f2b_ = f2pcl_ * predicted_pose_in_pcl_ * pose_delta;
        }
      } else {
        // no-covariance case, just take measurement at face value
        f2b_ = f2pcl_ * predicted_pose_in_pcl_ * pose_delta;
      }
      doPublishScanRate(time);
      double dur = (ros::WallTime::now() - start).toSec() * 1e3;
      ROS_DEBUG("scan matcher duration: %.1f ms, iterations: %d", dur, output_.iterations);
      ld_free(curr_ldp_scan);
      ld_free(ref_ldp_scan);
      return 1;
    } else {
      ld_free(curr_ldp_scan);
      ld_free(ref_ldp_scan);
      return 0;
    }
  } else {
    ROS_WARN("Error in scan matching");
    ld_free(curr_ldp_scan);
    ld_free(ref_ldp_scan);
    return 0;
  }
}

int LaserScanMatcher::processScan(LDP& curr_ldp_scan, const ros::Time& time)
{
  ros::WallTime start = ros::WallTime::now();
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

  double dt = (time - last_icp_time_).toSec();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the fixed frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

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
  tf::Transform corr_ch;

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;
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
    ROS_WARN("Error in scan matching");
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

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
  return success;
}

bool LaserScanMatcher::newKeyframeNeeded(const tf::Transform& d)
{
  if (fabs(tf::getYaw(d.getRotation())) > kf_dist_angular_) return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
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
    if (constructed_intensities_[i] > 0.0) {
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

bool LaserScanMatcher::getBaseToLaserTf (const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, frame_id, t, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

bool LaserScanMatcher::getBaseToFootprintTf (const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform footprint_to_base_tf;
  try
  {
    tf_listener_.waitForTransform(
      frame_id, base_frame_, t, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      frame_id, base_frame_, t, footprint_to_base_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get initial transform from footprint to base frame, %s", ex.what());
    return false;
  }
  footprint_to_base_ = footprint_to_base_tf;
  base_to_footprint_ = footprint_to_base_.inverse();

  return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void LaserScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y,
                                     double& pr_ch_a, double dt)
{
  boost::mutex::scoped_lock(mutex_);

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use velocity (for example from ab-filter)
  if (use_vel_)
  {
    pr_ch_x = dt * latest_vel_msg_.linear.x;
    pr_ch_y = dt * latest_vel_msg_.linear.y;
    pr_ch_a = dt * latest_vel_msg_.angular.z;

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {
    pr_ch_x = current_odom_msg_.pose.pose.position.x -
              reference_odom_msg_.pose.pose.position.x;

    pr_ch_y = current_odom_msg_.pose.pose.position.y -
              reference_odom_msg_.pose.pose.position.y;

    pr_ch_a = tf::getYaw(current_odom_msg_.pose.pose.orientation) -
              tf::getYaw(reference_odom_msg_.pose.pose.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    reference_odom_msg_ = current_odom_msg_;
    gsl_matrix_set_zero(Sigma_odom_);
    theta_odom_ = 0.0;
  }

  // **** use imu
  if (use_imu_ && received_imu_)
  {
    pr_ch_a = tf::getYaw(latest_imu_msg_.orientation) -
              tf::getYaw(reference_imu_msg_.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    reference_imu_msg_ = latest_imu_msg_;
  }
}

void LaserScanMatcher::createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

} // namespace scan_tools
