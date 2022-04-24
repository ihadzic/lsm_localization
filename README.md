# LSM Localization for ROS

## Overview

This package implements a localization algorithm that can be used
with [ROS Navigation](http://wiki.ros.org/navigation) as an alternative
to the popular [AMCL](http://wiki.ros.org/amcl).

In the core of the algorithm is the Kalman filter that fuses the odometry
with LIDAR scan measurements. The twist (velocity) part of the odometry topic
is integrated in the SE(2) space to calculate the prediction for the
Kalman filter. Measurement is produced by first constructing the expected
LIDAR image that the robot would see at its previous pose, given the
map, followed by calculating the incremental transform that would bring the
constructed image into the alignment with the actual LIDAR scan image.
The incremental transform is applied to the previous robot pose to produce
the measured pose, which updates the Kalman filter state. The incremental
pose is calculated using the PL-ICP algorithm from
[Canonical Scan Matcher](http://wiki.ros.org/csm) package and has been
derived from the [Laser Scan Matcher](http://wiki.ros.org/laser_scan_matcher)
package.

# Usage

You can start the LSM Localization node using the launch files provided
in this repository. The stand-alone `lsm_localization.launch` has a sane
and robust set of parameters that will work across many environments.
You can start the node as follows:

```
roslaunch lsm_localization lsm_localization.launch
```

The node will expect map, odometry, and LIDAR scan on standard topics
(`/map`, `/scan` and `/odom` respectively) and it will publish the result
on `lsm_localization/pose` topic. Frame names are
standard `map` for fixed frame and `base_link` for robot base. Other
frames (e.g. odometry, LIDAR) are deduced from the messages and the
transform tree. The initial pose must be sent on `initialpose` topic to
bootstrap the localization.

For localization to work correctly, you must make sure that odometry
is publishing valid covariance in the velocity (twist) section of the
odometry message. A common mistake is to publish velocity with zero
covariance, in which case the Kalman filter will give all weight
to odometry and the output will simply become the replica of odometry.

To save the computation, the node will downsample the LIDAR scan
and the downsample rate is set to 4 in the above example launch file.
Further the node will only publish the output pose when PL-ICP
matching succeeds. So the output pose will not be continuous and
it rate will be at most the LIDAR scan rate divided by the downsampling
rate.

If you need continuous tracking, you must enable publishing of map-odometry
transformation and make sure that your odometry is continuous. Very
often poorly designed and or buggy odometry may cause the appearance
that the localization is not working, so make sure your odometry
is continuous and presents valid covariance.

For a more canned example, you can run

```
roslaunch lsm_localization lsm_localization_jackal_example.launch
```

For this example to work, you must install
[`jackal_gazebo`](https://wiki.ros.org/jackal_gazebo)
and [`jackal_navigation`](https://wiki.ros.org/jackal_navigation) packages.
The launch file will start the simulation if Clearpath
Jackal robot on a racetrack along with along with the map server and
LSM Localization node. The node will be configured to subscribe to
topics that the Jackal simulation package publishes and it will
publish map-odometry transformation. It will also take the initial
pose from `/initialpose` topic, so you can bootstrap it from RVIZ.
An example configuration for RVIZ is available in `misc` directory
in this repository.

You can give it the initial pose that approximately matches the
robot position and orientation and watch it converge. Unlike most
implementations of particle filters, LSM Localization runs
even if the robot is not moving, so after giving it a reasonably
close initial pose, you can watch it converge without moving the robot.

## Subscribed topics

`/scan` [sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
>This is the standard LIDAR topic. Only the `ranges` array is considered
and `intensities` can be all zeros.

`/initialpose` [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
>This topic is used to sent the initial pose to bootstrap the filter.
It must be sent after the filter has received at least one odometry message.

`/map` [nav_msgs/OccupancyGrid](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html)
>This is the standard map topic used to communicate the map information
to the filter.

`/odom` [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
>This is the standard odometry topic used for prediction stage of the filter.
The frame must be continuous with no jumping.
It is very important that the topic publishes valid velocity (`twist` field)
with valid covariance associated with it.

In addition to the above-listed topics, the LSM Localization node
listens on standard transform topic to determine the relationship
among the frames of reference.


## Published topics

`/lsm_localization/pose` [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
>This topic is used to publish the result of the localization. It represents
the pose of the robot's base in the fixed frame. Alternative topics
with different types are also available and can turned on by configuration.

`/lsm_localization/constructed_scan` [sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
>This topic is the LIDAR scan that the filter constructs given the predicted
pose and feeds it into the PL-ICP matcher together with the actual scan (see the [Overview](#Overview) section above). One way to tell that the filter has
converged is to observe that the constructed scan is aligned with the actual
scan. If the two scans also match the map, the localization output is
correct. If the two scans are aligned, but not aligned to the map features,
then the filter is stuck in the local minimum.

`/lsm_localization/measured_pose` [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
>This is the pose produced by the output of the PL-ICP matched and before
feeding it into the Kalman filter. It can be used to analyze the filter
operation.

`/lsm_localization/predicted_pose` [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
>This is the pose predicted by the odometry before feeding it into the
Kalman filter. It can be used to analyze the filter operation.

The node can be enabled to publish map-odometry or map-base_link
transform. Normally, only one of these two transforms should be
published. Publishing both will result in conflicting transformation tree.

### Alternative outputs

In addition to the primary output pose topic, the filter can also publish
on the following topics using different type. Each topic can be
turned on by setting the parameter that controls it. Note that some topics
may not include all information that the filter output is producing:

`lsm_localization/pose2D`: [geometry_msgs/Pose2D](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Pose2D.html)

`lsm_localization/pose_stamped`: [geometry_msgs/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)

`lsm_localization/pose_with_covariance`: [geometry_msgs/PoseWithCovariance](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)

### Debug topics

The topics listed here are meant for debugging and provide additional
insight into the filter operation. The type for all topics is
[geometry_msgs/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)

`lsm_localization/debug/odom_delta`: This is the differential pose between
previously sampled odometry and current odometry at the time when the
prediction is calculated. This topic essentially represents the estimated
robot motion.

`lsm_localization/debug/laser_delta`: This is the differential pose
between the constructed LIDAR scan and the actual LIDAR scan. This topic
essentially represents the transform that PL-ICP matcher has generated.

`lsm_localization/debug/odom_reference`: This is the reference pose in the
odometry frame used to calculate the motion prediction.

`lsm_localization/debug/odom_current`: This is the current pose in
the odometry frame used to calculate the motion prediction.

## Parameters

`base_frame` (`string`, default: `base_link`)
>This parameter specifies the frame of reference for the robot base.
The output publishes the pose of this frame in fixed frame.

`fixed_frame` (`string`, default: `world`)
>This parameter specified the frame of reference for the fixed reference frame.
The output publishes the pose of the base frame in this frame.

`initialpose_topic_name` (`string`, default: `initialpose`)
>This parameter specifies which topic to listen on for initial pose.

`scan_downsample_rate` (`int`, default: 1)
>This parameter specifies the downsample rate for LIDAR

`no_odom_fusing` (`boolean`, default: `false`)
>If this parameter is set to `true`, the Kalman filter will not
fuse odometry-based prediction with the measurement.
Instead, the measured pose based on PL-ICP matching will be used at
face value. This is mathematically equivalent to running the Kalman filter
whose odometry covariance is always infinite.

`use_map` (`boolean`, default: `true`)
>If this parameter is set to `false`, the PL-ICP matching will
not be done against the constructed scan, but against two consecutive
scans of LIDAR measurement. This will essentially degenerate the
LSM Localization into the
[original Laser Scan Matcher](http://wiki.ros.org/laser_scan_matcher)
function from which this node has been derived.

`map_occupancy_threshold` (`int`, default: 10)
>This parameter specifies which value in the map occupancy grid will be
considered occupied. The value to use depends on the map representation,
but for most maps the default value will work.

`max_allowed_range` (`int`, default: -1)
>This parameter effectively cuts the LIDAR range. This may come useful
for long-range LIDARs operating in environment in which nearby features are
diverse enough to perform successful matching. Cutting the LIDAR range
will reduce the computational load in this case. Another use is when
evaluating how the algorithm would perform with shorter-range LIDARs.
The value of -1 means no limit (full LIDAR range is used).

`max_variance_trans` (`float`, default: 1e-5)
>This parameter specifies the positional variance above which the
PL-ICP matcher output will not be accepted. Setting this parameter
too high will allow false-positive matches, resulting in incorrect
localization, typically manifesting itself in shifted or skewed position.
Setting it too low, will result in rejecting good matches, effectively
reducing the output rate of the filter. In our experience, default
value is a reasonable tradeoff.

`max_variance_rot` (`float`, default: 1e-5)
>This parameter specifies the rotational variance above which the
PL-ICP matcher output will not be accepted. The same tradeoff
as for `max_variance_trans` apply.

`max_pose_delta_yaw` (`float`, default: 0.707)
>This parameter defines the maximum yaw difference between the output
and input of the PL-ICP matcher that will be accepted. Setting this
value too low will result in a low pull-in range and the algorithm may not
converge. Setting it too high may result in locking the output off by
90-degrees.

`publish_base_tf` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish map-base
transform. Only one of `publish_base_tf` and `publish_odom_tf` may
be set at the same time. It is OK to set both to `false` if there
is an external node responsible for publishing the transformations.

`publish_odom_tf` (`boolean`, default: `true`)
>When this parameter is set to `true`, the node will publish map-odom
transform. Only one of `publish_base_tf` and `publish_odom_tf` may
be set at the same time. It is OK to set both to `false` if there
is an external node responsible for publishing the transformations.

`publish_pose_with_covariance_stamped` (`boolean`, default: `true`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/pose_with_covariance_stamped` topic. This should
be the preferred topic to publish the filter result.

`publish_constructed_scan` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/constructed_scan` topic.

`publish_pose` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/pose2D` topic.

`publish_pose_with_covariance` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/pose_with_covariance` topic.

`publish_measured_pose` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/measured_pose` topic.

`publish_predicted_pose` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
`lsm_localization/predicted_pose` topic.

`publish_debug` (`boolean`, default: `false`)
>When this parameter is set to `true`, the node will publish on
debug topics

`debug_csm` (`boolean`, default: `false`)
>When this parameter is set to `true`, it will turn on debug
log messages in CSM library.

`position_covariance` (`float`, default: 1e-9)
>The value to use for z-component of the position covariance. Because
localization is performed in 2D space, z-height is not calculated and
this parameter gives the option to use the placeholder value for the
corresponding covariance. The covariance along x and y dimension is
calculated and reflects the filter's output confidence.

`orientation_covariance` (`float`, default: 1e-9)
>The value to use for pitch and roll components of the orientation covariance.
Because localization is performed in 2D space, pitch and roll are not
calculated and this parameter gives the option to use the placeholder
value for the corresponding covariance. The covariance or yaw estimate is
calculated and reflects the filter's output confidence.

In addition to the above-listed parameters, LSM Localization node
carries over a number of parameters that control the PL-ICP algorithm.
We don't describe these parameters here, but instead we
refer to the original
[Laser Scan Matcher node](http://wiki.ros.org/laser_scan_matcher).
These parameters are: `kf_dist_linear`, `kf_dist_angular`,
`max_angular_correction_deg`, `max_linear_correction`, `max_iterations`,
`epsilon_xy`, `epsilon_theta`, `max_correspondence_dist`, `sigma`
`use_corr_tricks`, `restart`, `restart_threshold_mean_error`,
`restart_dt`, `restart_dtheta`, `clustering_theta`, `orientation_neighbourhood`,
`use_point_to_line_distance`, `do_alpha_test`, `do_alpha_test_thresholdDeg`,
`outliers_maxPerc`, `outliers_adaptive_order`, `outliers_adaptive_mult`,
`do_visiblity_test`, `outliers_remove_doubles`, `debug_verify_tricks`,
`use_ml_weights`, `use_sigma_weights`.

While this may look like a lot, the values listed in example launch
files, provided in this repository, along with default values
that are not specified in launch files, are typically good
enough to achieve robust localization for most environments.
