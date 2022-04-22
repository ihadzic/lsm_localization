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
