#include <laser_scan_matcher/scan_constructor.h>

#include <algorithm>

#include <cmath>

namespace scan_tools
{

ScanConstructor::ScanConstructor(const grid_t &grid,
                                 const map_params_t &map_params):
  map_grid_(grid), map_params_(map_params)
{
}

ScanConstructor::ScanConstructor(grid_t &&grid,
                                 const map_params_t &map_params):
  map_grid_(std::move(grid)), map_params_(map_params)
{
}

ScanConstructor::scan_t
ScanConstructor::constructScan(double laser_x, double laser_y,
                                    double laser_yaw,
                                    const scan_params_t &scan_params) const
{
  const auto range_min = scan_params.range_min;
  const auto range_max = scan_params.range_max;
  const auto angle_min = scan_params.angle_min;
  const auto angle_max = scan_params.angle_max;
  const auto angle_inc = scan_params.angle_inc;

  const auto max_allowed_range_ = scan_params.max_allowed_range;

  const auto map_res_ = map_params_.map_res;
  const auto map_width_ = map_params_.map_width;
  const auto map_height_ = map_params_.map_height;
  const auto map_occupancy_threshold_ = map_params_.map_occupancy_threshold;

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

  scan_t constructed_ranges_(num_angles, -1.0);

  // scan all points in the area of interest, convert to polar and
  // remember the point if it is currently the closest for the given point
  for (int y = y0; y < y0 + h; y++) {
    for (int x = x0; x < x0 + w; x++) {
      if (map_grid_[y][x] > map_occupancy_threshold_) {
        // add .5 to reference the middle of the pixel
        double delta_x = (x + .5) * map_res_ - laser_x;
        double delta_y = (y + .5) * map_res_ - laser_y;

        // go to polar cordinates relative to the scanner heading
        // calculate incident angles for all four corners of the pixel
        const double xs[] = {delta_x - .5*map_res_, delta_x + .5*map_res_};
        const double ys[] = {delta_y - .5*map_res_, delta_y + .5*map_res_};

        std::vector<double> incident_angles;
        for (auto x : xs)  for (auto y : ys)
        {
          double theta = atan2(y, x) - laser_yaw;

          // ensure that theta is in -pi to pi range
          while (theta < -M_PI) theta += 2 * M_PI;
          while (theta >= M_PI) theta -= 2 * M_PI;
          // if LIDAR range uses only positive angles, make the angle compatible with it
          if (angle_min >= 0 && theta < 0.0) theta += 2 * M_PI;
          if (theta < angle_min) continue;
          if (theta > angle_max) continue;
          incident_angles.push_back(theta);
        }

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
        int start_index = (int)((start_theta - angle_min) / angle_inc) % num_angles;
        int n_thetas = (int)(delta_theta / angle_inc) + 1;
        for (int i = 0; i <= n_thetas; i++) {
          const auto theta_index = (start_index + i) % num_angles;
          const auto scan_theta = angle_min + theta_index*angle_inc;
          const auto theta = laser_yaw + scan_theta;
          const auto t = tan(theta);

          // find rho for this point along the pixel's border
          auto best_rho = range_max;
          for (const auto x : xs) {
            if (t*x >= ys[0] && t*x <= ys[1])
              best_rho = std::min(best_rho, x/cos(theta));
          }
          for (const auto y : ys) {
            if (y/t >= xs[0] && y/t <= xs[1])
              best_rho = std::min(best_rho, y/sin(theta));
          }

          // Check if beam is valid
          if (best_rho < range_min || best_rho >= range_max)
            continue;
          if (max_allowed_range_ > 0 && best_rho > max_allowed_range_)
            continue;

          // either no point ever recorded for this angle, so take it
          // or the current point is closer than previously recorded point
          if (constructed_ranges_[theta_index] <= 0.0 ||
            best_rho < constructed_ranges_[theta_index]) {
            constructed_ranges_[theta_index] = best_rho;
          }
        }
      }
    }
  }

  return constructed_ranges_;
}




}
