#ifndef SCAN_CONSTRUCTOR_H
#define SCAN_CONSTRUCTOR_H

#include <laser_scan_matcher/segment_tree.h>

#include <vector>
#include <cmath>

namespace scan_tools
{

class ScanConstructor
{
  public:
    using grid_t = std::vector<std::vector<int> >;
    using scan_t = std::vector<double>;
    struct map_params_t
    {
      double map_res;
      int map_width;
      int map_height;
      int map_occupancy_threshold;
    };

    struct scan_params_t
    {
      double range_min;
      double range_max;
      double angle_min = 0;
      double angle_max = 2*M_PI;
      double angle_inc;
      double max_allowed_range;
    };

  public:
    ScanConstructor() = default;
    ScanConstructor(const grid_t &grid, const map_params_t &map_params);
    ScanConstructor(grid_t &&grid, const map_params_t &map_params);

    ScanConstructor(const ScanConstructor&) = default;
    ScanConstructor(ScanConstructor&&) = default;

    ScanConstructor& operator= (const ScanConstructor&) = default;
    ScanConstructor& operator= (ScanConstructor&&) = default;

    ~ScanConstructor() = default;

  public:
    const map_params_t& map_params() const {return map_params_;}

  public:
    scan_t constructScan(double x, double y, double yaw,
                         const scan_params_t &scan_params) const;

  private:
    SegmentTree segments_;
    map_params_t map_params_;
};





}


#endif
