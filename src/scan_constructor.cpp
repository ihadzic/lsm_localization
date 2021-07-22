#include <laser_scan_matcher/scan_constructor.h>

#include <potracelib.h>

#include <algorithm>
#include <utility>
#include <memory>
#include <stdexcept>

#include <cmath>

namespace scan_tools
{

/* See section 2.1.2 of potracelib API manual
 */
std::pair<potrace_bitmap_t, std::unique_ptr<potrace_word[]> >
prepareBitmap(const ScanConstructor::grid_t &grid,
              int map_occupancy_threshold)
{
  constexpr auto N = 8*sizeof(potrace_word);

  potrace_bitmap_t ret;
  ret.h = grid.size();
  ret.w = grid[0].size();
  ret.dy = (ret.w - 1)/N + 1;
  std::unique_ptr<potrace_word[]> data(new potrace_word[ret.dy*ret.h]);
  ret.map = data.get();

  for (auto y = 0u; y < ret.h; ++y)
  {
    // initialize
    for (auto w = 0u; w < ret.dy; ++w) ret.map[y*ret.dy + w] = 0;

    for (auto x = 0u; x < ret.w; ++x)
    {
      const bool val = (grid[y][x] >= map_occupancy_threshold);
      const auto bit = (N - 1 - x%N);

      ret.map[y*ret.dy + x/N] |= (potrace_word(val) << bit);
    }
  }

  return std::make_pair(ret, std::move(data));
}

template<int N>
Point bezier(const Point* points, double t)
{
  return (1-t)*bezier<N-1>(points, t) + t*bezier<N-1>(points+1, t);
}

template<>
Point bezier<1>(const Point* point, double t)
{
  return *point;
}

template<int N>
Point bezier(const Point (&points)[N], double t)
{
  return bezier<N>(&points[0], t);
}

Point potrace_to_point(const potrace_dpoint_t &p)
{
  return {p.x, p.y};
}

// determinant
double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return a[0]*b[1] - a[1]*b[0];
}

std::vector<Segment>
extractSegments(const potrace_path_t* path_head)
{
  std::vector<Segment> segments;

  for (auto path = path_head; path != nullptr; path = path->next)
  {
    const auto &curve = path->curve;

    auto seg_start = potrace_to_point(curve.c[curve.n - 1][2]);
    for (auto i = 0; i < curve.n; ++i)
    {
      const auto seg_end = potrace_to_point(curve.c[i][2]);

      if (curve.tag[i] == POTRACE_CORNER)
      {
        const auto corner = potrace_to_point(curve.c[i][1]);
        segments.push_back(Segment{seg_start, corner});
        segments.push_back(Segment{corner, seg_end});
      }
      else if (curve.tag[i] == POTRACE_CURVETO)
      {
        constexpr auto segs_per_curve = 4;  // Totally arbitrary

        const Point points[] = {
          seg_start,
          potrace_to_point(curve.c[i][0]),
          potrace_to_point(curve.c[i][1]),
          seg_end
        };

        Point prev = points[0];
        for (auto j = 1; j <= segs_per_curve; ++j)
        {
          const double t = double(j)/segs_per_curve;
          const Point p = bezier(points, t);
          segments.push_back(Segment{prev, p});
          prev = p;
        }
      }

      seg_start = seg_end;
    }
  }

  return segments;
}

ScanConstructor::ScanConstructor(const grid_t &grid,
                                 const map_params_t &map_params):
  map_params_(map_params)
{
  const auto &p = prepareBitmap(grid, map_params.map_occupancy_threshold);
  const auto &bmp = p.first;

  auto potrace_params =
    std::unique_ptr<potrace_param_t, decltype(&potrace_param_free)>(
      potrace_param_default(), &potrace_param_free);

  potrace_params->turdsize = 5;
  potrace_params->alphamax = 0.6;
  potrace_params->opticurve = false;

  auto potrace_state =
    std::unique_ptr<potrace_state_t, decltype(&potrace_state_free)>(
      potrace_trace(potrace_params.get(), &bmp), &potrace_state_free);

  if (potrace_state->status != POTRACE_STATUS_OK)
    throw std::runtime_error("ScanConstructor: potrace_trace failed to trace");

  auto extracted_segments = extractSegments(potrace_state->plist);

  for (auto &seg : extracted_segments)
  {
    const auto normal = seg.surface_normal();

    // Adjust coordinates so they go through the centers of the edge pixels
    seg.first -= Point{0.5, 0.5};
    seg.second -= Point{0.5, 0.5};

    // Hypothetically this would be the unbiased reconstruction
    // but in practice it seems that edge occupied grid cells don't have the
    // edge at the center on average.
    //seg.first -= 0.5*normal;
    //seg.second -= 0.5*normal;

    // Use world coordinates
    seg.first *= map_params.map_res;
    seg.second *= map_params.map_res;
  }

  segments_ = SegmentTree(std::move(extracted_segments));
}

ScanConstructor::ScanConstructor(grid_t &&grid,
                                 const map_params_t &map_params):
  ScanConstructor(grid, map_params)
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
  const auto max_allowed_range = scan_params.max_allowed_range;

  Point scan_origin{laser_x, laser_y};

  auto neighborhood = segments_.segments_within(scan_origin, range_max);

  int num_angles = (int)round((angle_max - angle_min + angle_inc) / angle_inc);
  scan_t constructed_ranges(num_angles, -1.0);

  for (const auto &world_seg : neighborhood)
  {
    const Segment local_seg{
      world_seg.first - scan_origin,
      world_seg.second - scan_origin
    };

    // skip surfaces facing away from the scanner
    if (local_seg.first.dot(local_seg.surface_normal()) > 0) continue;

    const auto seg_delta = local_seg.diff();

    const double thetas[] = {
      std::atan2(local_seg.first[1], local_seg.first[0]) - laser_yaw,
      std::atan2(local_seg.second[1], local_seg.second[0]) - laser_yaw
    };

    const double min_theta = std::min(thetas[0], thetas[1]);
    const double max_theta = std::max(thetas[0], thetas[1]);

    double delta_theta = max_theta - min_theta;
    double start_theta = min_theta;
    // handle wrap-around the circle
    if (delta_theta > M_PI) {
      start_theta = max_theta;
      delta_theta = 2 * M_PI - delta_theta;
    }

    const int start_index =
      (int)((start_theta - angle_min)/angle_inc + num_angles) % num_angles;

    const int n_thetas = (int)(delta_theta / angle_inc) + 1;

    for (int i = 0; i <= n_thetas; i++)
    {
      const auto theta_index = (start_index + i) % num_angles;
      const auto scan_theta = angle_min + theta_index*angle_inc;
      const auto theta = laser_yaw + scan_theta;

      const Eigen::Vector2d laser{std::cos(theta), std::sin(theta)};

      // distance from origin to intersection with line 'seg' along angle theta
      const auto rho =
        cross2(local_seg.first, seg_delta)/cross2(laser, seg_delta);

      // weary of numeric degeneracy
      // manually ensure the intersection point is still within the segment
      const double t = (rho*laser - local_seg.first).dot(seg_delta)/
                          seg_delta.squaredNorm();

      if (t < 0.0 || t > 1.0) continue;

      // Check if beam is valid
      if (rho < range_min || rho >= range_max)
        continue;
      if (max_allowed_range > 0 && rho > max_allowed_range)
        continue;

      // either no point ever recorded for this angle, so take it
      // or the current point is closer than previously recorded point
      if (constructed_ranges[theta_index] <= 0.0
          || rho < constructed_ranges[theta_index])
      {
        constructed_ranges[theta_index] = rho;
      }
    }
  }

  return constructed_ranges;
}

} // scan_tools
