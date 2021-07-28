#include <laser_scan_matcher/segment_tree.h>

namespace scan_tools
{

double min_dist_to_segment(const Point &p, const Segment &seg)
{
  const auto diff = seg.diff();
  const auto len2 = diff.squaredNorm();

  // t represents location along seg where p projects, clamped to the endpoints
  const auto t = std::max(0.0, std::min(1.0, (p - seg.first).dot(diff)/len2));
  const Point projection = seg.first + t*diff;

  return (p - projection).norm();
}

SegmentTree::SegmentTree(std::vector<Segment> segs):
  root_(std::move(segs)) {}

template<int Axis>
SegmentTree::Node<Axis>::Node(std::vector<Segment> segs)
{
  const auto n = segs.size();

  if (n <= leaf_capacity)
  {
    segments_ = std::move(segs);
    return;
  }

  pivot_ = 0;
  // Center of segments
  for (const auto &seg : segs)
  {
    pivot_ += (seg.first[Axis] + seg.second[Axis]);
  }
  pivot_ /= 2*n;

  // Partition segs as follows: [split, left, right]
  auto first_left_seg = std::partition(segs.begin(), segs.end(),
                                       [this](const Segment &seg)
                                       {
                                         return (seg.first[Axis] < pivot_) != (seg.second[Axis] < pivot_);
                                       });

  auto first_right_seg = std::partition(first_left_seg, segs.end(),
                                        [this](const Segment &seg)
                                        {
                                          return seg.first[Axis] < pivot_;
                                        });

  std::vector<Segment> right_segs(first_right_seg, segs.end());
  segs.erase(first_right_seg, segs.end());

  for (auto i = segs.begin(); i != first_left_seg; ++i)
  {
    // Split the segments split by the pivot into 2 smaller segments
    // Order along Axis must be first, pivot_, second, either plus or minus.
    const auto diff = i->diff();
    const double factor = (pivot_ - i->first[Axis])/diff[Axis];

    Segment left{i->first, i->first + diff*factor};
    Segment right{i->first + diff*factor, i->second};

    // This segment goes backwards
    if (diff[Axis] < 0) std::swap(left, right);

    // segs is the new left_segs vector.
    *i = left;
    right_segs.push_back(right);
  }

  if (segs.size()) left_.reset(new ChildNode(std::move(segs)));
  if (right_segs.size()) right_.reset(new ChildNode(std::move(right_segs)));
}

template<int Axis>
template<class Callback>
void SegmentTree::Node<Axis>::segments_within(
  Point p, double radius, const Callback &cb) const
{
  for (const auto& seg : segments_)
    if (min_dist_to_segment(p, seg) <= radius)
      cb(seg);

    if (left_ && p[Axis] - radius <= pivot_)
      left_->segments_within(p, radius, cb);

    if (right_ && p[Axis] + radius >= pivot_)
      right_->segments_within(p, radius, cb);
}

std::vector<Segment> SegmentTree::segments_within(Point p, double radius) const
{
  std::vector<Segment> ret;
  root_.segments_within(p, radius, [&ret](const Segment& seg)
  {
    ret.push_back(seg);
  });

  return ret;
}

template<int Axis>
template<class Callback>
void SegmentTree::Node<Axis>::traverse(const Callback &cb, int depth) const
{
  if (left_) left_->traverse(cb, depth+1);
  cb(segments_, pivot_, Axis, depth);
  if (right_) right_->traverse(cb, depth+1);
}

void SegmentTree::dump(std::ostream &os) const
{
  root_.traverse([&os](const std::vector<Segment>& segs,
                        const double pivot, int axis, int depth)
  {
    os << std::string(10+depth*2, ' ');
    if (segs.size())
      os << segs.size() << " segments";
    else
      os << axis << ":" << pivot;

    os << std::endl;
  });
}

}
