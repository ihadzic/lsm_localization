#ifndef SEGMENT_TREE_H
#define SEGMENT_TREE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility>
#include <vector>
#include <memory>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <string>

#undef min
#undef max

namespace scan_tools
{

using Point = Eigen::Vector2d;
struct Segment: std::pair<Point, Point>
{
  // Inherit constructor
  using std::pair<Point, Point>::pair;

  void flip() {std::swap(first, second);};
  Eigen::Vector2d diff() const {return second - first;}
  Eigen::Vector2d surface_normal() const
  {
    const auto d = diff();
    return Eigen::Vector2d{d[1], -d[0]}/d.norm();
  }
};


class SegmentTree
{
  private:
    template<int Axis>
    struct Node
    {
      constexpr static auto leaf_capacity = 64;
      constexpr static auto ChildAxis = (Axis+1)%2;
      using ChildNode = Node<ChildAxis>;
      using Ptr = std::unique_ptr<Node>;
      using ChildPtr = typename ChildNode::Ptr;

      ChildPtr left_, right_;
      double pivot_;
      std::vector<Segment> segments_;

      Node(std::vector<Segment> segs);

      Node() = default;
      Node(Node&&) = default;
      Node& operator= (Node&&) = default;

      Node(const Node&) = delete;
      Node& operator= (const Node&) = delete;

      template<class Callback>
      void segments_within(Point p, double radius, const Callback &cb) const;

      template<class Callback>
      void traverse(const Callback &cb, int depth=0) const;
    };

  public:
    SegmentTree(std::vector<Segment> segments);

    SegmentTree() = default;
    SegmentTree(SegmentTree&&) = default;
    SegmentTree& operator= (SegmentTree&&) = default;

    SegmentTree(const SegmentTree&) = delete;
    SegmentTree& operator= (const SegmentTree&) = delete;

  public:
    std::vector<Segment> segments_within(Point p, double radius) const;

  public:
    void dump(std::ostream &os) const;

  private:
    Node<0> root_;
};

}


#endif // KD_TREE_H
