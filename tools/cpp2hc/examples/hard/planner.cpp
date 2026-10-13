#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_planners
{
class Point
{
public:
  Point(double x, double y) : x_(x), y_(y) {}
  double dist(const Point & o) const
  {
    return std::hypot(o.x_ - x_, o.y_ - y_);
  }
  double heading(const Point & o) const { return std::atan2(o.y_ - y_, o.x_ - x_); }
  double x_, y_;
};

class StraightLine : public nav_base::GlobalPlanner
{
public:
  StraightLine() : step_(0.1), count_(0u) {}
  ~StraightLine() { delete last_; }

  void configure(const std::string & name, rclcpp::Node::SharedPtr node)
  {
    name_ = name;
    node_ = node;
    RCLCPP_INFO(node->get_logger(), "configured %s step %lf", name.c_str(), step_);
  }

  int plan(Point * start, Point * goal, std::vector<Point> & out)
  {
    auto d = start->dist(*goal);
    int n = static_cast<int>(d / step_);
    for (int i = 0; i <= n; ++i) {
      out.push_back(Point(0, 0));
    }
    last_ = new Point(goal->x_, goal->y_);
    std::cout << "planned " << n << " points for " << name_ << std::endl;
    count_ += 1;
    return n;
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  double step_;
  unsigned int count_;
  Point * last_ = nullptr;
};
}

PLUGINLIB_EXPORT_CLASS(my_planners::StraightLine, nav_base::GlobalPlanner)
