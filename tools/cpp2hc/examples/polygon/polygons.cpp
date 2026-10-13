#include <cmath>
#include <iostream>
#include <polygon_base/regular_polygon.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace polygon_plugins
{
  // A square: four equal sides.
  class Square : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
        calls_ = 0;
      }

      double area() override
      {
        calls_++;
        return side_length_ * side_length_;
      }

      void describe()
      {
        std::cout << "Square with side " << side_length_ << " area " << area() << std::endl;
      }

    protected:
      double side_length_ = 1.0f;
      int calls_;
  };

  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override;
      double area() override;

    protected:
      double side_length_;
      double getHeight();
  };

  void Triangle::initialize(double side_length)
  {
    side_length_ = side_length;
  }

  double Triangle::area()
  {
    return 0.5 * side_length_ * getHeight();
  }

  double Triangle::getHeight()
  {
    double sum = 0.0;
    for (int i = 0; i < 3; i++) {
      sum += 1.0;
    }
    printf("height for side %lf (%zu sides)\n", side_length_, (size_t)3);
    return std::sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
  }
}

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
