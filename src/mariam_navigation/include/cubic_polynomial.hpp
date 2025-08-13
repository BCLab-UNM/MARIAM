#include <vector>
#include <cmath>

// A struct to hold the coefficients of a cubic polynomial
struct CubicPolynomial {
  double a, b, c, d;
  
  // Evaluate the polynomial at a given time t
  double position(double t) const {
    return a * std::pow(t, 3) + b * std::pow(t, 2) + c * t + d;
  }
  
  // Evaluate the first derivative (velocity)
  double velocity(double t) const {
    return 3 * a * std::pow(t, 2) + 2 * b * t + c;
  }
  
  // Evaluate the second derivative (acceleration)
  double acceleration(double t) const {
    return 6 * a * t + 2 * b;
  }
};

// 2D trajectory using two cubic polynomials
struct Trajectory2D {
  CubicPolynomial x_poly;
  CubicPolynomial y_poly;
  double duration;
  
  struct Point2D {
    double x, y;
    double vx, vy;
    double ax, ay;
  };
  
  Point2D getPoint(double t) const {
    Point2D point;
    point.x = x_poly.position(t);
    point.y = y_poly.position(t);
    point.vx = x_poly.velocity(t);
    point.vy = y_poly.velocity(t);
    point.ax = x_poly.acceleration(t);
    point.ay = y_poly.acceleration(t);
    return point;
  }
};

