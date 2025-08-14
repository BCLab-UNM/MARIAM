#include<vector>

// Cubic spline segment: f(t) = a + b*t + c*t^2 + d*t^3
struct SplineSegment {
  double a, b, c, d;
  double t_start, t_end;
  
  double evaluate(double t) const {
    double dt = t - t_start;
    double dt_squared = dt * dt;
    double dt_cubed = dt * dt * dt;
    return a + b * dt + c * dt_squared + d * dt_cubed;
  }
  
  double derivative(double t) const {
    double dt = t - t_start;
    double dt_squared = dt * dt;
    return b + 2 * c * dt + 3 * d * dt_squared;
  }
  
  double second_derivative(double t) const {
    double dt = t - t_start;
    return 2 * c + 6 * d * dt;
  }
};

// Cubic spline class
class CubicSpline {
  public:
  /**
   * This function will set 
   */
  void setPoints(const std::vector<double>& t, const std::vector<double>& y) {
    if (t.size() != y.size() || t.size() < 2) {
      throw std::invalid_argument("Invalid input vectors for spline");
    }
      
    int n = t.size(); // N + 1 points
    segments_.clear();
    segments_.reserve(n - 1); // reserve space for N splines
      
    // Setup system of equations Ac = b
    // the system is (N+1) x (N+) = (n) x (n)
    std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
    std::vector<double> b(n, 0.0);
    
    // Set natural boundary conditions: second derivatives = 0 at endpoints
    A[0][0] = 1.0;
    A[n-1][n-1] = 1.0;
    b[0] = 0.0;
    b[n-1] = 0.0;
    
    // Set interior points (continuity of second derivative)
    for (int i = 1; i < n - 1; ++i) {
      double h1 = t[i] - t[i-1];
      double h2 = t[i+1] - t[i];
      
      A[i][i-1] = h1;
      A[i][i] = 2.0 * (h1 + h2);
      A[i][i+1] = h2;
      
      b[i] = 6.0 * ((y[i+1] - y[i]) / h2 - (y[i] - y[i-1]) / h1);
    }
    
    // Solve tridiagonal system for second derivatives
    std::vector<double> c = solveTridiagonal(A, b);
    
    // Build spline segments
    for (int i = 0; i < n - 1; ++i) {
      SplineSegment seg;
      double h = t[i+1] - t[i];
      
      seg.t_start = t[i];
      seg.t_end = t[i+1];
      
      seg.a = y[i];
      seg.b = (y[i+1] - y[i]) / h - h * (2.0 * c[i] + c[i+1]) / 6.0;
      seg.c = c[i] / 2.0;
      seg.d = (c[i+1] - c[i]) / (6.0 * h);
      
      segments_.push_back(seg);
    }
  }
    
  double evaluate(double t) const {
    auto seg = findSegment(t);
    return seg->evaluate(t);
  }
    
  double derivative(double t) const {
    auto seg = findSegment(t);
    return seg->derivative(t);
  }
    
  double secondDerivative(double t) const {
    auto seg = findSegment(t);
    return seg->second_derivative(t);
  }

private:
  std::vector<SplineSegment> segments_;
  
  const SplineSegment* findSegment(double t) const {
    // Handle edge cases
    if (t <= segments_[0].t_start) return &segments_[0];
    if (t >= segments_.back().t_end) return &segments_.back();
    
    // Binary search for the correct segment
    auto it = std::lower_bound(segments_.begin(), segments_.end(), t,
        [](const SplineSegment& seg, double t) { return seg.t_end < t; });
    
    return &(*it);
  }
  
  // Simple Gaussian elimination for tridiagonal matrix
  std::vector<double> solveTridiagonal(std::vector<std::vector<double>>& A, std::vector<double>& b) {
    int n = b.size();
      
    // Forward elimination
    for (int i = 1; i < n; ++i) {
      double factor = A[i][i-1] / A[i-1][i-1];
      A[i][i] -= factor * A[i-1][i];
      b[i] -= factor * b[i-1];
    }
    
    // Back substitution
    std::vector<double> x(n);
    x[n-1] = b[n-1] / A[n-1][n-1];
    for (int i = n - 2; i >= 0; --i) {
      x[i] = (b[i] - A[i][i+1] * x[i+1]) / A[i][i];
    }
    
    return x;
  }
};

// 2D trajectory using two splines
struct SplineTrajectory2D {
    CubicSpline x_spline;
    CubicSpline y_spline;
    double duration;
    
    struct Point2D {
      double x, y;
      double vx, vy;
      double ax, ay;
    };
    
    Point2D getPoint(double t) const {
      Point2D point;
      point.x = x_spline.evaluate(t);
      point.y = y_spline.evaluate(t);
      point.vx = x_spline.derivative(t);
      point.vy = y_spline.derivative(t);
      point.ax = x_spline.secondDerivative(t);
      point.ay = y_spline.secondDerivative(t);
      return point;
    }
};