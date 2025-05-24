#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>
#include <cstdlib>

template <typename T>
struct Point {
  T x;
  T y;

  Point() = default;

  Point(T x1, T y1) : x(x1), y(y1) {}

  bool operator<(const Point& p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }

  Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }

  Point operator-() const { return Point(-x, -y); }

  friend std::istream& operator>>(std::istream& is, Point& p) {
    is >> p.x >> p.y;
    return is;
  }

  friend std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << p.x << ' ' << p.y;
    return os;
  }
};

template <typename T>
struct Vector {
  T x;
  T y;

  Vector() = default;

  Vector(T x1, T y1) : x(x1), y(y1) {}

  Vector(T x1, T y1, T x2, T y2) {
    x = x2 - x1;
    y = y2 - y1;
  }

  Vector(Point<T> start, Point<T> end) {
    x = end.x - start.x;
    y = end.y - start.y;
  }

  T LengthSquared() const { return x * x + y * y; }

  T CrossProduct(const Vector& other) const {
    return x * other.y - y * other.x;
  }

  T ScalarProduct(const Vector& other) const {
    return x * other.x + y * other.y;
  }

  void Upscale(T scale) {
    x *= scale;
    y *= scale;
  }

  friend std::istream& operator>>(std::istream& is, Vector& v) {
    Point<T> start;
    Point<T> end;
    is >> start >> end;
    v.x = end.x - start.x;
    v.y = end.y - start.y;
    return is;
  }
};

template <typename T>
struct Polygon {
  std::vector<Point<T>> vertices;

  Polygon() : vertices() {}

  explicit Polygon(size_t n) : vertices(n) {}

  explicit Polygon(const std::vector<Point<T>>& points) : vertices(points) {}

  size_t size() const { return vertices.size(); }

  Point<T>& operator[](size_t index) { return vertices[index]; }

  const Point<T>& operator[](size_t index) const { return vertices[index]; }

  Polygon operator-() const {
    Polygon result(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
      result[i] = -vertices[i];
    }
    return result;
  }

  friend std::istream& operator>>(std::istream& is, Polygon& polygon) {
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
      is >> polygon.vertices[i];
    }
    return is;
  }
};

template <typename T>
T CrossProduct(const Point<T>& a, const Point<T>& b, const Point<T>& c) {
  return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

template <typename T>
T CrossProduct(const Point<T>& A, const Point<T>& B) {
  return A.x * B.y - A.y * B.x;
}

template <typename T>
Polygon<T> GrahamScan(const Polygon<T>& poly) {
  std::vector<Point<T>> points = poly.vertices;
  size_t n = points.size();
  size_t k = 0;
  std::vector<Point<T>> hull(2 * n);

  std::sort(points.begin(), points.end());

  for (size_t i = 0; i < n; ++i) {
    while (k >= 2 && CrossProduct(hull[k - 2], hull[k - 1], points[i]) < 0) {
      --k;
    }
    hull[k++] = points[i];
  }

  for (size_t i = n - 1, t = i + 1; i > 0; --i) {
    while (k >= t && CrossProduct(hull[k - 2], hull[k - 1], points[i - 1]) < 0) {
      --k;
    }
    hull[k++] = points[i - 1];
  }

  hull.resize(k - 1);

  return Polygon<T>(hull);
}

template <typename T>
Polygon<T> MinkowskiSum(const Polygon<T>& A, const Polygon<T>& B) {
  size_t n = A.size();
  size_t m = B.size();

  size_t start_A = std::min_element(A.vertices.begin(), A.vertices.end()) - A.vertices.begin();
  size_t start_B = std::min_element(B.vertices.begin(), B.vertices.end()) - B.vertices.begin();

  Polygon<T> result;

  size_t i_A = start_A;
  size_t i_B = start_B;

  Point<T> current = A[i_A] + B[i_B];
  do {
    result.vertices.push_back(current);
    Point<T> edge_A = A[(i_A + 1) % n] - A[i_A];
    Point<T> edge_B = B[(i_B + 1) % m] - B[i_B];
    T cross_product = CrossProduct(edge_A, edge_B);

    if (cross_product > 0) {
      i_A = (i_A + 1) % n;
      current = current + edge_A;
      continue;
    } 
    if (cross_product < 0) {
      i_B = (i_B + 1) % m;
      current = current + edge_B;
      continue;
    }
    i_A = (i_A + 1) % n;
    i_B = (i_B + 1) % m;
    current = current + (edge_A + edge_B);
  } while (i_A != start_A || i_B != start_B);

  return result;
}

int main() {
  size_t n = 0;
  size_t m = 0;

  std::cin >> n >> m;

  Polygon<int64_t> airport(n);
  std::cin >> airport;

  Polygon<int64_t> cloud(m);
  std::cin >> cloud;

  Polygon<int64_t> minkowski_sum = MinkowskiSum(airport, -cloud);


  double result = std::numeric_limits<double>::infinity();
  size_t k = minkowski_sum.size();
  for (size_t i = 0; i < k; ++i) {
    Point<int64_t> a = minkowski_sum[i];
    Point<int64_t> b = minkowski_sum[(i + 1) % k];
    double edge_length = std::sqrt(Vector(a, b).LengthSquared());
    double dist =
        std::abs((CrossProduct(a, b)) / edge_length);
    result = std::min(result, dist);
  }

  std::cout << std::fixed << std::setprecision(6)
            << std::max(0.0L, result - 60.0L) << '\n';
}
