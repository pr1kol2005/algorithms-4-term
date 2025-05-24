#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

struct Point {
  int64_t x;
  int64_t y;

  Point() = default;

  Point(int64_t x1, int64_t y1) : x(x1), y(y1) {}

  bool operator<(const Point& p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  bool operator==(const Point& p) const { return x == p.x && y == p.y; }

  friend std::istream& operator>>(std::istream& is, Point& p) {
    is >> p.x >> p.y;
    return is;
  }

  friend std::ostream& operator<<(std::ostream& os, Point& p) {
    os << p.x << ' ' << p.y;
    return os;
  }
};

struct Polygon {
  std::vector<Point> vertices;

  Polygon() : vertices() {}

  explicit Polygon(size_t n) : vertices(n) {}

  friend std::istream& operator>>(std::istream& is, Polygon& polygon) {
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
      is >> polygon.vertices[i];
    }
    return is;
  }
};

struct Vector {
  int64_t x;
  int64_t y;

  Vector() = default;

  Vector(int64_t x1, int64_t y1) : x(x1), y(y1) {}

  Vector(int64_t x1, int64_t y1, int64_t x2, int64_t y2) {
    x = x2 - x1;
    y = y2 - y1;
  }

  Vector(Point p1, Point p2) {
    x = p2.x - p1.x;
    y = p2.y - p1.y;
  }

  int64_t LengthSquared() const { return x * x + y * y; }

  int64_t CrossProduct(const Vector& other) const {
    return x * other.y - y * other.x;
  }

  int64_t ScalarProduct(const Vector& other) const {
    return x * other.x + y * other.y;
  }

  friend std::istream& operator>>(std::istream& is, Vector& v) {
    int64_t x1 = 0;
    int64_t y1 = 0;
    int64_t x2 = 0;
    int64_t y2 = 0;
    is >> x1 >> y1 >> x2 >> y2;
    v.x = x2 - x1;
    v.y = y2 - y1;
    return is;
  }
};

int64_t CrossProduct(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

std::vector<Point> GrahamScan(std::vector<Point>& points) {
  size_t n = points.size();
  size_t k = 0;
  std::vector<Point> hull(2 * n);
  std::sort(points.begin(), points.end());

  for (size_t i = 0; i < n; ++i) {
    while (k >= 2 && CrossProduct(hull[k - 2], hull[k - 1], points[i]) < 0) {
      k--;
    }
    hull[k++] = points[i];
  }

  for (size_t i = n - 1, t = k + 1; i > 0; --i) {
    while (k >= t &&
           CrossProduct(hull[k - 2], hull[k - 1], points[i - 1]) < 0) {
      k--;
    }
    hull[k++] = points[i - 1];
  }

  hull.resize(k - 1);
  return hull;
}

void RemoveElements(std::vector<Point>& target,
                    const std::vector<Point>& source) {
  for (const auto& elem : source) {
    target.erase(std::remove(target.begin(), target.end(), elem), target.end());
  }
}

struct Normal {
  int64_t a;
  int64_t b;
  int64_t c;

  Normal() : a(0), b(0), c(0) {}

  Normal(int64_t a, int64_t b, int64_t c) : a(a), b(b), c(c) {}

  Normal(Point first, Point second) {
    int64_t x = second.x - first.x;
    int64_t y = second.y - first.y;
    a = -y;
    b = x;
    c = -a * first.x + -b * first.y;
  }
};

bool PointCheck(Point first, Point second, Point point) {
  Normal normal(first, second);

  if (normal.a == 0) {
    return false;
  }

  double x_sol = 0;
  x_sol = static_cast<double>((-normal.c - normal.b * point.y)) /
          static_cast<double>(normal.a);

  return (x_sol <= static_cast<double>(point.x) &&
          (first.y - point.y) * (second.y - point.y) <= 0);
}

bool TimesCrossesSides(Point first, Point second, Point point) {
  if (PointCheck(first, second, point)) {
    if (point.y == first.y || point.y == second.y) {
      return (2 * point.y >= first.y + second.y);
    }

    return true;
  }

  return false;
}

bool BelongsToSection(Point first, Point second, Point point) {
  Normal normal(first, second);

  bool flag1 = (((first.x - point.x) * (second.x - point.x) <= 0) &&
                ((first.y - point.y) * (second.y - point.y) <= 0));

  bool flag2 = (normal.a * point.x + normal.b * point.y + normal.c == 0);

  return flag1 && flag2;
}

bool IsPointInsidePolygon(std::vector<Point> vertices, Point point) {
  bool result = false;
  for (size_t i = 1; i <= vertices.size(); ++i) {
    if (BelongsToSection(vertices[i % vertices.size()],
                         vertices[(i + 1) % vertices.size()], point)) {
      return true;
    }

    result =
        result ^ TimesCrossesSides(vertices[i % vertices.size()],
                                   vertices[(i + 1) % vertices.size()], point);
  }

  return result;
}

int main() {
  size_t n = 0;
  std::cin >> n;
  std::vector<Point> attractions(n);
  for (size_t i = 0; i < n; i++) {
    std::cin >> attractions[i];
  }

  size_t k = 0;
  std::cin >> k;
  std::vector<Point> metro_stations(k);
  for (size_t i = 0; i < k; i++) {
    std::cin >> metro_stations[i];
  }

  std::vector<int> result(k, 0);

  bool in_zero_zone = true;

  std::vector<Point> hull = GrahamScan(attractions);

  while (attractions.size() > 0) {
    for (size_t j = 0; j < k; j++) {
      if (IsPointInsidePolygon(hull, metro_stations[j]) && in_zero_zone) {
        result[j] += 1;
      }
    }

    RemoveElements(attractions, hull);

    if (!attractions.empty()) {
      hull = GrahamScan(attractions);
    }

    in_zero_zone = true;
  }

  for (size_t i = 0; i < k; i++) {
    std::cout << result[i] << '\n';
  }

  return 0;
}
