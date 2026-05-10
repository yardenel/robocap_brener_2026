#pragma once

class Vec2f {
   public:
    double x, y;
    Vec2f();
    Vec2f(const double& x, const double& y);
    Vec2f(const Vec2f& other) = default;
    Vec2f(Vec2f&& other)      = default;
    Vec2f& operator=(const Vec2f& other);
    Vec2f& operator=(Vec2f&& other);
    Vec2f operator-(const Vec2f& other);
    Vec2f operator+(const Vec2f& other);
    Vec2f operator*(const Vec2f& other);
    Vec2f operator/(const Vec2f& other);
};
