#include <Vec2f.hpp>

Vec2f::Vec2f()
    : x(0)
    , y(0) {}

Vec2f& Vec2f::operator=(const Vec2f& other) {
    x = other.x;
    y = other.y;
    return *this;
}

Vec2f& Vec2f::operator=(Vec2f&& other) {
    x = other.x;
    y = other.y;
    return *this;
}

Vec2f Vec2f::operator+(const Vec2f& other) {
    return Vec2f(x + other.x, y + other.y);
}

Vec2f Vec2f::operator-(const Vec2f& other) {
    return Vec2f(x - other.x, y - other.y);
}

Vec2f Vec2f::operator*(const Vec2f& other) {
    return Vec2f(x * other.x, y * other.y);
}

Vec2f Vec2f::operator/(const Vec2f& other) {
    return Vec2f(x / other.x, y / other.y);
};