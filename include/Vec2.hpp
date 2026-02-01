#pragma once
#include "concepts.hpp"

template <Arithmetic T>
class Vec2 {
   public:
    T x, y;
    Vec2(const T& x, const T& y);
    Vec2(const Vec2<T>& other) = default;
    Vec2(Vec2<T>&& other)      = default;
    Vec2<T>& operator=(const Vec2<T>& other);
    Vec2<T>& operator=(Vec2<T>&& other);
    Vec2<T> operator-(const Vec2<T>& other);
    Vec2<T> operator+(const Vec2<T>& other);
    Vec2<T> operator*(const Vec2<T>& other);
    Vec2<T> operator/(const Vec2<T>& other);
};

template <typename T>
Vec2<T>& Vec2<T>::operator=(const Vec2<T>& other) {
    x = other.x;
    y = other.y;
    return *this;
}

template <typename T>
Vec2<T>& Vec2<T>::operator=(Vec2<T>&& other) {
    x = std::move(other.x);
    y = std::move(other.y);
    return *this;
}

template <Arithmetic T>
Vec2<T> Vec2<T>::operator+(const Vec2<T>& other) {
    return Vec2(x + other.x, y + other.y);
}

template <Arithmetic T>
Vec2<T> Vec2<T>::operator-(const Vec2<T>& other) {
    return Vec2(x - other.x, y - other.y);
}

template <Arithmetic T>
Vec2<T> Vec2<T>::operator*(const Vec2<T>& other) {
    return Vec2(x * other.x, y * other.y);
}

template <Arithmetic T>
Vec2<T> Vec2<T>::operator/(const Vec2<T>& other) {
    return Vec2(x / other.x, y / other.y);
};
