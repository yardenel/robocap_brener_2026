#pragma once
#include <type_traits>

template <typename T>
concept Arithmetic = std::is_integral_v<T>;
