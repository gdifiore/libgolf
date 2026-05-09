#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <array>
#include <vector>

using Vector3D = std::array<float, 3>;

// Free-function arithmetic on Vector3D. Defined inline for header-only use
// inside default models. Operate elementwise; scalar multiply commutes.
constexpr Vector3D operator+(const Vector3D& a, const Vector3D& b) noexcept
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
constexpr Vector3D operator-(const Vector3D& a, const Vector3D& b) noexcept
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
constexpr Vector3D operator-(const Vector3D& v) noexcept
{
    return {-v[0], -v[1], -v[2]};
}
constexpr Vector3D operator*(const Vector3D& v, float s) noexcept
{
    return {v[0] * s, v[1] * s, v[2] * s};
}
constexpr Vector3D operator*(float s, const Vector3D& v) noexcept
{
    return v * s;
}
constexpr Vector3D& operator+=(Vector3D& a, const Vector3D& b) noexcept
{
    a[0] += b[0]; a[1] += b[1]; a[2] += b[2];
    return a;
}
constexpr Vector3D& operator-=(Vector3D& a, const Vector3D& b) noexcept
{
    a[0] -= b[0]; a[1] -= b[1]; a[2] -= b[2];
    return a;
}
constexpr Vector3D& operator*=(Vector3D& v, float s) noexcept
{
    v[0] *= s; v[1] *= s; v[2] *= s;
    return v;
}

namespace math_utils
{
    auto convertFahrenheitToCelsius(float fahrenheit) -> float;
    auto convertCelsiusToKelvin(float celsius) -> float;
    auto convertFahrenheitToKelvin(float fahrenheit) -> float;

    auto convertFeetToMeters(float feet) -> float;
    auto convertMetersToFeet(float meters) -> float;

    auto getDistanceInYards(Vector3D position) -> float;

    // Vector math operations
    auto dot(const Vector3D& a, const Vector3D& b) noexcept -> float;
    auto cross(const Vector3D& a, const Vector3D& b) noexcept -> Vector3D;
    auto magnitude(const Vector3D& v) noexcept -> float;
    auto normalize(const Vector3D& v) -> Vector3D;
    auto project(const Vector3D& v, const Vector3D& onto) -> Vector3D;
}

#endif // MATH_UTILS_HPP