#pragma once

#include "math/quantity/quantity_tolerance.hpp"

namespace math
{

class Distance : public Quantity
{
public:
    static Distance fromMeters(double meters) noexcept;
    static Distance fromKilometers(double kilometers) noexcept;

    double m() const noexcept;
    double km() const noexcept;

    // 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool equals(const Distance& other, const QuantityTolerance<Distance>& tolerance) const noexcept;

    // DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 값은 false를 반환한다.
    bool operator==(const Distance& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 값은 자기 자신과도 다르다.
    bool operator!=(const Distance& other) const noexcept;

    // DEFAULT 오차 범위 밖에서 더 작으면 true를 반환한다.
    bool operator<(const Distance& other) const noexcept;
    // 더 작거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator<=(const Distance& other) const noexcept;
    // DEFAULT 오차 범위 밖에서 더 크면 true를 반환한다.
    bool operator>(const Distance& other) const noexcept;
    // 더 크거나 DEFAULT 오차 범위 안이면 true를 반환한다.
    bool operator>=(const Distance& other) const noexcept;

    Distance operator-() const noexcept;

private:
    explicit Distance(double meters) noexcept;
};

Distance abs(Distance distance) noexcept;

namespace literals
{

Distance operator""_m(unsigned long long meters) noexcept;
Distance operator""_m(long double meters) noexcept;

Distance operator""_km(unsigned long long kilometers) noexcept;
Distance operator""_km(long double kilometers) noexcept;

}

}
