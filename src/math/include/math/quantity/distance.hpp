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

    // 단위를 보존한 절대·상대오차 정책을 적용한다. 비유한 값은 false이다.
    bool equals(const Distance& other, const QuantityTolerance<Distance>& tolerance) const noexcept;

    // 공통 DEFAULT 절대·상대오차를 함께 적용한다. 무한대·NaN 값은 false이다.
    bool operator==(const Distance& other) const noexcept;
    // ==의 부정이다. 무한대·NaN 값은 자기 자신과도 다르다.
    bool operator!=(const Distance& other) const noexcept;

    // DEFAULT 오차 범위 밖에서 더 작으면 true이다.
    bool operator<(const Distance& other) const noexcept;
    // 더 작거나 DEFAULT 오차 범위 안이면 true이다.
    bool operator<=(const Distance& other) const noexcept;
    // DEFAULT 오차 범위 밖에서 더 크면 true이다.
    bool operator>(const Distance& other) const noexcept;
    // 더 크거나 DEFAULT 오차 범위 안이면 true이다.
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
