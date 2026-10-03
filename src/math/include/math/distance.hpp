#pragma once

namespace math
{

class Distance
{
public:
    static Distance fromMeters(double meters) noexcept;
    static Distance fromKilometers(double kilometers) noexcept;

    double m() const noexcept;
    double km() const noexcept;

    // 미터 단위 절대 차이가 허용오차 이하인지 비교한다.
    // 음수·무한대·NaN 허용오차는 false를 반환한다.
    bool equals(const Distance& other, const Distance& absoluteTolerance) const noexcept;

    // 큰 쪽 값의 절댓값에 대한 차이의 비율로 비교한다. 예: 0.01은 1%이다.
    // 음수·무한대·NaN 허용오차와 무한대·NaN 값은 false를 반환한다.
    bool equals(const Distance& other, double relativeTolerance) const noexcept;

    bool operator<(const Distance& other) const noexcept;
    bool operator<=(const Distance& other) const noexcept;
    bool operator>(const Distance& other) const noexcept;
    bool operator>=(const Distance& other) const noexcept;

    Distance operator-() const noexcept;

private:
    explicit Distance(double meters) noexcept;

    double _meters = 0.0;
};

namespace literals
{

Distance operator""_m(unsigned long long meters) noexcept;
Distance operator""_m(long double meters) noexcept;

Distance operator""_km(unsigned long long kilometers) noexcept;
Distance operator""_km(long double kilometers) noexcept;

}

}
