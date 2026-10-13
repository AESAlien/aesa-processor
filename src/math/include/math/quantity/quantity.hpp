#pragma once

namespace math
{

class Quantity
{
public:
    double value() const noexcept;

    bool isFinite() const noexcept;

    // 저장된 값이 0 이상인지 검사한다. NaN은 false이다.
    bool isNonnegative() const noexcept;

protected:
    Quantity(const Quantity&) noexcept;
    Quantity(Quantity&&) noexcept;
    Quantity& operator=(const Quantity&) noexcept;
    Quantity& operator=(Quantity&&) noexcept;

    explicit Quantity(double value) noexcept;

    ~Quantity() noexcept;

    double _value;
};

}
