#pragma once

#include "math/linear_algebra/matrix.hpp"

namespace math
{

class SquareMatrix;

class Vector
{
public:
    Vector();
    explicit Vector(std::size_t size, double value = 0.0);
    Vector(std::initializer_list<double> values);
    // 열 수가 1이 아니면 std::invalid_argument를 던진다.
    explicit Vector(Matrix matrix);

    Vector(const Vector&);
    Vector& operator=(const Vector&);
    // 이동한 원본은 빈 0×1 열벡터가 된다.
    Vector(Vector&& other);
    Vector& operator=(Vector&& other);

    std::size_t size() const noexcept;
    bool empty() const noexcept;
    // 모든 원소가 유한하면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool isFinite() const noexcept;

    // 차원이 같고 모든 원소가 지정한 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    // 허용오차를 생략하면 DEFAULT 절대·상대오차를 적용한다.
    bool equals(const Vector& other, const Tolerance& tolerance = DEFAULT_TOLERANCE) const noexcept;

    // 차원이 같고 모든 원소가 DEFAULT 절대오차 또는 상대오차 범위 안이면 true를 반환한다. 무한대·NaN 원소가 있으면 false를 반환한다.
    bool operator==(const Vector& other) const noexcept;
    // ==의 결과를 반전한다. 무한대·NaN 원소가 있으면 자기 자신과도 다르다.
    bool operator!=(const Vector& other) const noexcept;

    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    double& at(std::size_t index);
    // 원소 접근 시 범위를 검사한다. 범위 밖이면 std::out_of_range를 던진다.
    const double& at(std::size_t index) const;
    double& operator()(std::size_t index);
    const double& operator()(std::size_t index) const;
    double& operator[](std::size_t index);
    const double& operator[](std::size_t index) const;

    // 전치 결과는 1×N 일반 행렬이다.
    Matrix transpose() const;
    // 두 벡터의 길이가 다르면 std::invalid_argument를 던진다. 빈 벡터의 내적은 0이다.
    double dot(const Vector& other) const;
    double norm() const;

    // 두 벡터의 길이가 다르면 std::invalid_argument를 던진다.
    Vector operator+(const Vector& other) const;
    // 두 벡터의 길이가 다르면 std::invalid_argument를 던진다.
    Vector operator-(const Vector& other) const;
    Vector operator*(double scalar) const;
    // 0으로 나누면 std::domain_error를 던진다.
    Vector operator/(double scalar) const;

    const Matrix& asMatrix() const noexcept;

private:
    Matrix _matrix;
};

Vector operator*(double scalar, const Vector& vector);
// 행렬 열 수와 벡터 길이가 다르면 std::invalid_argument를 던진다.
Vector operator*(const Matrix& matrix, const Vector& vector);
Vector operator*(const SquareMatrix& matrix, const Vector& vector);

}
