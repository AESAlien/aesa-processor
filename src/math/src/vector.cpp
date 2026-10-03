#include "math/vector.hpp"
#include "math/square_matrix.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace math
{

Vector::Vector()
    : Vector(0)
{
}

Vector::Vector(std::size_t size, double value)
    : _matrix(size, 1, value)
{
}

Vector::Vector(std::initializer_list<double> values)
    : Vector(values.size())
{
    std::size_t index = 0;
    for (double value : values)
    {
        _matrix(index++, 0) = value;
    }
}

Vector::Vector(Matrix matrix)
    : _matrix(std::move(matrix))
{
    if (_matrix.columns() != 1)
    {
        throw std::invalid_argument("Vector requires exactly one column");
    }
}

Vector::Vector(Vector&& other)
    : Vector()
{
    std::swap(_matrix, other._matrix);
}

Vector& Vector::operator=(Vector&& other)
{
    if (this != &other)
    {
        Vector moved(std::move(other));
        std::swap(_matrix, moved._matrix);
    }
    return *this;
}

std::size_t Vector::size() const noexcept
{
    return _matrix.rows();
}

bool Vector::empty() const noexcept
{
    return _matrix.empty();
}

bool Vector::equals(const Vector& other, const Vector& absoluteTolerance) const noexcept
{
    if (size() != other.size() || size() != absoluteTolerance.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < size(); ++index)
    {
        const double a = at(index);
        const double b = other.at(index);
        const double tolerance = absoluteTolerance.at(index);
        if (!std::isfinite(a) || !std::isfinite(b) ||
            !std::isfinite(tolerance) || tolerance < 0.0 || std::abs(a - b) > tolerance)
        {
            return false;
        }
    }
    return true;
}

bool Vector::equals(const Vector& other, double relativeTolerance) const noexcept
{
    return _matrix.equals(other._matrix, relativeTolerance);
}

double& Vector::at(std::size_t index)
{
    return _matrix.at(index, 0);
}

double& Vector::operator()(std::size_t index)
{
    return at(index);
}

double& Vector::operator[](std::size_t index)
{
    return at(index);
}

const double& Vector::at(std::size_t index) const
{
    return _matrix.at(index, 0);
}

const double& Vector::operator()(std::size_t index) const
{
    return at(index);
}

const double& Vector::operator[](std::size_t index) const
{
    return at(index);
}

Matrix Vector::transpose() const
{
    return _matrix.transpose();
}

double Vector::dot(const Vector& other) const
{
    if (size() != other.size())
    {
        throw std::invalid_argument("Vector lengths must match");
    }
    double result = 0.0;
    for (std::size_t index = 0; index < size(); ++index)
    {
        result += at(index) * other.at(index);
    }
    return result;
}

double Vector::norm() const
{
    // 제곱의 합에서 발생할 수 있는 불필요한 오버플로·언더플로를 피한다.
    double result = 0.0;
    for (std::size_t index = 0; index < size(); ++index)
    {
        result = std::hypot(result, at(index));
    }
    return result;
}

Vector Vector::operator+(const Vector& other) const
{
    return Vector(_matrix + other._matrix);
}

Vector Vector::operator-(const Vector& other) const
{
    return Vector(_matrix - other._matrix);
}

Vector Vector::operator*(double scalar) const
{
    return Vector(_matrix * scalar);
}

Vector Vector::operator/(double scalar) const
{
    return Vector(_matrix / scalar);
}

const Matrix& Vector::asMatrix() const noexcept
{
    return _matrix;
}

Vector operator*(double scalar, const Vector& vector)
{
    return vector * scalar;
}

Vector operator*(const Matrix& matrix, const Vector& vector)
{
    return Vector(matrix * vector.asMatrix());
}

Vector operator*(const SquareMatrix& matrix, const Vector& vector)
{
    return matrix.asMatrix() * vector;
}

}
