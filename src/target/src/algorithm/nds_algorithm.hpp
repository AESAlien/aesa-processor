#pragma once

#include <math/linear_algebra/square_matrix.hpp>
#include <math/linear_algebra/vector.hpp>

#include <cstddef>
#include <optional>

namespace target
{

// Computes r^T S^-1 r for Cartesian residuals in metres and covariance in m^2.
// Owns a factorization for reuse; association thresholds belong to the caller.
class NdsAlgorithm
{
public:
    static constexpr std::size_t DIMENSION = 3;

    // Rejects unusable covariance with nullopt; retains an independent snapshot.
    static std::optional<NdsAlgorithm> from(const math::SquareMatrix& covariance);
    static std::optional<double> calculate(
        const math::Vector& residual,
        const math::SquareMatrix& covariance
    );
    std::optional<double> calculate(const math::Vector& residual) const;

private:
    explicit NdsAlgorithm(math::SquareMatrix lowerCovariance);

    math::SquareMatrix _lowerCovariance;
};

} // namespace target
