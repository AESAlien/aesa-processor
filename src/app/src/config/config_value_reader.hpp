#pragma once

#include <math/linear_algebra/square_matrix.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace app
{

class ConfigValueReader
{
public:
    explicit ConfigValueReader(std::unordered_map<std::string, std::string> configValues);

    const std::string& requireString(const char* key);
    double requireDouble(const char* key);
    math::SquareMatrix requireSquareMatrix(const char* key, std::size_t size);

    template <typename Value>
    Value requireUnsigned(const char* key)
    {
        static_assert(
            std::is_integral_v<Value> && std::is_unsigned_v<Value>,
            "Value must be an unsigned integer type");

        const unsigned long long value = requireUnsignedValue(key);
        if (value > static_cast<unsigned long long>(
                std::numeric_limits<Value>::max()))
        {
            throw std::runtime_error(
                "Unsigned integer value is out of range for " +
                std::string(key));
        }
        return static_cast<Value>(value);
    }

    template <typename Duration>
    Duration requireDuration(const char* key)
    {
        using Rep = typename Duration::rep;
        using Period = typename Duration::period;
        static_assert(
            std::is_same_v<Duration, std::chrono::duration<Rep, Period>>,
            "Duration must be a std::chrono::duration type");
        static_assert(
            std::is_integral_v<Rep>,
            "Duration representation must be an integer type");

        const std::uint64_t value = requireUnsigned<std::uint64_t>(key);
        if (value > static_cast<std::uint64_t>(
                std::numeric_limits<Rep>::max()))
        {
            throw std::runtime_error(
                "Duration value is out of range for " +
                std::string(key));
        }
        return Duration(static_cast<Rep>(value));
    }

    void ensureAllConsumed() const;

private:
    unsigned long long requireUnsignedValue(const char* key);

    std::unordered_map<std::string, std::string> _configValues;
    std::unordered_set<std::string> _consumedKeys;
};

} // namespace app
