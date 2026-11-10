#pragma once

#include <cstdint>
#include <string>

namespace comm
{

class CommEndpoint
{
public:
    // std::invalid_argument를 던질 수 있다.
    CommEndpoint(std::string address, std::uint16_t port);

    const std::string& getAddress() const noexcept;

    // bind()에 전달하고 수락한 연결의 주소와 비교하는 값이다.
    std::uint32_t getIntAddress() const noexcept;

    std::uint16_t getPort() const noexcept;

    bool operator==(const CommEndpoint& other) const noexcept;

private:
    std::string _address;
    std::uint32_t _intAddress;
    std::uint16_t _port;
};

} // namespace comm
