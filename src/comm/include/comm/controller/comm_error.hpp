#pragma once

#include <stdexcept>
#include <string>

namespace comm
{

class CommError : public std::runtime_error
{
public:
    explicit CommError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

class ServerOpenError : public CommError
{
public:
    using CommError::CommError;
};

class ConnectionError : public CommError
{
public:
    using CommError::CommError;
};

class ReceiveError : public CommError
{
public:
    using CommError::CommError;
};

class SendError : public CommError
{
public:
    using CommError::CommError;
};

} // namespace comm
