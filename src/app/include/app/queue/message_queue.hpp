#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace app
{
class ChannelClosedError : public std::runtime_error
{
public:
    ChannelClosedError()
        : std::runtime_error("Cannot write to a closed channel")
    {
    }
};

template <typename T>
class MessageQueue
{
public:
    // 용량을 지정하지 않으면 개수 제한 없이 보관한다.
    MessageQueue() = default;

    // 용량이 찬 상태에서 push하면 가장 오래된 메시지를 버린다.
    // 소비자가 느려도 생산자가 막히거나 메모리가 계속 늘지 않아야 하는 큐에 사용한다.
    explicit MessageQueue(std::size_t capacity)
        : _capacity(capacity)
    {
        if (capacity == 0)
        {
            throw std::invalid_argument("Message queue capacity must be positive");
        }
    }

    void push(T message)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);

            if (_closed)
            {
                throw ChannelClosedError();
            }

            if (_capacity && _queue.size() >= *_capacity)
            {
                _queue.pop();
            }

            _queue.push(std::move(message));
        }

        _condition.notify_one();
    }

    std::optional<T> pop()
    {
        std::unique_lock<std::mutex> lock(_mutex);

        _condition.wait(lock, [this]
        {
            return _closed || !_queue.empty();
        });

        if (_queue.empty())
        {
            return std::nullopt;
        }

        T message = std::move(_queue.front());
        _queue.pop();

        return message;
    }

    void close()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _closed = true;
        }

        _condition.notify_all();
    }

private:
    std::queue<T> _queue;
    std::mutex _mutex;
    std::condition_variable _condition;
    std::optional<std::size_t> _capacity;
    bool _closed = false;
};
} // namespace app