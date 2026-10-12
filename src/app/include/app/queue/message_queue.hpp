#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>

namespace app
{
template <typename T>
class MessageQueue
{
public:
    bool push(T message)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_closed)
            {
                return false;
            }
            _queue.push(std::move(message));
        }
        _condition.notify_one();
        return true;
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
    bool _closed = false;
};
} // namespace app
