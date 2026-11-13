#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>

namespace app
{
template <typename T>
class MessageQueue
{
public:
    void write(T message)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);

            if (_closed)
            {
                throw std::runtime_error("Input is closed");
            }

            _queue.push(std::move(message));
        }

        _condition.notify_one();
    }

    // 닫힌 큐는 남은 메시지와 관계없이 읽기를 거부한다.
    T read()
    {
        std::unique_lock<std::mutex> lock(_mutex);

        _condition.wait(lock, [this]
        {
            return _closed || !_queue.empty();
        });

        if (_closed)
        {
            throw std::runtime_error("Input is closed");
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