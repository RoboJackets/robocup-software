#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

namespace rj_utils {
/**
 * @brief A SPSC blocking queue that is unbounded in size, using std::queue
 * as the internal backing data structure.
 * @tparam T The type to be used in the queue.
 */
template <typename T>
class ConcurrentQueue {
public:
    /**
     * @brief Blocking get.
     * @return The first item in the queue, blocking until one is available.
     */
    T Get() {
        std::unique_lock<std::mutex> lock{mutex_};
        cv_.wait(lock, [this]() { return !queue_.empty(); });
        auto item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    bool TryGet(T& item, const std::chrono::milliseconds& timeout) {
        std::unique_lock<std::mutex> lock{mutex_};
        const auto timeout_time = std::chrono::steady_clock::now() + timeout;
        cv_.wait_until(lock, timeout_time,
                       [this]() { return !queue_.empty(); });

        // If it's empty by the time we time out, return false.
        if (queue_.empty()) {
            return false;
        }

        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief Pushes an item onto the queue.
     * @param item The item to be added.
     */
    void Push(const T& item) {
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_.emplace(item);
        }
        cv_.notify_one();
    }

    /**
     * @brief Pushes an item onto the queue.
     * @param item The item to be added.
     */
    void Push(T&& item) {
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_.emplace(std::move(item));
        }
        cv_.notify_one();
    }

private:
    std::queue<T> queue_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
};
}  // namespace rj_utils