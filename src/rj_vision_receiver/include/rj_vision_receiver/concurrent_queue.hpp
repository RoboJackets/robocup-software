#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

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
    T get() {
        std::unique_lock<std::mutex> lock{mutex_};
        cv_.wait(lock, [this]() { return !queue_.empty(); });
        auto item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    /**
     * @brief Try getting the first item with a timeout to fail. See Get() for
     * getting an item without a timeout.
     * @param[out] item The item to be returned.
     * @param timeout How long to wait for this operation.
     * @return True if an item was obtained, false if it timed out.
     */
    bool try_get(T& item, const std::chrono::milliseconds& timeout) {
        std::unique_lock<std::mutex> lock{mutex_};
        const auto timeout_time = std::chrono::steady_clock::now() + timeout;
        cv_.wait_until(lock, timeout_time,
                       [this]() { return !queue_.empty(); });

        // If it's empty by the time we time out, return false.
        if (queue_.empty()) {
            return false;
        }

        item = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    /**
     * @brief Returns all of the messages in the queue in a vector.
     * @return A vector containing all the messages in the queue.
     */
    std::vector<T> get_all() {
        std::unique_lock<std::mutex> lock{mutex_};
        std::vector<T> vector(std::make_move_iterator(queue_.begin()),
                              std::make_move_iterator(queue_.end()));
        queue_.clear();

        return vector;
    }

    /**
     * @brief Pushes an item onto the queue.
     * @param item The item to be added.
     */
    void push(const T& item) {
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_.emplace_back(item);
        }
        cv_.notify_one();
    }

    /**
     * @brief Pushes an item onto the queue.
     * @param item The item to be added.
     */
    void push(T&& item) {
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_.emplace_back(std::move(item));
        }
        cv_.notify_one();
    }

private:
    std::deque<T> queue_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
};
}  // namespace rj_utils