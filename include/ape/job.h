#pragma once
#include <functional>
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>

namespace ape {

class JobSystem {
public:
    explicit JobSystem(unsigned workers = std::thread::hardware_concurrency());
    ~JobSystem();

    void enqueue(std::function<void()> job);
    void wait_idle();

private:
    void worker_loop();

    std::vector<std::thread> threads_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> q_;
    std::atomic<bool> quit_{false};
    std::atomic<int> active_{0};
};

}
