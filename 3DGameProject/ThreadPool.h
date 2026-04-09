#pragma once

#include <cstddef>
#include <functional>
#include <future>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <queue>
#include <algorithm>

class ThreadPool {
public:
    static ThreadPool& Instance() noexcept {
        static ThreadPool inst;
        return inst;
    }

    size_t WorkerCount() const noexcept { return _workers.size(); }

    template<typename F>
    auto Enqueue(F&& f) -> std::future<decltype(f())> {
        using ReturnType = decltype(f());
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(std::forward<F>(f));
        std::future<ReturnType> result = task->get_future();
        {
            std::lock_guard<std::mutex> lk(_mtx);
            if (_stop) return result;
            _tasks.emplace([task]() { (*task)(); });
        }
        _cv.notify_one();
        return result;
    }

    template<typename Func>
    void ParallelFor(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        if (begin >= end) return;
        const size_t count = end - begin;
        const size_t workerCount = _workers.size();
        if (workerCount == 0 || count <= grainSize) {
            for (size_t i = begin; i < end; ++i) {
                func(i);
            }
            return;
        }
        const size_t desiredChunks = workerCount * 2;
        const size_t chunkSize = (std::max)(grainSize, (count + desiredChunks - 1) / desiredChunks);
        const size_t numChunks = (count + chunkSize - 1) / chunkSize;
        std::vector<std::future<void>> futures;
        futures.reserve(numChunks);
        for (size_t chunk = 0; chunk < numChunks; ++chunk) {
            const size_t chunkBegin = begin + chunk * chunkSize;
            const size_t chunkEnd = (std::min)(chunkBegin + chunkSize, end);
            if (chunk == numChunks - 1) {
                for (size_t i = chunkBegin; i < chunkEnd; ++i) {
                    func(i);
                }
            }
            else {
                futures.push_back(Enqueue([chunkBegin, chunkEnd, &func]() {
                    for (size_t i = chunkBegin; i < chunkEnd; ++i) {
                        func(i);
                    }
                }));
            }
        }
        for (auto& f : futures) {
            f.get();
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(_mtx);
            _stop = true;
        }
        _cv.notify_all();
        for (auto& w : _workers) {
            if (w.joinable()) w.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

private:
    ThreadPool() {
        const unsigned int hwThreads = std::thread::hardware_concurrency();
        const unsigned int numWorkers = (hwThreads > 1) ? (hwThreads - 1) : 1;
        _workers.reserve(numWorkers);
        for (unsigned int i = 0; i < numWorkers; ++i) {
            _workers.emplace_back([this]() { WorkerLoop_(); });
        }
    }

    void WorkerLoop_() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lk(_mtx);
                _cv.wait(lk, [this]() { return _stop || !_tasks.empty(); });
                if (_stop && _tasks.empty()) return;
                task = std::move(_tasks.front());
                _tasks.pop();
            }
            task();
        }
    }

    std::vector<std::thread> _workers;
    std::queue<std::function<void()>> _tasks;
    std::mutex _mtx;
    std::condition_variable _cv;
    bool _stop = false;
};
