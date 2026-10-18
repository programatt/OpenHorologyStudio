// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace ohs {

/// Runs f(0) .. f(n-1) on up to `max_threads` threads (default: the hardware concurrency, capped at
/// 16) and waits. Indices are handed out dynamically, so uneven work balances itself; callers write
/// results into slot i of a pre-sized container, which keeps outputs independent of scheduling.
/// The first exception thrown by f is rethrown here after all threads have stopped.
template <class F>
void parallel_for(std::size_t n, F f, unsigned max_threads = 0) {
    unsigned threads = max_threads ? max_threads : std::min(16u, std::max(1u, std::thread::hardware_concurrency()));
    threads = static_cast<unsigned>(std::min<std::size_t>(threads, n));
    if (threads <= 1) {
        for (std::size_t i = 0; i < n; ++i) f(i);
        return;
    }
    std::atomic<std::size_t> next{0};
    std::exception_ptr failure;
    std::mutex failure_mutex;
    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (unsigned t = 0; t < threads; ++t) {
        pool.emplace_back([&] {
            for (std::size_t i = next.fetch_add(1); i < n; i = next.fetch_add(1)) {
                try {
                    f(i);
                } catch (...) {
                    std::lock_guard<std::mutex> lock(failure_mutex);
                    if (!failure) failure = std::current_exception();
                    next.store(n);
                }
            }
        });
    }
    for (std::thread& th : pool) th.join();
    if (failure) std::rethrow_exception(failure);
}

}  // namespace ohs
