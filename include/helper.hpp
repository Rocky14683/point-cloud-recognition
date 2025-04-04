#pragma once
#include <print>
#include <algorithm>
#include <thread>
#include <type_traits>
#include <execution>
#include <future>


namespace helper {
    template <class F, class T, class... Args>
    requires std::invocable<F, Args...>
    std::vector<F> parallel_for(std::function<F(Args...)> callback, size_t start, size_t end, const std::vector<T>& input, Args&&... args) {
        std::vector<F> output_vector(input.size());

        std::vector<std::future<void>> futures;

        for (int i = 0; i < input.size(); i++) {
            futures.emplace_back(std::async(std::launch::async, [&, i]() {
                try {
                    input.at(i) = callback(std::forward<Args>(args)...);
                } catch(const std::exception& e) {
                    std::println("Error in function: {}: frame {}: {}", std::string(__PRETTY_FUNCTION__), i, e.what());
                }
            }));
        }

        for (auto& future : futures) { future.wait(); }

        return output_vector;
    }


};