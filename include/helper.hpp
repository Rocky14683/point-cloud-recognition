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

    std::vector<float> flatten_mdspan(const std::mdspan<const float, std::extents<size_t, 3, 32, 32, 32>>& m) {
        std::vector<float> out;
        out.reserve(3 * 32 * 32 * 32);

        for (size_t c = 0; c < 3; ++c)
            for (size_t d = 0; d < 32; ++d)
                for (size_t h = 0; h < 32; ++h)
                    for (size_t w = 0; w < 32; ++w)
                        out.push_back(m[c, d, h, w]);

        return out;
    }

};