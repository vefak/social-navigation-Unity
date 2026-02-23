#pragma once
#include <assert.h>

#include <vector>

namespace util {

template <typename T>
inline std::vector<T> arange(T start, T stop, T step) {
    std::vector<T> values;
    for (T value = start; value <= stop; value += step) values.push_back(value);
    return values;
}

template <typename T>
inline std::vector<T> linspace(T start, T step, int n_points) {
    std::vector<T> values;
    for (int i = 0; i < n_points; ++i) values.push_back(start + step * i);
    return values;
}

template <typename T>
inline std::vector<T> arange(T start, T stop, int n_points) {
    return linspace<T>(start, (stop - start) / (n_points - 1), n_points);
}

template <typename T>
inline std::vector<T> normalize(std::vector<T> vec) {
    T min_element = *std::min_element(vec.begin(), vec.end());
    for (auto& element : vec) element -= min_element;
    return vec;
}

}  // namespace util

// Useful operators for vector computations
// We put them outside of the namespace for visibility in other headers and source files
inline std::vector<double> operator+(const std::vector<double>& u, const std::vector<double>& w) {
    assert(u.size() == w.size());
    std::vector<double> v(u.size());

    for (size_t i = 0; i < u.size(); ++i) {
        v[i] = u[i] + w[i];
    }

    return v;
}

inline std::vector<double> operator*(double k, const std::vector<double>& u) {
    std::vector<double> v(u.size());

    for (size_t i = 0; i < u.size(); ++i) {
        v[i] = k * u[i];
    }

    return v;
}
