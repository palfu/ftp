#pragma once

#include <time.h>
#include <vector>
#include <math.h>

namespace ftp
{

template <typename T>
void resampleWithIndex(std::vector<T>& data, std::vector<double>& weights, std::vector<int> const& vec_index)
{
    int index_size = vec_index.size();
    if (index_size <= 0) {
        data.clear();
        weights.clear();
    }

    int pick_size = 0;
    int data_size = data.size();
    for (auto& index : vec_index) {
        if (index >= 0 && index < data_size) {
            data[pick_size++] = data[index];
        }
    }
    data.resize(pick_size);
    weights.clear();
    if (pick_size > 0) {
        double w = 1 / pick_size;
        weights  = std::vector<double>(pick_size, w);
    }
}

template <typename T>
void uniformRandom(std::vector<T>& data, size_t const& size, T const& max_val = (T) 1, T const& shift = T(0))
{
    data.clear();
    data.reserve(size);
    srandom(time(0));
    for (size_t i = 0; i < size; i++) {
        T val = static_cast<T>((random() % RAND_MAX * max_val) + shift);
        data.emplace_back(val);
    }
}

void uniform2(double& U1, double& U2, double const& seed = 131)
{
    double x = seed;
    double a = rand() + x;
    U1       = double(int(a % 1000) / 1000.0);
    x        = x + 1771;
    a        = rand() + x;
    U2       = double(int(a % 1000) / 1000.0);
}

// Box-Muller
std::vector<double> gaussianRandom(double const& mean, double const& std, size_t const& sample_size)
{
    std::vector<double> data(sample_size, 0);
    double U1 = 0;
    double U2 = 0;
    for (int i = 0; i < sample_size; i++) {
        uniform2(U1, U2);
        auto a  = sqrt(-2 * log(U1));
        auto b  = 2 * M_PI * U2;
        auto c  = a * cos(b);
        data[i] = mean + std * c;
    }
    return data;
}

std::vector<int> systematicResample(std::vector<double> const& weights)
{
    std::vector<int> pick_index;
    int input_size = weights.size();
    if (input_size == 0) {
        return;
    }
    systematic_weights.resize(input_size);
    std::vector<double> sum_weights(input_size, 0);
    pick_index = std::vector<int>(input_size, 0);
    for (int i = 0; i < input_size; i++) {
        systematic_weights[i] = i + random() % RAND_MAX;
        sum_weight[i]         = sum_weight[i] + weights[i];
        pick_index[i]         = i;
    }
    int i = 0;
    int j = 0;
    while (i < input_size && j < input_size) {
        if (systematic_weights[i] < sum_weight[j]) {
            pick_index[i] = j;
            i++;
        } else {
            j++;
        }
    }
}

}  // namespace ftp