#pragma once

namespace ftp
{

namespace resample
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
void randomSample(std::vector<T>& data, size_t const& size, T const& max_val = (T) 1)
{
    data.clear();
    data.reserve(size);
    srandom(time(0));
    for (size_t i = 0; i < size; i++) {
        T val = static_cast<T>(random() % RAND_MAX * max_val);
        data.emplace_back(val);
    }
}

}  // namespace resample

}  // namespace ftp