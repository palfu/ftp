#include <stdio.h>
#include <iostream>
#include <array>
#include <vector>
#include <memory>

template <size_t S = 1, size_t M = 1>
class Filter
{
public:
    Filter() {}
    virtual ~Filter(){};
    virtual void print() { printf("basic filter %lu %lu\n", state_.size(), meas_.size()); }

    std::array<double, S> state_;
    std::array<double, M> meas_;
};

template <size_t S, size_t M>
class GHFilter : public Filter<S, M>
{
public:
    GHFilter() : Filter<S, M>() {}
    virtual ~GHFilter() {}
    virtual void print() { printf("gh filter %lu %lu\n", state_.size(), meas_.size()); }

    std::array<double, S> state_;
    std::array<double, M> meas_;
};

template <size_t S, size_t M>
class KalmanFilter : public Filter<S, M>
{
public:
    KalmanFilter() : Filter<S, M>() {}
    virtual ~KalmanFilter(){};
    virtual void print() { printf("kalman filter %lu %lu\n", state_.size(), meas_.size()); }

    std::array<double, S> state_;
    std::array<double, M> meas_;
};

class Base
{
public:
    virtual ~Base() {}
    virtual void test_print() { printf("base virtual!\n"); }
};

class Derived : public Base
{
public:
    virtual ~Derived() {}
    virtual void test_print() { printf("Derived virtual!\n"); }
};

int main(int argc, char** argv)
{
    std::vector<std::shared_ptr<Filter<3, 4>>> vec_filter = {std::make_shared<Filter<3, 4>>(), std::make_shared<GHFilter<3, 4>>(),
                                                             std::make_shared<KalmanFilter<3, 4>>()};

    for (auto& filter : vec_filter) {
        // filter->virtualFunction();
        const std::type_info& typeInfo = typeid(*filter);
        std::cout << "Type information: " << typeInfo.name() << std::endl;
        filter->print();
    }

    Base* basePtr = new Derived();
    basePtr->test_print();
    delete basePtr;

    return 0;
}