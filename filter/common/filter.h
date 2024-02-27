#pragma once

#include "log.h"
#include "eigen_helper.hpp"

namespace ftp
{
template <size_t STATE_SIZE, size_t MEASUREMENT_SIZE>
class Filter
{
public:
    using STATE             = Eigen::Matrix<UNIT, STATE_SIZE, 1, RowMajor>;
    using MEASUREMENT       = Eigen::Matrix<UNIT, MEASUREMENT_SIZE, 1, RowMajor>;
    using TRANSITION_FUNC   = Eigen::Matrix<UNIT, STATE_SIZE, STATE_SIZE, RowMajor>;
    using MEASUREMENT_FUNC  = Eigen::Matrix<UNIT, MEASURE_SIZE, STATE_SIZE, RowMajor>;
    using MEASUREMENT_NOISE = Eigen::Matrix<UNIT, MEASURE_SIZE, MEASURE_SIZE, RowMajor>;
    using TRANSITION_NOISE  = TRANSITION_FUNC;
    using STATE_COVARIANCE  = TRANSITION_NOISE;

    inline const STATE& getState() const { return state_; }

    inline STATE& getState() { return state_; }

    inline void setMeasurementFunc(MEASUREMENT_FUNC const& meas_func) { measure_func_ = meas_func; }

    inline void setSensor(MEASUREMENT const& meas, MEASUREMENT_NOISE const& meas_noise)
    {
        measure_       = meas;
        measure_noise_ = measure_noise_;
    }

    inline void setTransitionNoise(TRANSITION_NOISE const& trans_noise) { transition_noise_ = trans_noise; }

    void predict(UNIT const& dt, TRANSITION_NOISE const& trans_noise)
    {
        setTransitionNoise(trans_noise);
        predict(dt);
    }

    virtual void predict(UNIT const& dt){};

    virtual void update(){};

public:
    STATE state_;
    STATE_COVARIANCE covariance_;
    MEASUREMENT measure_;
    MEASUREMENT_NOISE measure_noise_;
    MEASUREMENT_FUNC measure_func_;
    TRANSITION_NOISE transition_noise_;
};

}  // namespace ftp