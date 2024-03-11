#pragma once

#include "common.h"

namespace ftp
{

using StateVar            = Eigen::VectorXd;
using StateCovariance     = Eigen::MatrixXd;
using MeasureVar          = Eigen::VectorXd;
using MeasureCovariance   = Eigen::MatrixXd;
using TransitionMatrix    = Eigen::MatrixXd;
using MeasureMatrix       = Eigen::MatrixXd;
using KalmanCoeff         = Eigen::MatrixXd;
using MeasureFunction     = std::function<MeasureVar(StateVar const&)>;
using TransitionFunction  = std::function<StateVar(StateVar const&, double const&)>;
using StateMeanFunction   = std::function<StateVar(std::vector<StateVar> const&, std::vector<double> const&)>;
using MeasureMeanFunction = std::function<MeasureVar(std::vector<MeasureVar> const&, std::vector<double> const&)>;

class Filter
{
public:
    DEFINE_SMART_PTR(Filter);

    Filter(size_t const& state_size = 1, size_t const& meas_size = 1) : state_size_(state_size), meas_size_(meas_size)
    {
        setStateConstant(state_);
        setStateConstant(state_predict_);
        setMatrixDiagonal(state_covariance_, state_size_, state_size_);
        setMatrixDiagonal(state_predict_covariance_, state_size_, state_size_);
        setMeasureConstant(meas_);
    }

    virtual ~Filter() {}

    inline size_t getStateSize() const { return state_size_; }

    inline size_t getMeasureSize() const { return meas_size_; }

    inline void setStateConstant(Eigen::VectorXd& vd, double const& val = 0.0) const { vd = Eigen::VectorXd::Constant(state_size_, val); }
    inline void setMeasureConstant(Eigen::VectorXd& vd, double const& val = 0.0) const { vd = Eigen::VectorXd::Constant(meas_size_, val); }
    inline void setMatrixDiagonal(Eigen::MatrixXd& md, size_t const& row, size_t const& col = 1, double const& val = 1.0) const
    {
        md = Eigen::MatrixXd::Constant(row, col, val);
    }

    virtual void updateTransitionMatrix(double const& dt) {}

    virtual void predict(double const& dt) { state_predict_ = state_, state_predict_covariance_ = state_covariance_; }

    inline void setMeasure(MeasureVar const& meas)
    {
        meas_        = meas;
        meas_update_ = true;
    }

    virtual Eigen::VectorXd getAlignedState() const { return state_; }

    virtual MeasureVar measure(StateVar const& state) { return meas_; }

    virtual void update()
    {
        state_            = state_predict_;
        state_covariance_ = state_predict_covariance_;
        meas_update_      = false;
    };

    bool meas_update_ = false;
    StateVar state_, state_predict_;
    MeasureVar meas_;
    StateCovariance state_covariance_, state_predict_covariance_;
    double like_hood_;
};

using FilterPtr = Filter::Ptr;

}  // namespace ftp
