#pragma once

#include "common.h"

namespace ftp
{
using StateVar            = Eigen::Matrix<UNIT, Dynamic, 1>;
using MeasureVar          = Eigen::Matrix<UNIT, Dynamic, 1>;
using StateCovariance     = Eigen::Matrix<UNIT, Dynamic, Dynamic>;
using MeasureCovariance   = Eigen::Matrix<UNIT, Dynamic, Dynamic>;
using TransitionMatrix    = Eigen::Matrix<UNIT, Dynamic, Dynamic>;
using MeasureMatrix       = Eigen::Matrix<UNIT, Dynamic, Dynamic>;
using KalmanCoeff         = Eigen::Matrix<UNIT, Dynamic, Dynamic>;
using MeasureFunction     = std::function<MeasureVar(StateVar const&, void* ptr = nullptr)>;
using TransitionFunction  = std::function<StateVar(StateVar const&, double const&, void* ptr = nullptr)>;
using StateMeanFunction   = std::function<StateVar(std::vector<StateVar> const&, std::vector<double> const&)>;
using MeasureMeanFunction = std::function<MeasureVar(std::vector<MeasureVar> const&, std::vector<double> const&)>;

class Filter
{
public:
    DEFINE_SMART_PTR(Filter);

    Filter(size_t const& state_size = 1, size_t const& meas_size = 1) : state_size_(state_size), meas_size_(meas_size)
    {
        // 按照状态和观测进行维度的初始化
        setStateConstant(state_);
        setStateConstant(state_predict_);
        setMatrixDiagonal(state_covariance_, state_size_, state_size_);
        setMatrixDiagonal(state_predict_covariance_, state_size_, state_size_);
        setMeasureConstant(meas_);
    }

    virtual ~Filter() {}

    // 获取状态对应的维度
    inline size_t getStateSize() const { return state_size_; }

    // 获取测量对应的维度
    inline size_t getMeasureSize() const { return meas_size_; }

    // 设置常量状态量
    inline void setStateConstant(&vd, double const& val = 0.0) const { vd = Eigen::VectorXd::Constant(state_size_, val); }

    // 设置常量观测
    inline void setMeasureConstant(Eigen::VectorXd& vd, double const& val = 0.0) const { vd = Eigen::VectorXd::Constant(meas_size_, val); }

    // 设置对角矩阵
    inline void setMatrixDiagonal(Eigen::MatrixXd& md, size_t const& row, size_t const& col = 1, double const& val = 1.0) const
    {
        md = Eigen::MatrixXd::Constant(row, col, val);
    }

    // 更新状态转移方程
    virtual void updateTransitionMatrix(double const& dt) {}

    // 更新测量方程
    virtual void setMeasureMatrix(Eigen::MatrixXd const& meas_matrix) { meas_matrix_ = meas_matrix; }

    // 进行估计
    virtual void predict(double const& dt, bool state_ready = false)
    {
        if (trans_func_ != nullptr) {
            trans_func_(state_, dt);
        } else {
            state_predict_            = state_;
            state_predict_covariance_ = state_covariance_;
        }
    }

    // 设置观测
    inline void setMeasure(MeasureVar const& meas)
    {
        meas_        = meas;
        meas_update_ = true;
    }

    // 将状态量进行对齐，统一不同维度模型之间的估计，返回融合状态量的对齐，或者估计状态量的对齐
    virtual Eigen::VectorXd getAlignedState(bool by_predict = false) const { return state_; }

    // 将状态量协方差进行对齐，统一不同维度模型之间的估计，返回融合状态量的对齐，或者估计状态量的对齐
    virtual Eigen::MatrixXd getAlignedStateCovariance(bool by_predict = false) const { return state_covariance_; }

    // 由状态获取观测
    virtual MeasureVar measure(StateVar const& state)
    {
        if (meas_func_ != nullptr) {
            return meas_func_(state);
        } else {
            return meas_;
        }
    }

    // 设置状态转移函数
    inline void setTransitionFunction(TransitionFunction const& func) { trans_func_ = func; }

    // 设置观测函数
    inline void setMeasureFunction(MeasureFunction const& func) { meas_func_ = func; }

    // 更新函数
    virtual void update(bool use_diag_inverse = true, bool state_ready = false)
    {
        state_            = state_predict_;
        state_covariance_ = state_predict_covariance_;
        meas_update_      = false;
    };

    virtual void setState(StateVar const& state, StateCovariance const& covariance, bool by_predict = false)
    {
        if (by_predict) {
            state_predict_            = state;
            state_predict_covariance_ = covariance;
        } else {
            state_            = state;
            state_covariance_ = covariance;
        }
    }

    bool meas_update_ = false;
    StateVar state_;
    StateVar state_predict_;
    MeasureVar meas_;
    StateCovariance state_covariance_;
    StateCovariance state_predict_covariance_;
    TransitionMatrix trans_matrix_;
    MeasureMatrix meas_matrix_;
    TransitionFunction trans_func_ = nullptr;
    MeasureFunction meas_func_     = nullptr;
};

using FilterPtr = Filter::Ptr;

}  // namespace ftp
