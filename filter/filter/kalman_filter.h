#pragma once

#include "common/filter.h"
#include "common/common_function.h"

namespace ftp
{

class KalmanFilter : public Filter
{
public:
    KalmanFilter(size_t const& state_size = 1, size_t const& meas_size = 1, double const& fading_coeff = 1.0)
        : Filter(state_size, meas_size), fading_coeff_(fading_coeff)
    {
        setMatrixDiagonal(trans_matrix_, state_size_, state_size_, 1.0);
        setMatrixDiagonal(trans_uncertainty_, state_size_, state_size_, 0.0);
        setMatrixDiagonal(meas_matrix_, meas_size_, state_size_, 1.0);
        setMatrixDiagonal(meas_uncertainty_, meas_size_, meas_size_, 0.0);
        setMatrixDiagonal(kalman_coeff_, state_size_, meas_size_, 1.0);
        setMatrixDiagonal(identity_, meas_size_, meas_size_, 1.0);
        setMatrixDiagonal(system_uncertainty_, meas_size_, meas_size_, 0.0);
    }

    virtual ~KalmanFilter() {}

    virtual void predict(double const& dt, bool state_ready = false)
    {
        trans_matrix_  = updateTransitionMatrix(dt);
        state_predict_ = trans_matrix_ * state_;
        if (state_ready) {
            state_predict_covariance_ = state_covariance_;

        } else {
            state_predict_covariance_ = fading_coeff_ * trans_matrix_ * state_covariance_ * trans_matrix_.transpose() + trans_uncertainty_;
        }
    }

    virtual inline MeasureVar residual(MeasureVar const& meas, StateVar const& state) const { return meas - meas_matrix_ * state; }

    virtual inline MeasureVar measure(StateVar const& state) const { return meas_matrix_ * state; }

    virtual void update(bool use_diag_inverse = true, bool state_ready = false)
    {
        // TODO
        // P = (I - KH) P(I - KH) ' + KRK'
        // This is more numerically stable
        // and works for non - optimal K vs the equation
        // P = (I - KH) P usually seen in the literature.

        if (!state_ready) {
            auto ph_t           = state_predict_covariance_ * meas_matrix_.transpose();
            system_uncertainty_ = meas_matrix_ * ph_t + measure_uncertainty_;

            if (use_diag_inverse) {
                kalman_coeff_ = ph_t * inverseDiagonal(system_uncertainty_);
            } else {
                kalman_coeff_ = ph_t * system_uncertainty_.inverse();
            }
        }

        MeasureVar residual = meas_ - meas_matrix_ * state_predict_;
        state_              = state_predict_ + kalman_coeff_ * residual;
        auto i_kh           = meas_identity_ - kalman_coeff_ * meas_matrix_;
        state_covariance_   = i_kh * state_predict_ * i_kh.transpose() + kalman_coeff_ * measure_uncertainty_ * kalman_coeff_;
    }

    // 假设状态作为观测，状态转移方程等同于观测方式，状态转移方程为单位阵
    virtual void rtsSmooth(std::vector<Eigen::VectorXd> const& vec_state,
                           std::vector<Eigen::MatrixXd> const& vec_covariance,
                           std::vector<double> const& vec_dt,
                           std::vector<Eigen::VectorXd>& smoothed_state,
                           std::vector<Eigen::MatrixXd>& smoothed_covariance)
    {
        int state_size      = vec_state.size();
        int covariance_size = vec_covariance.size();
        int dt_size         = vec_dt.size();
        if (state_size < 2 || state_size != covariance_size || (dt_size != state_size && dt_size != state_size - 1)) {
            FLOG_ERROR("state_size %s, cov %d, dt %d, size not match!", state_size, covariance_size, dt_size);
            return;
        }

        int dt_idx = 1;
        if (dt_size == state_size - 1) {
            dt_idx = 0;
        }

        Eigen::VectorXd predict_mean_state;
        Eigen::MatrixXd predict_mean_state_covariance;
        Eigen::MatrixXd cross_state_covariance;
        smoothed_state      = vec_state;
        smoothed_covariance = vec_covariance;
        for (int i = state_size - 2; i >= 0; i--) {
            trans_matrix_          = updateTransitionMatrix(dt);
            system_uncertainty_    = trans_matrix_ * vec_covariance[i] * trans_matrix_.transpose() + trans_uncertainty_;
            kalman_coeff_          = vec_covariance[i].trans_matrix_.transpose() * system_uncertainty_.inverse();
            smoothed_state[i]      = vec_state[i] + coeff * (vec_state[i + 1] - trans_matrix_ * vec_state[i]);
            smoothed_covariance[i] = vec_covariance[i] + coeff * (vec_covariance[i + 1] - system_uncertainty_) * coeff.transpose();
        }
    }

    double fading_coeff_ = 1.0;
    StateCovariance trans_uncertainty_;
    KalmanCoeff kalman_coeff_;

    MeasureCovariance meas_identity_ = TransitionMatrix::Identity();
    MeasureCovariance meas_uncertainty_;
    MeasureCovariance system_uncertainty_;
};

}  // namespace ftp