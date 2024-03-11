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

    virtual void predict(double const& dt)
    {
        trans_matrix_             = updateTransitionMatrix(dt);
        state_predict_            = trans_matrix_ * state_;
        state_predict_covariance_ = fading_coeff_ * trans_matrix_ * state_covariance_ * trans_matrix_.transpose() + trans_uncertainty_;
    }

    virtual inline MeasureVar residual(MeasureVar const& meas, StateVar const& state) const { return meas - meas_matrix_ * state; }

    virtual inline MeasureVar measure(StateVar const& state) const { return meas_matrix_ * state; }

    virtual void update(bool use_diag_inverse = true)
    {  // P = (I - KH) P(I - KH) ' + KRK'
        // This is more numerically stable
        // and works for non - optimal K vs the equation
        // P = (I - KH) P usually seen in the literature.
        auto ph_t           = state_predict_covariance_ * meas_matrix_.transpose();
        system_uncertainty_ = meas_matrix_ * ph_t + measure_uncertainty_;
        if (use_diag_inverse) {
            kalman_coeff_ = ph_t * inverseDiagonal(system_uncertainty_);
        } else {
            kalman_coeff_ = ph_t * system_uncertainty_.inverse();
        }

        MeasureVar residual = meas_ - meas_matrix_ * state_predict_;
        state_              = state_predict_ + kalman_coeff_ * residual;
        auto i_kh           = meas_identity_ - kalman_coeff_ * meas_matrix_;
        state_covariance_   = i_kh * state_predict_ * i_kh.transpose() + kalman_coeff_ * measure_uncertainty_ * kalman_coeff_;
    }

    double fading_coeff_ = 1.0;
    StateCovariance trans_uncertainty_;
    KalmanCoeff kalman_coeff_;
    TransitionMatrix trans_matrix_;
    MeasureMatrix meas_matrix_;
    MeasureCovariance meas_identity_ = TransitionMatrix::Identity();
    MeasureCovariance meas_uncertainty_;
    MeasureCovariance system_uncertainty_;
};

}  // namespace ftp