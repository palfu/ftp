#pragma once

#include "filter.h"
#include "sigma_points.h"

namespace ftp
{

class UKF : public KalmanFilter
{
public:
    UKF(size_t const& state_size, size_t const& meas_size) : KalmanFilter(state_size, meas_size)
    {
        if (ptr_sigma_points_ == nullptr) {
            ptr_sigma_points_ = std::make_shared<SigmaPoints>(state_size);
        }
    }

    DEFINE_SMART_PTR(UKF);

    virtual ~UKF(){};

    bool unscentedTransform(std::vector<StateVar> const& predict_state,
                            StateVar& mean_state,
                            StateCovariance& mean_covariance,
                            StateCovariance const& noise = StateCovariance::Zero()) const
    {
        int vec_size = predict_size;
        if (vec_size < 1) {
            return false;
        }
        int trans_size  = predict_size[0].rows();
        mean_state      = StateVar::Constant(trans_size, 0);
        mean_covariance = StateVar::Constant(trans_size, trans_size, 0);
        int weight_size = mean_weights_.size();
        if (weight_size != vec_size) {
            FLOG_ERROR("input state size %d, weight size %d, not equal!", vec_size, weight_size);
            return false;
        }

        for (int i = 0; i < weight_size; i++) {
            mean_state = mean_weights_[i] * predict_state[i];
        }
        std::vector<StateVar> vec_dy(weight_size);
        for (int i = 0; i < weight_size; i++) {
            vec_dy          = predict_state[i] - mean_state;
            mean_covariance = mean_covariance + vec_dy * covariance_weights_[i] * vec_dy.transpose();
        }

        if (noise.rows() == trans_size && noise.cols() == trans_size) {
            mean_covariance += noise;
        }

        return true;
    }

    virtual void predict(double const& dt, bool state_ready = false)
    {
        if (!is_init_) return;
        calcSigmaPoints(state_, state_covariance_);
        sigma_predict_.clear();
        sigma_predict_.reserve(sigma_points_.size());
        for (auto& state : sigma_points_) {
            sigma_predict_.emplace_back(trans_func_(state, dt));
        }

        unscentedTransform(sigma_predict_, state_predict_, state_predict_covariance_, trans_uncertainty_);
        if (state_ready) {
            state_predict_covariance_ = state_covariance_;
        }

        // TODO whether update sigma weights
        // calcSigmaPoints(state_predict_, state_predict_covariance_);
    }

    Eigen::MatrixXd crossVariance(Eigen::VectorXd const& mean_state,
                                  Eigen::VectorXd const& mean_measure,
                                  std::vector<Eigen::VectorXd> const& state_predict,
                                  std::vector<Eigen::VectorXd> const& measure_estimate)
    {
        int sz              = mean_state.size();
        int sm              = mean_measure.size();
        Eigen::MatrixXd Pxz = Eigen::MatrixXd::Constant(sz, sm, 0);
        int sigma_size      = state_predict.size();
        if (sigma_size != (int) measure_estimate.size()) {
            return Pxz;
        }

        for (int i = 0; i < sigma_size; i++) {
            Eigen::VectorXd dx = state_predict[i] - mean_state;
            Eigen::VectorXd dz = measure_estimate[i] - mean_measure;
            Pxz                = Pxz + dx * covariance_weights_[i] * dz.transpose();
        }
        return Pxz;
    }

    virtual void update(bool use_diag_inverse = true, bool state_ready = false)
    {
        sigma_measures_.clear();
        sigma_measures_.reserve(sigma_predict_.size());
        for (auto& state : sigma_predict_) {
            sigma_measures_.emplace_back(meas_func_(state));
        }
        MeasureVar mean_measure;
        unscentedTransform(sigma_measures_, mean_measure, system_uncertainty_, meas_uncertainty_);
        if (!state_ready) {
            MeasureCovariance inverse_system_uncertainty = system_uncertainty_.inverse();
            Eigen::MatrixXd cross_variance               = crossVariance(state_predict_, mean_measure, sigma_predict_, sigma_measures_);
            kalman_coeff_                                = Pxz * inverse_system_uncertainty;
        }
        Eigen::VectorXd residual = mean_measure - state_predict_;
        state_                   = state_predict_ + kalman_coeff_ * residual;
        state_covariance_        = state_predict_covariance_ - kalman_coeff_ * system_uncertainty_ * kalman_coeff_.transpose();
    }

    void calcSigmaPoints(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covariance)
    {
        sigma_points_ = ptr_sigma_points_->sigmaPoints(mean, covariance);
        ptr_sigma_points_->computeWeights(mean_weights_, covariance_weights_);
    }

    // 以后验估计作为观测，对前面的估计进行修正
    void rtsSmooth(std::vector<Eigen::VectorXd> const& vec_state,
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
            calcSigmaPoints(vec_state[i], vec_covariance[i]);
            predict(vec_dt[i + dt_idx]);

            unscentedTransform(sigma_predict_, predict_mean_state, predict_mean_state_covariance, trans_uncertainty_);
            cross_state_covariance = crossVariance(predict_mean_state, vec_state[i], sigma_predict_, sigma_points_);
            auto coeff             = cross_state_covariance.predict_mean_state_covariance.inverse();

            smoothed_state[i]      = vec_state[i] + coeff * (vec_state[i + 1] - predict_mean_state);
            smoothed_covariance[i] = vec_covariance[i] + coeff * (vec_covariance[i + 1] - predict_mean_state_covariance) * coeff.transpose();
        }
    }

    bool is_init_                    = false;
    SigmaPointsPtr ptr_sigma_points_ = nullptr;

    std::vector<StateVar> sigma_points_;
    std::vector<StateVar> sigma_predict_;
    std::vector<MeasureVar> sigma_measures_;
    std::vector<double> mean_weights_;
    std::vector<double> covariance_weights_;
};

}  // namespace ftp