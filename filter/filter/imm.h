#pragma once

#include "filter.h"
#include "common/common_function.h"

namespace ftp
{

class ImmFilter : public Filter
{
public:
    ImmFilter(int state_size = 1) : Filter(state_size, state_size){};

    virtual ~ImmFilter() {}

    virtual void init(std::vector<FilterPtr> const& vec_filter, Eigen::VectorXd const& model_prob, Eigen::MatrixXd const& model_trans_prob)
    {
        vec_filter_       = vec_filter;
        model_prob_       = model_prob;
        model_trans_prob_ = model_trans_prob;
        filter_size_      = vec_filter_.size();
        computeMixingProb();
        computeStateEstimate(false);
    }

    void computeMixingProb()
    {
        mix_model_prob_ = model_trans_prob_.transpose() * model_prob_;
        mix_trans_prob_ = Eigen::MatrixXd::Constant(model_trans_prob_.rows(), model_trans_prob_.cols(), 0);
        for (int r = 0; r < model_trans_prob_.rows(); r++) {
            for (int c = 0; c < model_trans_prob_.cols(); c++) {
                mix_trans_prob_(r, c) = model_trans_prob_(r, c) * model_prob_(r) / mix_model_prob_(c);
            }
        }
    }

    void computeStateEstimate(bool by_predict = false)
    {
        Eigen::VectorXd* ptr_state            = &state_;
        Eigen::MatrixXd* ptr_state_covariance = &state_covariance_;
        if (by_predict) {
            ptr_state            = &state_predict_;
            ptr_state_covariance = &state_predict_covariance_;
        }

        *ptr_state = Eigen::VectorXd::Constant(state_size_, 0);
        std::vector<Eigen::VectorXd> vec_state(filter_size_);
        for (int i = 0; i < filter_size; i++) {
            vec_state[i] = vec_filter_[i]->getAlignedState(by_predict);
            if (by_predict) {
                *ptr_state = *ptr_state + vec_state[i] * model_prob_(i);
            } else {
                *ptr_state = *ptr_state + vec_state[i] * model_prob_(i);
            }
        }

        *ptr_state_covariance = Eigen::MatrixXd::Constant(state_size_, state_size_, 0);
        for (int i = 0; i < filter_size; i++) {
            auto residual = vec_state[i] - *ptr_state;
            *ptr_state_covariance =
                *ptr_state_covariance + model_prob_(i) * (residual * residual.transpose() + vec_filter_[i]->getAlignedStateCovariance(by_predict));
        }
    }

    virtual void predict(double const& dt, bool state_ready = false)
    {
        std::vector<StateVar> mix_state(filter_size_);
        std::vector<StateCovariance> mix_state_covariance(filter_size_);
        std::vector<StateVar> aligned_state(filter_size_);
        std::vector<StateVar> aligned_state_covariance(filter_size_);

        for (int i = 0; i < filter_size_; i++) {
            aligned_state[i]            = vec_filter_[i]->getAlignedState(true);
            aligned_state_covariance[i] = vec_filter_[i]->getAlignedStateCovariance(true);
        }

        for (int i = 0; i < filter_size_; i++) {
            mix_state[i] = StateVar::Constant(state_size_, 0);
            for (int j = 0; j < filter_size_; j++) {
                mix_state[i] = mix_state[i] + aligned_state[j] * mix_trans_prob_(j, i);
            }

            mix_state_covariance[i] = StateCovariance::Constant(state_size_, state_size_, 0);
            for (int j = 0; j < filter_size_; j++) {
                StateVar residual       = aligned_state[j] - mix_state[i];
                mix_state_covariance[i] = mix_state_covariance[i] + mix_trans_prob_(j, i) * (residual * residual.transpose() + aligned_state_covariance[j]);
            }
        }

        for (int i = 0; i < filter_size_; i++) {
            vec_filter_[i]->setState(mix_state[i], mix_state_covariance[i]);
            vec_filter_[i]->update(dt, state_ready);
        }

        computeStateEstimate(true);
    }

    virtual void update(bool use_diag_inverse = true, bool state_ready = false)
    {
        Eigen::VectorXd filter_likelihood = Eigen::VectorXd::Constant(filter_size_, 0);
        for (int i = 0; i < filter_size_; i++) {
            auto& filter = vec_filter_[i];
            filter->update(use_diag_inverse, state_ready);
            filter_likelihood(i) = exp(logpdf(filter->state_, filter->state_covariance_));
        }
        model_prob_ = mix_model_prob_.cwiseProduct(filter_likelihood);
        model_prob_ = model_prob_ / model_prob_.norm();

        computeMixingProb();
        computeStateEstimate(false);
    }

private:
    int filter_size_ = 0;
    std::vector<FilterPtr> vec_filter_;
    Eigen::VectorXd model_prob_;
    Eigen::VectorXd mix_model_prob_;
    Eigen::MatrixXd model_trans_prob_;  //model_trans_prob_(i, j), model from i to j
    Eigen::MatrixXd mix_trans_prob_;    //mix_trans_prob_(i, j), state from i to j
}

}  // namespace ftp