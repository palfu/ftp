#pragma once

#include "filter.h"

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
        computeStateEstimate(false);
    }

    void computeStateTransProb()
    {
        Eigen::VectorXd new_mode_prob = model_trans_prob_.transpose() * model_prob_;
        state_trans_prob_             = Eigen::MatrixXd::Constant(model_trans_prob_.rows(), model_trans_prob_.cols(), 0);
        for (int r = 0; r < model_trans_prob_.rows(); r++) {
            for (int c = 0; c < model_trans_prob_.cols(); c++) {
                state_trans_prob_(r, c) = model_trans_prob_(r, c) * model_prob_(r) / new_mode_prob(c);
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
            vec_state[i] = vec_filters_[i]->getAlignedState(by_predict);
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
                *ptr_state_covariance + model_prob_(i) * (residual * residual.transpose() + vec_filters_[i]->getAlignedStateCovariance(by_predict));
        }
    }

    virtual void predict(double const& dt, bool state_ready = false)
    {
        std::vector<StateVar> mix_state(filter_size_);
        std::vector<StateCovariance> mix_state_covariance(filter_size_);
        std::vector<StateVar> aligned_state(filter_size_);
        std::vector<StateVar> aligned_state_covariance(filter_size_);

        for (int i = 0; i < filter_size_; i++) {
            aligned_state[i]            = vec_filters_[i]->getAlignedState();
            aligned_state_covariance[i] = vec_filters_[i]->getAlignedStateCovariance();
        }

        for (int i = 0; i < filter_size_; i++) {
            mix_state[i] = StateVar::Constant(state_size_, 0);
            for (int j = 0; j < filter_size_; j++) {
                mix_state[i] = mix_state[i] + aligned_state[j] * state_trans_prob_(j, i);
            }

            mix_state_covariance[i] = StateCovariance::Constant(state_size_, state_size_, 0);
            for (int j = 0; j < filter_size_; j++) {
                StateVar residual       = aligned_state[j] - mix_state[i];
                mix_state_covariance[i] = mix_state_covariance[i] + residual * residual.transpose() + aligned_state_covariance[j];
            }
        }

        for (int i = 0; i < filter_size_; i++) {
            vec_filters_[i]->setState(mix_state[i], mix_state_covariance[i]);
            vec_filters_[i]->update(dt, state_ready);
        }

        computeStateEstimate(true);
    }

    virtual void update(bool use_diag_inverse = true, bool state_ready = false) {}

private:
    int filter_size_ = 0;
    std::vector<FilterPtr> vec_filters_;
    Eigen::VectorXd model_prob_;
    Eigen::MatrixXd model_trans_prob_;  //model_trans_prob_(i, j), model from i to j
    Eigen::MatrixXd state_trans_prob_;  //state_trans_prob_(i, j), state from i to j
}

}  // namespace ftp