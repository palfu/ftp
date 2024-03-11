#pragma once

#include "common/common.h"

namespace ftp
{

using SQRT_FUNCTION     = std::function<Eigen::VectorXd(Eigen::MatrixXd const&)>;
using SUBTRACT_FUNCTION = std::function<Eigen::VectorXd(Eigen::VectorXd const&, double const&, size_t const&)>;

class SigmaPoints
{
public:
    SigmaPoints(
        size_t state_size, double alpha = 0.1, double beta = 2.0, double kappa = -1.0, SQRT_FUNCTION sqrt_fun = nullptr, SUBTRACT_FUNCTION sub_fun = nullptr)
        : state_size_(state_size), alpha_(alpha), beta_(beta), kappa_(kappa), sqrt_fun_(sqrt_fun), sub_fun_(sub_fun)
    {
        if (sub_fun_ == nullptr) {
            sub_fun_ = [](Eigen::VectorXd const& mean, double const& diff, size_t const& index) -> Eigen::VectorXd {
                Eigen::VectorXd res = mean;
                res(index) -= diff;
                return res;
            }
        }

        if (sqrt_fun_ == nullptr) {
            sqrt_fun_ = [](Eigen::MatrixXd const& covariance) -> Eigen::VectorXd {
                int rows            = covariance.rows();
                int cols            = covariance.cols();
                Eigen::VectorXd std = Eigen::VectorXd::Constant(rows, 0.0);
                if (rows != cols) {
                    FLOG_ERROR("input covariance rows not equal cols, %d, %d", rows, cols);
                    return std;
                }
                for (int i = 0; i < rows; i++) {
                    if (covariance(i, i) > 0.0) {
                        std(i) = sqrt(covariance(i, i));
                    }
                }
                return std;
            }
        }
    }

    DEFINE_SMART_PTR(SigmaPoints);

    inline size_t getSigmaSize() const { return 2 * state_size_ + 1; }

    std::vector<Eigen::VectorXd> sigmaPoints(Eigen::VectorXd const& means, Eigen::MatrixXd const& covariance)
    {
        int state_size = means.size();
        int rows       = covariance.rows();
        int cols       = covariance.cols();
        std::vector<Eigen::VectorXd> res;
        if (state_size != rows || state_size != cols) {
            return res;
        }

        double lambda       = alpha_ * alpha_ * (state_size_ + kappa_) - state_size_;
        Eigen::MatrixXd U_M = (lambda + state_size_) * covariance;
        Eigen::VectorXd std = sqrt_fun_(U_M);
        res.reserve(2 * state_size_ + 1);
        res.emplace_back(means);
        for (int i = 0; i < state_size_; i++) {
            res.emplace_back(sub_fun_(means, std(i), i));
            res.emplace_back(sub_fun_(means, -std(i), i));
        }
        return res;
    }

    void computeWeights(std::vector<double>& mean_weights, std::vector<double>& covariance_weights)
    {
        double lambda                 = alpha_ * alpha_ * (state_size_ + kappa_) - state_size_;
        double c                      = 0.5 / (state_size_ + lambda);
        mean_weights                  = std::vector<double>(2 * state_size_ + 1, c);
        covariance_weights            = std::vector<double>(2 * state_size_ + 1, c);
        mean_weights[0]               = lambda / (state_size_ + lambda) + (1 - alpha_ * alpha_ + beta_);
        covariance_weights[0]         = lambda / (state_size_ + lambda);
        double sum_mean_weights       = c * 2 * state_size_ + mean_weights[0];
        double sum_covariance_weights = c * 2 * state_size_ + covariance_weights[0];
        for (auto& weight : mean_weights) {
            weight /= sum_mean_weights;
        }

        for (auto& weight : covariance_weights) {
            weight /= covariance_weights;
        }
    }

private:
    size_t state_size_         = 1;
    double aplha_              = 1.0;
    double beta_               = 2.0;
    double kappa_              = -1.0;
    SQRT_FUNCTION sqrt_fun_    = nullptr;
    SUBTRACT_FUNCTION sub_fun_ = nullptr;
};

using SigmaPointsPtr = SigmaPoints::Ptr;

}  // namespace ftp