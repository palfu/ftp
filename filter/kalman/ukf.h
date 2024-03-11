#pragma once

#include "filter.h"
#include "sigma_points.h"

namespace ftp
{

class UKF : public KalmanFilter
{
public:
    DEFINE_SMART_PTR(UKF);

    void calcSigmaPoints(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covariance)
    {
        sigma_points_ = ptr_sigma_points_->sigmaPoints(mean, covariance);
        ptr_sigma_points_->computeWeights(mean_weights_, covariance_weights_);
    }

    SigmaPointsPtr ptr_sigma_points_ = nullptr;
    std::vector<StateVar> sigma_points_;
    std::vector<MeasureVar> sigma_measures_;
    std::vector<double> mean_weights_;
    std::vector<double> covariance_weights_;
};

}  // namespace ftp