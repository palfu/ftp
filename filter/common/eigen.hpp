#pragma once
#include <Eigen/Eigen>
#include "log.h"

namespace ftp
{

class EigenHelper
{
public:
    static Eigen::MatrixXd inverseDiagonal(Eigen::MatrixXd const& mat)
    {
        Eigen::MatrixXd inv = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
        if (mat.rows() != mat.cols() || mat.rows() == 0) {
            FLOG_INFO("input mat rows %d, cols %d, invalid!~", mat.rows(), mat.cols());
        } else {
            for (int i = 0; i < mat.rows(); i++) {
                if (fabs(mat(i, i)) < 1e-6) {
                    inv(i, i) = 1.0 / 1e6;
                } else {
                    inv(i, i) = 1.0 / mat(i, i);
                }
            }
        }
        return inv;
    }
};

}  // namespace ftp
