#pragma once
#include <Eigen/Eigen>
#include "log.h"

namespace ftp
{

#ifdef _USE_DOUBLE_
using UNIT = double;
using VD   = Eigen::VectorXd;
using MD   = Eigen::MatrixXd;
#else
using UNIT = float;
using VD   = Eigen::VectorXf;
using MD   = Eigen::MatrixXf;
#endif
using RowMajor = Eigen::RowMajor;

template <typename MAT>
std::string prettyPrint(MAT const& mat, std::string label = "", int format_bit = 3)
{
    std::string format = "%%0.%df";
    if (label != "") {
        label = "matrix";
    }
    std::string basic_str(' ', label.size() + 1);
    int rows = mat.rows();
    int cols = mat.cols();
    char msg[128];
    std::string line;
    for (int r = 0; r < rows; r++) {
        std::string str;
        for (int c = 0; c < cols; c++) {
            snprintf(msg, 128, format.c_str(), mat(r, c));
            if (c != cols - 1) {
                str = str + msg + ", ";
            } else {
                str = str + msg;
            }
        }
        std::string tmp = label;
        if (r != 0) {
            tmp = basic_str;
        }
        if (r != rows - 1) {
            line = line + tmp + str + "\n";
        } else {
            line = line + tmp + str;
        }
    }
    return line;
}

template <typename X, typename Y, typename Jac>
Y rungeKutta4(typename Y const& y, typename X const& x, typename X const& dx, std::function<typename Jac(typename X const&, typename Y const&)> fun)
{
    Y k1  = dx * fun(x, y);
    Y k2  = dx * fun(y + 0.5 * k1, x + 0.5 * dx);
    Y k3  = dx * fun(y + 0.5 * k2, x + 0.5 * dx);
    Y K4  = dx * fun(y + k3, x + dx);
    Y res = y + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    return res;
}

class EigenHelper
{
public:
    static MD inverseDiagonal(MD const& mat)
    {
        MD inv = MD::Zero(mat.rows(), mat.cols());
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
