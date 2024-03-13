#pragma once

#include "common.h"
namespace ftp
{

template <typename MAT>
Json::Value toJson(Mat const& mat)
{
    Json::Value js_mat;
    int rows       = mat.rows();
    int cols       = mat.cols();
    js_mat["rows"] = rows;
    js_mat["cols"] = cols;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            js_mat["data"].append(mat(r, c));
        }
    }
    return js_mat;
}  // namespace Json::ValuetoJson(Matconst&mat)

template <typename T>
bool fromJson(Json::Value const& js, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& mat)
{
    if (js["rows"].empty() || js["cols"].empty() || js["data"].empty()) {
        FUSION_WARN("js has no rows、cols、data in!");
        return false;
    }

    int rows            = js["rows"].asInt();
    int cols            = js["cols"].asInt();
    Json::Value js_data = js["data"];
    int data_size       = js_data.size();
    if (data_size != rows * cols) {
        FUSION_ERROR("data size not equal rows x cols, %d, %d %d", data_size, rows, cols);
        return false;
    }

    mat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(rows, cols, T(0));
    std::vector<T> vec_data(data_size, 0);
    int r = 0;
    int c = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx   = r * cols + c;
            mat(r, c) = T(js_data[idx].asDouble());
        }
    }
    return true;
}

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

template <typename, T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> inverseDiagonal(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& mat)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> inv = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(mat.rows(), mat.cols());
    if (mat.rows() != mat.cols() || mat.rows() == 0) {
        FLOG_INFO("input mat rows %d, cols %d, invalid!~", mat.rows(), mat.cols());
    } else {
        for (int i = 0; i < mat.rows(); i++) {
            if (fabs(mat(i, i)) < 1e-6) {
                inv(i, i) = T(1.0 / 1e6);
            } else {
                inv(i, i) = T(1.0 / mat(i, i));
            }
        }
    }
    return inv;
}

double chi2Test(Eigen::VectorXd const& residual, Eigen::MatrixXd const& covariance, bool use_diagonal_inverse = true)
{
    Eigen::MatrixXd inv_cov;
    if (use_diagonal_inverse) {
        inv_cov = inverseDiagonal(covariance);
    } else {
        inv_cov = covariance.llt().solve(Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
    }
    double chi_value = residual.transpose() * inv_cov * residual;
    return chi_value;
}

// test input residual<<0.3, 0.5; covariance<<0.4, 0,2 0,2, 0.4
// output 0.334736

double logpdf(Eigen::VectorXd const& residual, Eigen::MatrixXd const& covariance)
{
    const double log2pi = log(2 * M_PI);
    int rank            = residual.size();
    double log_det      = log(covariance.determinant());

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    // Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd S          = svd.singularValues();
    Eigen::VectorXd inv_sqrt_S = Eigen::VectorXd::Constant(S.size(), 0);
    for (int i = 0; i < S.size(); i++) {
        if (S(i) > 1e-6) {
            inv_sqrt_S(i) = sqrt(1 / S(i));
        }
    }
    Eigen::VectorXd proj_residual = (residual * U).cwiseProduct(inv_sqrt_S);
    double noise                  = proj_residual.norm() * proj_residual.norm();

    return -0.5 * (rank * log2pi + log_det + noise);
}

}  // namespace ftp