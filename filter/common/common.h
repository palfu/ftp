#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <memory.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <string>
#include "log.h"
#include "json/json.h"
#include <function>

namespace ftp
{

#define _USE_DOUBLE_
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
using Dynamic  = Eigen::Dynamic;

#define DEFINE_SMART_PTR(CLASS_NAME)              \
    using Ptr      = std::shared_ptr<CLASS_NAME>; \
    using ConstPtr = std::shared_ptr<const CLASS_NAME>;

}  // namespace ftp
