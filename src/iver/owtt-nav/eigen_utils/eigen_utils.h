#ifndef __EIGEN_UTILS_H__
#define __EIGEN_UTILS_H__

#include <vector>

// forward declare new slice classes
namespace Eigen
{
    template<typename MatrixType, typename RowIndexType, typename ColIndexType> class MatSlice;
    template<typename MatrixType> class MatSliceVect;
    template<typename MatrixType, typename IndexType> class VectSlice;
    template<typename MatrixType> class VectSliceVect;
}

#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define EIGEN_MATRIXBASE_PLUGIN "matrixbase_addons.h"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <matrix_slice.h>

//TODO: add ALL index
namespace Eigen
{
    typedef VectorXi Index;
}

#endif // __EIGEN_UTILS_H__
