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

//#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define EIGEN_MATRIXBASE_PLUGIN "matrixbase_addons.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

#include <matrix_slice.h>

//TODO: add ALL index
namespace Eigen
{
    typedef VectorXi Index;
    
    template<typename T>
    T pinv(const T &a, double eps = std::numeric_limits<typename T::Scalar>::epsilon()) {
        
        bool m_lt_n = (a.rows() < a.cols());
        
        JacobiSVD<T> svd;
        if (m_lt_n) {
            T tmp = a.transpose();
            svd = tmp.jacobiSvd(ComputeThinU | ComputeThinV);
        } else { 
            svd = a.jacobiSvd(ComputeThinU | ComputeThinV);
        }
        
        typename T::Scalar tolerance = eps * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

        T result = svd.matrixV() *
                   T( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() *
                   svd.matrixU().adjoint();
        
        
        if (m_lt_n) {
            return result.transpose();
        } else {
            return result;        
        }
        
    }
    
    template<typename T>
    double pdet(const T &a, double eps = std::numeric_limits<typename T::Scalar>::epsilon()) {
        
        double pd = 1.0;
        
        SelfAdjointEigenSolver<T> eig(a);
        if (eig.info() != Success) {
            return 1.0;
        }
        
        double tolerance = eps * std::max(a.cols(), a.rows()) * eig.eigenvalues().array().abs().maxCoeff();
        //typename T::Scalar tolerance = eps * std::max(a.cols(), a.rows());
    
        VectorXd d = eig.eigenvalues();
        for (int i=0; i<d.size(); i++) {
            if (d(i) > tolerance)
                pd *= d(i);
        }
        
        return pd;
    }
    
    // more numerically stable for large mats
    template<typename T>
    double plogdet(const T &a, double eps = std::numeric_limits<typename T::Scalar>::epsilon()) {
        
        double pld = 0.0;

        SelfAdjointEigenSolver<T> eig(a);
        if (eig.info() != Success) {
            return 0.0;
        }
        
        double tolerance = eps * std::max(a.cols(), a.rows()) * eig.eigenvalues().array().abs().maxCoeff();
        //typename T::Scalar tolerance = eps * std::max(a.cols(), a.rows());
    
        VectorXd d = eig.eigenvalues();
        for (int i=0; i<d.size(); i++) {
            if (d(i) > tolerance)
                pld += log (d(i));
        }
        
        return pld;
    }
    
    template<typename T>
    T posdef_pinv(const T &a, double eps = std::numeric_limits<typename T::Scalar>::epsilon()) {
        
        SelfAdjointEigenSolver<T> eig(a);
        if (eig.info() != Success) {
            return T();
        }
        
        typename T::Scalar tolerance = eps * std::max(a.cols(), a.rows()) * eig.eigenvalues().array().abs().maxCoeff();
        //typename T::Scalar tolerance = eps * std::max(a.cols(), a.rows());

        T result = eig.eigenvectors() *
                   T( (eig.eigenvalues().array() > tolerance).select(eig.eigenvalues().array().inverse(), 0) ).asDiagonal() *
                   eig.eigenvectors().transpose();
        
        return result;        
    }
    
    
    
    
    template<typename T>
    T ldlt_inverse (const T &A) {
       return A.ldlt ().solve (T::Identity (A.rows (), A.cols ()));
    }
}

#endif // __EIGEN_UTILS_H__
