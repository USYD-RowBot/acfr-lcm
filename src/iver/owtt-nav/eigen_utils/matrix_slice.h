#ifndef __MATRIX_SLICE_H__
#define __MATRIX_SLICE_H__

#include <algorithm>

namespace Eigen
{
    template<typename MatrixType, typename RowIndexType, typename ColIndexType> class MatSlice;
    template<typename MatrixType> class MatSliceVect;
    template<typename MatrixType, typename IndexType> class VectSlice;
    template<typename MatrixType> class VectSliceVect;
    
    namespace internal
    {
        template<typename MatrixType, typename RowIndexType, typename ColIndexType>
        struct traits<MatSlice<MatrixType, RowIndexType, ColIndexType> > : traits<MatrixType>
        {
            typedef typename nested<MatrixType>::type MatrixTypeNested;
            typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
            typedef typename MatrixType::StorageKind StorageKind;
            enum {
                RowsAtCompileTime = RowIndexType::SizeAtCompileTime,
                ColsAtCompileTime = ColIndexType::SizeAtCompileTime,
                MaxRowsAtCompileTime = RowIndexType::MaxSizeAtCompileTime,
                MaxColsAtCompileTIme = ColIndexType::MaxSizeAtCompileTime,
                Flags = _MatrixTypeNested::Flags & (HereditaryBits | LvalueBit),
                CoeffReadCost = _MatrixTypeNested::CoeffReadCost
            };
        };

        // TODO: sizes should not be dynamic --- this should really only
        // affect performance when matrix size is known at compile time (not
        // very often)
        template<typename MatrixType>
        struct traits<MatSliceVect<MatrixType> > : traits<MatrixType>
        {
            typedef typename nested<MatrixType>::type MatrixTypeNested;
            typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
            typedef typename MatrixType::StorageKind StorageKind;
            enum {
                RowsAtCompileTime = Dynamic,
                ColsAtCompileTime = Dynamic,
                MaxRowsAtCompileTime = Dynamic,
                MaxColsAtCompileTIme = Dynamic,
                Flags = _MatrixTypeNested::Flags & (HereditaryBits | LvalueBit),
                CoeffReadCost = _MatrixTypeNested::CoeffReadCost
            };
        };

        template<typename MatrixType, typename IndexType>
        struct traits<VectSlice<MatrixType, IndexType> > : traits<MatrixType>
        {
            typedef typename nested<MatrixType>::type MatrixTypeNested;
            typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
            typedef typename MatrixType::StorageKind StorageKind;
            enum {
                RowsAtCompileTime = IndexType::RowsAtCompileTime,
                ColsAtCompileTime = IndexType::ColsAtCompileTime,
                MaxRowsAtCompileTime = IndexType::MaxRowsAtCompileTime,
                MaxColsAtCompileTIme = IndexType::MaxColsAtCompileTime,
                Flags = _MatrixTypeNested::Flags & (HereditaryBits | LvalueBit),
                CoeffReadCost = _MatrixTypeNested::CoeffReadCost
            };
        };

        // TODO: sizes should not be dynamic --- this should really only
        // affect performance when matrix size is known at compile time (not
        // very often)
        template<typename MatrixType>
        struct traits<VectSliceVect<MatrixType> > : traits<MatrixType>
        {
            typedef typename nested<MatrixType>::type MatrixTypeNested;
            typedef typename remove_reference<MatrixTypeNested>::type _MatrixTypeNested;
            typedef typename MatrixType::StorageKind StorageKind;
            enum {
                RowsAtCompileTime = Dynamic,
                ColsAtCompileTime = Dynamic,
                MaxRowsAtCompileTime = Dynamic,
                MaxColsAtCompileTIme = Dynamic,
                Flags = _MatrixTypeNested::Flags & (HereditaryBits | LvalueBit),
                CoeffReadCost = _MatrixTypeNested::CoeffReadCost
            };
        };
    }


    template<typename MatrixType, typename RowIndexType, typename ColIndexType> class MatSlice
        : public MatrixBase<MatSlice<MatrixType, RowIndexType, ColIndexType> >
    {
      public:
        typedef MatrixBase<MatSlice> Base;
        EIGEN_DENSE_PUBLIC_INTERFACE (MatSlice)

        MatSlice (const MatrixType &mat, const RowIndexType &row_index, const ColIndexType &col_index)
            : _matrix (mat), _row_index (row_index), _col_index (col_index)
        {
          eigen_assert (_row_index.minCoeff () >= 0 && _row_index.maxCoeff () < _matrix.rows ());
          eigen_assert (_col_index.minCoeff () >= 0 && _col_index.maxCoeff () < _matrix.cols ());
        }

        EIGEN_INHERIT_ASSIGNMENT_OPERATORS (MatSlice)

        inline Index rows () const {return _row_index.size ();}
        inline Index cols () const {return _col_index.size ();}

        EIGEN_STRONG_INLINE const Scalar& coeffRef (Index row, Index col) const
        {
            const Index &row_ind = _row_index.coeff (row);
            const Index &col_ind = _col_index.coeff (col);
            return _matrix.derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE Scalar& coeffRef (Index row, Index col) 
        {
            const Index &row_ind = _row_index.coeff (row);
            const Index &col_ind = _col_index.coeff (col);
            return _matrix.const_cast_derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE const CoeffReturnType coeff (Index row, Index col) const
        {
            const Index &row_ind = _row_index.coeff (row);
            const Index &col_ind = _col_index.coeff (col);
            return _matrix.coeff (row_ind, col_ind);
        }

      private:
        const typename MatrixType::Nested _matrix;
        const typename RowIndexType::Nested _row_index;
        const typename ColIndexType::Nested _col_index;
    };


    template<typename MatrixType> class MatSliceVect
        : public MatrixBase<MatSliceVect<MatrixType> >
    {
      public:
        typedef MatrixBase<MatSliceVect> Base;
        EIGEN_DENSE_PUBLIC_INTERFACE (MatSliceVect)

        MatSliceVect (const MatrixType &mat, const std::vector<int> &row_index, const std::vector<int> &col_index)
            : _matrix (mat), _row_index (row_index), _col_index (col_index)
        {
            int row_max = *std::max_element (_row_index.begin (), _row_index.end ());
            int row_min = *std::min_element (_row_index.begin (), _row_index.end ());
            int col_max = *std::max_element (_col_index.begin (), _col_index.end ());
            int col_min = *std::min_element (_col_index.begin (), _col_index.end ());
            eigen_assert (row_min >= 0 && row_max < _matrix.rows ());
            eigen_assert (col_min >= 0 && col_max < _matrix.cols ());
        }

        EIGEN_INHERIT_ASSIGNMENT_OPERATORS (MatSliceVect)

        inline Index rows () const {return _row_index.size ();}
        inline Index cols () const {return _col_index.size ();}

        EIGEN_STRONG_INLINE const Scalar& coeffRef (Index row, Index col) const
        {
            const Index &row_ind = _row_index[row];
            const Index &col_ind = _col_index[col];
            return _matrix.derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE Scalar& coeffRef (Index row, Index col) 
        {
            const Index &row_ind = _row_index[row];
            const Index &col_ind = _col_index[col];
            return _matrix.const_cast_derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE const CoeffReturnType coeff (Index row, Index col) const
        {
            const Index &row_ind = _row_index[row];
            const Index &col_ind = _col_index[col];
            return _matrix.coeff (row_ind, col_ind);
        }

      private:
        const typename MatrixType::Nested _matrix;
        const std::vector<int> _row_index;
        const std::vector<int> _col_index;
    };


    // TODO: the resulting VectSlice dim is that of index --- col vect with
    // row vect index results in row vect
    template<typename MatrixType, typename IndexType> class VectSlice
        : public MatrixBase<VectSlice<MatrixType, IndexType> >
    {
      public:
        typedef MatrixBase<VectSlice> Base;
        EIGEN_DENSE_PUBLIC_INTERFACE (VectSlice)

        VectSlice (const MatrixType &mat, const IndexType &index)
            : _matrix (mat), _index (index)
        {
            eigen_assert (_matrix.rows () == 1 || _matrix.cols () == 1);
            eigen_assert (_index.minCoeff () >= 0 && _index.maxCoeff () < _matrix.size ());
        }

        EIGEN_INHERIT_ASSIGNMENT_OPERATORS (VectSlice)

        inline Index rows () const {return _index.rows ();}
        inline Index cols () const {return _index.cols ();}

        EIGEN_STRONG_INLINE const Scalar& coeffRef (Index row, Index col) const
        {
            Index index = _index.coeff (row, col);
            return _matrix.coeffRef (index);
        }

        EIGEN_STRONG_INLINE Scalar& coeffRef (Index row, Index col) 
        {
            Index index = _index.coeff (row, col);
            return _matrix.const_cast_derived ().coeffRef (index);
        }

        EIGEN_STRONG_INLINE const CoeffReturnType coeff (Index row, Index col) const
        {
            Index index = _index.coeff (row, col);
            return _matrix.derived ().coeff (index);
        }

      private:
        const typename MatrixType::Nested _matrix;
        const typename IndexType::Nested _index;
    };


    template<typename MatrixType> class VectSliceVect
        : public MatrixBase<VectSliceVect<MatrixType> >
    {
      public:
        typedef MatrixBase<VectSliceVect> Base;
        EIGEN_DENSE_PUBLIC_INTERFACE (VectSliceVect)

        VectSliceVect (const MatrixType &mat, const std::vector<int> &_index)
            : _matrix (mat), _index (_index), _is_col_vect (mat.cols () == 1)
        {
            eigen_assert (_matrix.rows () == 1 || _matrix.cols () == 1);
            int ind_max = *std::max_element (_index.begin (), _index.end ());
            int ind_min = *std::min_element (_index.begin (), _index.end ());
            eigen_assert (ind_min >= 0 && ind_max < _matrix.size ());
        }

        EIGEN_INHERIT_ASSIGNMENT_OPERATORS (VectSliceVect)

        inline Index rows () const {return _is_col_vect ? _index.size () : 1;}
        inline Index cols () const {return _is_col_vect ? 1 : _index.size ();}

        EIGEN_STRONG_INLINE const Scalar& coeffRef (Index row, Index col) const
        {
            const Index &row_ind = _is_col_vect ? _index[row] : row; 
            const Index &col_ind = _is_col_vect ? col : _index[col]; 
            return _matrix.derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE Scalar& coeffRef (Index row, Index col) 
        {
            const Index &row_ind = _is_col_vect ? _index[row] : row; 
            const Index &col_ind = _is_col_vect ? col : _index[col]; 
            return _matrix.const_cast_derived ().coeffRef (row_ind, col_ind);
        }

        EIGEN_STRONG_INLINE const CoeffReturnType coeff (Index row, Index col) const
        {
            const Index &row_ind = _is_col_vect ? _index[row] : row; 
            const Index &col_ind = _is_col_vect ? col : _index[col]; 
            return _matrix.coeff (row_ind, col_ind);
        }

      private:
        const typename MatrixType::Nested _matrix;
        const std::vector<int> _index;
        bool _is_col_vect;
    };
}

#endif // __MATRIX_SLICE_H__
