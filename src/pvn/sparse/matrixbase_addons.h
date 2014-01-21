template <typename RowIndexType, typename ColIndexType>
EIGEN_STRONG_INLINE MatSlice<Derived,RowIndexType,ColIndexType> 
operator () (const MatrixBase<RowIndexType> &row_index, const MatrixBase<ColIndexType> &col_index)
{
    typedef MatSlice<Derived,RowIndexType,ColIndexType> ReturnType;
    return ReturnType (this->derived (),row_index.derived (),col_index.derived ());
}

template <typename RowIndexType, typename ColIndexType>
EIGEN_STRONG_INLINE const MatSlice<Derived,RowIndexType,ColIndexType> 
operator () (const MatrixBase<RowIndexType> &row_index, const MatrixBase<ColIndexType> &col_index) const
{
    typedef MatSlice<Derived,RowIndexType,ColIndexType> ReturnType;
    return ReturnType (this->derived (),row_index.derived (),col_index.derived ());
}

EIGEN_STRONG_INLINE MatSliceVect<Derived> 
operator () (const std::vector<int> &row_index, const std::vector<int> &col_index)
{
    typedef MatSliceVect<Derived> ReturnType;
    return ReturnType (this->derived (),row_index,col_index);
}

EIGEN_STRONG_INLINE const MatSliceVect<Derived> 
operator () (const std::vector<int> &row_index, const std::vector<int> &col_index) const
{
    typedef MatSliceVect<Derived> ReturnType;
    return ReturnType (this->derived (),row_index,col_index);
}

template <typename IndexType>
EIGEN_STRONG_INLINE VectSlice<Derived,IndexType>
operator () (const MatrixBase<IndexType> &index)
{
    typedef VectSlice<Derived,IndexType> ReturnType;
    return ReturnType (this->derived (), index.derived ());
}

template <typename IndexType>
EIGEN_STRONG_INLINE const VectSlice<Derived,IndexType>
operator () (const MatrixBase<IndexType> &index) const
{
    typedef VectSlice<Derived,IndexType> ReturnType;
    return ReturnType (this->derived (), index.derived ());
}

EIGEN_STRONG_INLINE VectSliceVect<Derived> 
operator () (const std::vector<int> &index)
{
    typedef VectSliceVect<Derived> ReturnType;
    return ReturnType (this->derived (),index);
}

EIGEN_STRONG_INLINE const VectSliceVect<Derived> 
operator () (const std::vector<int> &index) const
{
    typedef VectSliceVect<Derived> ReturnType;
    return ReturnType (this->derived (),index);
}

EIGEN_STRONG_INLINE Scalar&
operator () (typename internal::traits<Derived>::Index row, typename internal::traits<Derived>::Index col)
{
    eigen_assert (row >=0 && row < rows () && col >=0 && col < cols ());
    return derived ().coeffRef (row,col);
}

EIGEN_STRONG_INLINE const Scalar&
operator () (typename internal::traits<Derived>::Index row, typename internal::traits<Derived>::Index col) const
{
    eigen_assert (row >=0 && row < rows () && col >=0 && col < cols ());
    return derived ().coeffRef (row,col);
}

EIGEN_STRONG_INLINE Scalar&
operator () (typename internal::traits<Derived>::Index index)
{
    eigen_assert (index >=0 && index < size ());
    return derived ().coeffRef (index);
}

EIGEN_STRONG_INLINE const Scalar&
operator () (typename internal::traits<Derived>::Index index) const
{
    eigen_assert (index >=0 && index < size ());
    return derived ().coeffRef (index);
}

