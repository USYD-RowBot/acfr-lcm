#include <iostream>
#include "eigen_utils.h"

std::ostream& operator<< (std::ostream& os, const std::vector<int>& v)
{
    unsigned int size = v.size ();
    for (unsigned int i=0;i<size;++i) os << v[i] << (i == size-1 ? " " : "\n");
    return os;
}

int main (int argc, char *argv[])
{
    // 5x5 matrix
    Eigen::MatrixXd A (5,5);
    A << 
      00, 01, 02, 03, 04,
      10, 11, 12, 13, 14, 
      20, 21, 22, 23, 24, 
      30, 31, 32, 33, 34,
      40, 41, 42, 43, 44;
    std::cout << "A = " << std::endl << A << std::endl;
    
    // 4 element eigen index
    Eigen::Index i (4);
    i << 1,0,2,3;
    std::cout << "i = " << std::endl << i << std::endl;

    // access A with i
    std::cout << "A(i,i) = " << std::endl 
        << A(i,i) << std::endl;

    // 4 element std vector index
    std::vector<int> i_vect (i.data(), i.data() + i.size());
    std::cout << "i_vect = " << std::endl << i_vect << std::endl;

    // access A with i_vect
    std::cout << "A(i_vect,i_vect) = " << std::endl 
        << A(i_vect,i_vect) << std::endl;

    // 4x4 matrix
    Eigen::MatrixXd B(4,4);
    B <<
      55, 56, 57, 58,
      65, 66, 67, 68,
      75, 76, 77, 78,
      85, 86, 87, 88;
    std::cout << "B = " << std::endl << B << std::endl;

    // assign i elements of A to B
    A(i,i) = B;
    std::cout << "A(i,i) = B, A = " << std::endl << A << std::endl;
    //
    // assign i_vect elements of A to i_vect elements of B
    A(i_vect,i_vect) = B(i_vect,i_vect);
    std::cout << "A(i_vect,i_vect) = B(i_vect,i_vect), A = " << std::endl << A << std::endl;

    // 5 element vector
    Eigen::VectorXd v (5);
    v << 0, 1, 2, 3, 4;
    std::cout << "v = " << std::endl << v << std::endl;
    
    // access i elements of v
    std::cout << "v(i) = " << std::endl
        << v(i) << std::endl;

    // access i_vect elements of v
    std::cout << "v(i_vect) = " << std::endl
        << v(i_vect) << std::endl;

    // assign i elements of v to i elements of 3rd col of A
    Eigen::Index c (1); c << 3;
    v(i) = A(i,c);
    std::cout << "c = 3, v(i) = A(i,c), v = " << std::endl 
        << v << std::endl;

    // scalar access to A
    std::cout << "A(3,3) = " << std::endl
        << A(3,3) << std::endl;

    // scalar access to v
    std::cout << "v(3) = " << std::endl
        << v(3) << std::endl;
    
    // initialize new matrix C
    Eigen::MatrixXd C = A(i_vect,i_vect);
    std::cout << "C = A(i_vect,i_vect), C = " << std::endl
        << C << std::endl;

    return 0;
}
