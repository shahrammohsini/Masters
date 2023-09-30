// MCP_Matrix.h
//This function helps create an idenity matrix. The variables needed are defined here.
#ifndef MCP_MATRIX_H
#define MCP_MATRIX_H

#include <BasicLinearAlgebra.h>

namespace MCP {

const int Matrix_Size = 3;

template <int dim, class ElemT>
struct Diagonal {
    mutable ElemT m[dim];
    mutable ElemT offDiagonal;

    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const {
        if (row == col)
            return m[row];
        else
            return (offDiagonal = 0);
    }
};

class MatrixOperations {
public:
    BLA::Matrix<Matrix_Size, Matrix_Size, Diagonal<Matrix_Size, float>> Identity_Matrix();
};

} // namespace MCP

#endif // MCP_MATRIX_H
