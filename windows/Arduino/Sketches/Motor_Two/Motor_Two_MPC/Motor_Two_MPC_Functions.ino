// MCP_Matrix.cpp
#include "MCP_Matrix.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;



namespace MCP {
//Identity matrix creator function. 
//Returns: Identity matrix
BLA::Matrix<Matrix_Size, Matrix_Size, Diagonal<Matrix_Size, float>> MatrixOperations::Identity_Matrix() {  //create a function of type diagonal matrix
    BLA::Matrix<Matrix_Size, Matrix_Size, Diagonal<Matrix_Size, float>> diag;
    diag.Fill(1);
    return diag;
}
} 





