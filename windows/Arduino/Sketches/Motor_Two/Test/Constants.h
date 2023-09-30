//This is a header file used to hold constants. To use it #include "MyHeader.h"
#include <BasicLinearAlgebra.h>
#ifndef CONSTANTS_H  //This makes sure the constant isn't repeated twice. If its already declared in the program it will skip it here.
#define CONSTANTS_H

//MPC
const float dt = (0.5); //0.5 Two data points every sec
const int Run_Time = 5;
const int N = int(Run_Time/dt); // (prediction horizon) number of data points in matrix.

extern BLA::Matrix<N, 3> Matrix_A; //Declare Matrix_A as a global variable so it can be accessed in any file

#endif // CONSTANTS_H