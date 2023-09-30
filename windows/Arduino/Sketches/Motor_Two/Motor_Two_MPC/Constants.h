//This is a header file used to hold constants. To use it #include "MyHeader.h"
#include <BasicLinearAlgebra.h>
#ifndef CONSTANTS_H  //This makes sure the constant isn't repeated twice. If its already declared in the program it will skip it here.
#define CONSTANTS_H

////MPC variables/matracies


const float dt = (0.5); //0.5 Two data points every sec
const int Run_Time = 5;
const int N = int(Run_Time/dt); // (prediction horizon) number of data points in matrix.
//int N = int(3/0.1 + 1); // Needs to be corrected: (prediction horizon) number of data points in matrix.
const int nu = 3; //control horizon
const int matrixSize = nu; // Define the matrix dimensions
BLA::Matrix<nu> u; //#Voltage (Step input). Initialized as a vector of zeros of size N
BLA::Matrix<nu> PHI;//To correct model inaccuracy.
float LAMBDA = 1.01; //Penalty factor
BLA::Matrix<3,3> lambda {LAMBDA,0,0,0,LAMBDA,0,0,0,LAMBDA}; //Much easier to use this matrix for lambda than trying to multiply a scaler to get delta_u. They will have the sdame result
BLA::Matrix<nu> delta_u; //Optimized change in input (u) to minimize error
BLA::Matrix<N> error; //Optimized change in input (u) to minimize error
//BLA::Matrix<N> u; //Input
BLA::Matrix<N> u_prev; // input from previous iteration
BLA::Matrix<N, nu> A; //initialize the dynamic matrix
BLA::Matrix<nu, N> A_T; //initialize the dynamic matrix
//create Identity matrix
  MCP::MatrixOperations matrixOps;
BLA::Matrix<MCP::Matrix_Size, MCP::Matrix_Size, MCP::Diagonal<MCP::Matrix_Size, float>> I_Matrix = matrixOps.Identity_Matrix(); //set I_Matrix to identity matrix

extern BLA::Matrix<N, nu> Matrix_A; //Declare Matrix_A as a global variable so it can be accessed in any file



//constants
volatile int posi = 0; // specify posi as volatile
volatile int pos = 0; // specify posi as volatile
float prevTime = 0; // variable to store the previous time
float currentTime = 0;
volatile int prevPos = 0; // variable to store the previous position
float velocity = 0; // variable to store the calculated velocity
int CPR = 11; //CPR is counts per revolution of encoder

double desired_vel = 1350; //max speed is 2640 RPM
double pwm;
int max_vel = 2700; //This value will change depending on the battery charge
String run = "true"; //change this to "true" if you want to start the program from python. "false" to start it here
//Max voltage applied to motor two is 12 V. Must apply 15.5 V from the power source.



#endif // CONSTANTS_H