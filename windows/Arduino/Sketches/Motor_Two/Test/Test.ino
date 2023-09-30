#include "Constants.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

int Dynamic_Matrix();
BLA::Matrix<N, 3> A;
int* A_Data; //A is a pointer to the Matrix_A
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //A_Data = Matrix_A_Data_Collector();
  // Serial.println("reprinting the array");
  // for (int i = 0; i < N; i++) {
  //   delay(500);
  //   Serial.println(A_Data[i]);
  //   A[i] = A_Data[i];
  // }


  Matrix_A_Builder();
  A = Matrix_A;
  Serial << "A: " << A << '\n';
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(sizeof(A)/sizeof(A[0]));
  // Serial.println("Done");
  // delay(1000);
  
}
