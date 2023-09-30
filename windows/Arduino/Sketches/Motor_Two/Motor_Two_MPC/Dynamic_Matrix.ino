#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "Constants.h"
#include <BasicLinearAlgebra.h>
#include <math.h>
using namespace BLA;
//Motor driver pin decleration
#define enA 9
#define in1 6
#define in2 7
//Encoder pin Decleration
#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE




int Matrix_A_Data[N];
//int Matrix_A[]:
BLA::Matrix<N, 1> Vector_A;
Matrix<N, 3> Matrix_A;

int Get_Open_Loop_Data() {


//Encoder Run
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //prevents the posi var from being changed by the interupt while its being read. Makes it more accurate
    pos = posi;
  }
  //Serial.println(pos);
  delay(dt*1000); 
// Calculate velocity
  currentTime = millis(); // get the current time sec
  float timeDiff = currentTime - prevTime; // calculate the time difference
  if (timeDiff > 0) { //This insures we don't devide by zero but also inadvertantly ensures we only do one velocity calculation every sec as timeDiff variable will be in non decimal seconds so for TimeDiff to be >0 it has to be 1. 
    //velocity = ((pos - prevPos)/CPR) / (timeDiff/60000); // calculate the velocity in RPM
    velocity = ((pos - prevPos)/CPR) / (timeDiff/1000); // calculate the velocity
    prevPos = pos; // update the previous position
    prevTime = currentTime; // update the previous time
  }
  //Print Data
  Serial.print(String(millis()/1000));
  Serial.print(",");
  Serial.print(round(desired_vel)); //desired_vel
  Serial.print(",");
  Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   1380




//Motor Run
// Drive the motor in one direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Set the desired speed (PWM value)
  int pwmOutput = 100; // Change this value to adjust the motor speed
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin




  return (round(velocity*60));
}






void ReadEncoder(){ //check what ENCB is doing when A is rising. If Rising then add to pos if not sub from pos
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}




//***********************************************Matrix_A_Data_Collector*************************************************************************

int* Matrix_A_Data_Collector(){  //the int* turns the type of this function to a pointer pointing to location of the first item of the array.
  static int Matrix_A_Data[N];

  Serial.begin(9600);
  //Serial << "B: " << Matrix_A << '\n';
  //Encoder Setup
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),ReadEncoder,RISING); //check when encoder A is rising and un the readEncoder function

  //Motor driver Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);






  //Run get_open_loop_data funciton in a timed loop.
  int Loop_Time = millis()/1000;
  int i = 0;
  while (Loop_Time < Run_Time){
    
    Serial.print(Loop_Time);
    Loop_Time = millis()/1000;
    Matrix_A_Data[i] = Get_Open_Loop_Data();
    Serial.print("Array: ");
    Serial.println(Matrix_A_Data[i]);
    //Matrix_A(i,0) = Matrix_A_Data[i];
    i++;
  }
  


  int size_of_A = sizeof(Matrix_A_Data)/sizeof(Matrix_A_Data[0]); //get the size of A. It has to be divided by sizeof(Matrix_A[0]) bc sizeof function gives you number of bytes insead of number of items in the array
  Serial.print("Size of A: ");
  Serial.println(size_of_A);

  for ( i = 0; i <= size_of_A; i++){
    delay(10);
    Serial.println(Matrix_A_Data[i]);
  }
  Serial.println("Completed matrix");
  //Serial << "B: " << Matrix_A << '\n';

  return Matrix_A_Data;
}









//***********************************************Matrix_A_Builder*************************************************************************
 int* Matrix_A_Builder(){


//Add the data from Matrix_A_Data_Collector to A_Data
int* A_Data = Matrix_A_Data_Collector();

//
for (int i = 0; i < N; i++) {
    //Serial.println(A_Data[i]);
    //A[i] = A_Data[i];
    Vector_A(i,0) = A_Data[i];
  }

Serial.println("------------Printing Vector_A");
Serial << "B: " << Vector_A << '\n';

//create a matrix of zeros
BLA::Matrix<2, 1> zeroRows;
zeroRows.Fill(0);

// Concatenate the zero rows matrix with the original matrix
BLA::Matrix<(N + 2), 1> modified_A_Vector = zeroRows && Vector_A;

Serial.println("------------Printing Modified_A_Vector");
Serial << "B: " << modified_A_Vector << '\n';
  



for (int i = 0; i < (N); i++){

  Matrix_A(i,0) = modified_A_Vector(i+2); //col 1 of matrix A
  Matrix_A(i,1) = modified_A_Vector(i+1); //col 2 of matrrix A
  Matrix_A(i,2) = modified_A_Vector(i); //col 3 of matrix A
}

Serial << "TESTMatrix_A: " << Matrix_A << '\n';

 }

