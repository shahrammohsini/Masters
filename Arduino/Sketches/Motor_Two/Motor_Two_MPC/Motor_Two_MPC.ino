#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <BasicLinearAlgebra.h>
#include "MCP_Matrix.h"
#include "Constants.h"



using namespace BLA;

#define enA 9
#define in1 6
#define in2 7
//Encoder pin Decleration
#define ENCA 3 // YELLOW
#define ENCB 2 // WHITE
// void Motor_Two_MPC_Functions();
// int Dynamic_Matrix();
// void MPC_Controller();

void setup() {
  Serial.begin(9600);

  //create Identity matrix
  MCP::MatrixOperations matrixOps;
  // BLA::Matrix<MCP::Matrix_Size, MCP::Matrix_Size, MCP::Diagonal<MCP::Matrix_Size, float>> I_Matrix = matrixOps.Identity_Matrix(); //set I_Matrix to identity matrix

  Serial << "I_Matrix: " << I_Matrix << '\n';

  //Encoder Setup
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); //check when encoder A is rising and un the readEncoder function

  //Motor driver Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);



  Matrix_A_Builder();
  A = Matrix_A;
  Serial << "A: " << A << '\n';
  A_T = ~A;
Serial << "Matrix_A_T: " << A_T << '\n';
  //Controller();
}

void loop() {
  while(run == "false"){  //Don't run the program until data is recived from python
      Serial.println("not running");
      if (Serial.available() > 0){
        run = Serial.readString();
      
      }
    }

//Encoder Run
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //prevents the posi var from being changed by the interupt while its being read. Makes it more accurate
    pos = posi;
  }
  //Serial.println(pos);
  delay(800); 
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
  // Serial.print(String(millis()));
  // Serial.print(",");
  // Serial.print(round(desired_vel)); //desired_vel
  // Serial.print(",");
  // Serial.println(round(velocity*60));  //RPM  Max speed with load: 2640rpm (from experimentation)   Max speed without loat 4800rpm (from datasheed)   1380
  Serial << "A: " << A << '\n';

  

//Motor Run
// Drive the motor in one direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  


//MPC Controller


//delta_u = A_T*Invert(A_T*A - LAMBDA*I_Matrix)*(error);
Serial.print("A_Size: "); Serial.print(A.Rows);Serial.print(","); Serial.println(A.Cols); 
Serial.print("A_T_Size: "); Serial.print(A_T.Rows);Serial.print(","); Serial.println(A_T.Cols); 
Serial.print("error: "); Serial.print(error.Rows);Serial.print(","); Serial.println(error.Cols); 

// (3,10)*(10,3) = (3,3) A_T*A

// (3,3) - (3,3) = (3,3) A_T*A- LAMBDA*I

// (3,10)(10,1) = (3,1) A_T*error

// (3,3) * (3,1) = (3,1) inverse(A_T*A- LAMBDA*I) * (A_T*error)
//delta_u = Invert(A_T*A - (lambda*I_Matrix))*(A_T*(error));


BLA::Matrix<3, 3> C = A_T*A - (lambda*I_Matrix);

BLA::Matrix<3, 3> C_inv = Invert(C);; //I'm stuck here. I can't get the inverse of the matrix. The BasicLinearAlgebra library's Invert matrix can't handle large numbers


Serial << "C_inv: " << C_inv << '\n';












  // Set the desired speed (PWM value)
  int pwmOutput = 100; // Change this value to adjust the motor speed
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin


}

void readEncoder(){ //check what ENCB is doing when A is rising. If Rising then add to pos if not sub from pos
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}