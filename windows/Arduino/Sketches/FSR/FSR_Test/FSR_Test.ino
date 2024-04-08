int FSR; //Force sensative resistor


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:

FSR = 1023 - analogRead(A0);

Serial.print("FSR Value = ");
Serial.println(FSR);

delay(150);




}
