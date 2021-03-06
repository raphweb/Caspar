#include <Arduino.h>

//#include <AFMotor.h>
#include <Ps3Controller.h>

//AF_DCMotor motor1(1, MOTOR12_1KHZ);
//AF_DCMotor motor2(2, MOTOR12_1KHZ);
//AF_DCMotor motor3(3, MOTOR34_1KHZ);
//AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Wifi MAC: FC:F5:C4:01:A3:59
//   BT MAC: FC:F5:C4:01:A3:5A

void setup() {
  Serial.begin(115200);
  Ps3.begin();
  Serial.println("Ready.");
}

void loop() {
  if (Ps3.isConnected()) {
    Serial.println("Connected!");
  }

  delay(5000);
  // put your main code here, to run repeatedly:
  //long time_start = millis();

  // read result
  /*if (Mu.GetValue(VISION_BODY_DETECT, kStatus)) {                   // update vision result and get status, 0: undetected, other: detected
    Serial.println("vision body detected:");
    Serial.print("x = ");
    Serial.println(Mu.GetValue(VISION_BODY_DETECT, kXValue));       // get vision result: x axes value
    Serial.print("y = ");
    Serial.println(Mu.GetValue(VISION_BODY_DETECT, kYValue));       // get vision result: y axes value
    Serial.print("width = ");
    Serial.println(Mu.GetValue(VISION_BODY_DETECT, kWidthValue));   // get vision result: width value
    Serial.print("height = ");
    Serial.println(Mu.GetValue(VISION_BODY_DETECT, kHeightValue));  // get vision result: height value
 
    
    int xAxis = (Mu.GetValue(VISION_BODY_DETECT, kXValue));
    int yAxis = (Mu.GetValue(VISION_BODY_DETECT, kYValue));
    int distance = (Mu.GetValue(VISION_BODY_DETECT, kHeightValue));
    

    Serial.print("xAxis");
    Serial.println(xAxis);
    Serial.print("yAxis");
    Serial.println(yAxis);
    Serial.print("distance");
    Serial.println(distance);
   
    if (xAxis < 35) {
      motor1.setSpeed(140);
      motor1.run(FORWARD);
      motor2.setSpeed(140);
      motor2.run(BACKWARD);
      motor3.setSpeed(140);
      motor3.run(FORWARD);
      motor4.setSpeed(140);
      motor4.run(BACKWARD);
      //delay(10);
      
    }else if (xAxis > 65) {
      motor1.setSpeed(140);
      motor1.run(BACKWARD);
      motor2.setSpeed(140);
      motor2.run(FORWARD);
      motor3.setSpeed(140);
      motor3.run(BACKWARD);
      motor4.setSpeed(140);
      motor4.run(FORWARD);
      //delay(10);
      
    }else if (distance < 30) {
      motor1.setSpeed(130);
      motor1.run(FORWARD);
      motor2.setSpeed(130);
      motor2.run(FORWARD);
      motor3.setSpeed(130);
      motor3.run(FORWARD);
      motor4.setSpeed(130);
      motor4.run(FORWARD);
      //delay(10);

    }else if (distance > 60) {
      motor1.setSpeed(130);
      motor1.run(BACKWARD);
      motor2.setSpeed(130);
      motor2.run(BACKWARD);
      motor3.setSpeed(130);
      motor3.run(BACKWARD);
      motor4.setSpeed(130);
      motor4.run(BACKWARD);
      //delay(10);

    }else{
      motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
    }
  
    
  } else {
    Serial.println("vision body undetected.");
     motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
  }
  Serial.print("fps = ");
  Serial.println(1000/(millis()-time_start));
  Serial.println();*/
  //delay(500);
}
