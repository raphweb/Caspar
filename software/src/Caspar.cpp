#include <Arduino.h>
#include <Ps3Controller.h>
#include <esp32-hal-ledc.h>


#define LERP_FACTOR 10

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3

typedef struct {
  uint8_t speed;  // speed 0 -> 255 (0 == MIN; 255 == MAX)
  uint16_t angle; // angle range in degree 0 -> 359
  int8_t spin_speed; // -127 -> -1 counter clockwise spin speed, 1 -> 127 clockwise spin speed
} trajectory;

typedef struct {
  const uint8_t pin;
  const uint8_t aBit;
  const uint8_t bBit;
} motor_t;

// pin mappings for the dc motors of the old arduino motor shield
// aBit and bBit are the positions of the direction bits in the 8 bit latch_state
const motor_t motors[4] = {
    {23, 2, 3},  // motor shield pin 11 <=> gpio 23 on ESP32, aBit, bBit
    {25, 1, 4},  // motor shield pin  3 <=> gpio 25 on ESP32, aBit, bBit
    {27, 5, 7},  // motor shield pin  6 <=> gpio 27 on ESP32, aBit, bBit
    {16, 0, 6}   // motor shield pin  5 <=> gpio 16 on ESP32, aBit, bBit
};

#define MOTORLATCH 0
#define MOTORCLK 1
#define MOTORENABLE 2
#define MOTORDATA 3

// pin mappings for the 74HCT595 shift register
const uint8_t latchPin[4] = {
    19,  // motor latch (pin 12) <=> gpio 19 on ESP32
    17,  // motor clk (pin 4)    <=> gpio 17 on ESP32
    14,  // motor enable (pin 7) <=> gpio 14 on ESP32
    12   // motod data (pin 8)   <=> gpio 12 on ESP32
};

static uint8_t latch_state = 0;

void showData() {
  Serial.print("left(");
  Serial.print(Ps3.data.analog.stick.lx);
  Serial.print(", ");
  Serial.print(Ps3.data.analog.stick.ly);
  Serial.print("); right(");
  Serial.print(Ps3.data.analog.stick.rx);
  Serial.print(", ");
  Serial.print(Ps3.data.analog.stick.ry);
  Serial.println(");");
}

static int battery = 0;

void notify() {
  if (battery != Ps3.data.status.battery) {
    battery = Ps3.data.status.battery;
    Serial.print("The controller battery is ");
    if (battery == ps3_status_battery_charging)
      Serial.println("charging");
    else if (battery == ps3_status_battery_full)
      Serial.println("FULL");
    else if (battery == ps3_status_battery_high)
      Serial.println("HIGH");
    else if (battery == ps3_status_battery_low)
      Serial.println("LOW");
    else if (battery == ps3_status_battery_dying)
      Serial.println("DYING");
    else if (battery == ps3_status_battery_shutdown)
      Serial.println("SHUTDOWN");
    else
      Serial.println("UNDEFINED");
  }
}

void onConnect() {
  Serial.println("Ps3 Controller connected.");
}

// motorNum  => dc motor[motorNum+1]
// speed will be mapped as follows:
// -32..31   => speed = 0
// -128..-33 => speed = abs(speed) backwards
// 32..127   => speed = speed forwards
void setMotorSpeed(uint8_t motorNum, int8_t speed) {
  // a and b bits are used to control the direction
  // a = 0 b = 0 => release (allows passive movement)
  // a = 1 b = 0 => forward
  // a = 0 b = 1 => backward
  // a = 1 b = 1 => freeze (no movement at all)
  uint8_t a = motors[motorNum].aBit;
  uint8_t b = motors[motorNum].bBit;
  if (speed >= -32 && speed <= 31) {
    // don't move
    latch_state &= ~_BV(a);
    latch_state &= ~_BV(b);
  } else if (speed > 31) {
    // forwards
    latch_state |= _BV(a);
    latch_state &= ~_BV(b);
  } else if (speed < -32) {
    // backwards
    latch_state &= ~_BV(a);
    latch_state |= _BV(b);
  }
  ledcWrite(motorNum, abs(speed));
}

trajectory readInput() {
  // read in input from remote
  int8_t remote_input[3] = {
    Ps3.data.analog.stick.lx,
    Ps3.data.analog.stick.ly,
    Ps3.data.analog.stick.rx // right hand analog stick x axis used for spin
  };
  // increment negative values, so we have two zero positions
  for (uint8_t i = 0; i < 3; i++) {
    if (remote_input[i] < 0) remote_input[i]++;
  }
  // calc angle
  uint16_t angle;
  if (remote_input[0] != 0 || remote_input[1] != 0) {
    angle = (uint16_t)(degrees(atan2f(-remote_input[0], remote_input[1])) + 180) % 360;
  } else {
    angle = 0;
  }
  // calculate the mapped values from a square to a circle
  // see: https://mathproofs.blogspot.com/2005/07/mapping-square-to-circle.html
  // division by 32258 normalizes remote input to [0.0f..1.0f], squares it, and divides it by 2
  // i.e.: 32258 == 127 * 127 * 2
  // map square area values to circualr ones
  float mapped_input[2] = {
    sqrtf(1.0f - sq(remote_input[1]) / 32258.0f) * remote_input[0],
    sqrtf(1.0f - sq(remote_input[0]) / 32258.0f) * remote_input[1]
  };
  // calc speed
  uint8_t speed = roundf(sqrtf(sq(mapped_input[0]) + sq(mapped_input[1])) * 2.0f);
  Serial.println("Angle: " + String(angle) + ", speed: " + String(speed));
  return {speed, angle, remote_input[2]};
}

void drive(trajectory wish) {
  // Angle Range in degree 0 -> 359
  // Speed 0 --> 255 
  // 0 is stop 
  // Forward motor direction [+ + + +]
  // Backward motor direction [- - - -]
  //  motor direction [+ + + +]
  // Forward motor direction [+ + + +]


}



void updateLatch() {
  // the switch time of the 74HCT595 shift register is ~30ns
  // the ESP32 can switch IOs with upto 80MHz (12.5ns)
  // luckily, the digitalWrite method takes more than two cycles
  // so the MOTORCLK bit shifting doesn't "swallow" bits
  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(latchPin[MOTORLATCH], LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(latchPin[MOTORDATA], LOW);

  for (uint8_t i = 0; i < 8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(latchPin[MOTORCLK], LOW);

    if (latch_state & _BV(7 - i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(latchPin[MOTORDATA], HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(latchPin[MOTORDATA], LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(latchPin[MOTORCLK], HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(latchPin[MOTORLATCH], HIGH);
}

void setup() {
  Serial.begin(115200);
  // init ps3 controller
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin();
  Serial.println("BT Address: " + Ps3.getAddress());

  // init pwms for all 4 motors and the 4 shift register pins
  for (uint8_t i = 0; i < 4; i++) {
    // 880 Hz pwm signal with 7 bits resolution
    ledcSetup(i, 880, 7);
    ledcAttachPin(motors[i].pin, i);
    // attach the shift register pins for the motor directions
    pinMode(latchPin[i], OUTPUT);
  }
  latch_state = 0;

  Serial.println("Ready.");
}


float lerp(float a, float b, float x)
{ 
  return a + x * (b - a);
}

int8_t motor_speed[4] = {0, 0, 0, 0};

void loop() {
  //Check if controller is connected
  if (!Ps3.isConnected()){
    // If no controller, die!
    return;
  }

  //Check if controller is charged
  if ( Ps3.data.status.battery < ps3_status_battery_high ){
    return;
  }

  readInput();

  // Read in input from remote
  int8_t remote_speed[4] = {Ps3.data.analog.stick.lx,
                            Ps3.data.analog.stick.ly,
                            Ps3.data.analog.stick.rx,
                            Ps3.data.analog.stick.ry}; 

  for (uint8_t i = 0; i < 4; i++) {
    motor_speed[i] = lerp(motor_speed[i], remote_speed[i], LERP_FACTOR);
  }

  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, remote_speed[1]);
  }

  updateLatch();
  //showData();
  delay(50);
}
