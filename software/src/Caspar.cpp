#include <Arduino.h>
#include <Ps3Controller.h>
#include <esp32-hal-ledc.h>



#define LERP_FACTOR 10
uint8_t reverse_switch[4] = {1, 1, 1, 1}; //Toggle if you want the engine direction reversed
  
typedef struct {
  const uint8_t speed;
  const uint16_t angle;
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
float degree2rad(float degree){
  return (degree * 71) / 4068;
}




void drive(trajectory wish) {
  // Angle Range in degree 0 -> 359
  // Speed 0 --> 255 
  // 0 is stop 
  // Forward motor direction [+ + + +]
  // Backward motor direction [- - - -]
  // Left motor direction [+ - + -]
  // Right motor direction [- + - +]

  float rad = degree2rad(wish.angle);

  uint8_t speed[4] ={0, 0, 0, 0};
  

  float factor_a = sin(rad - 90);
  float factor_b = sin(rad);

  speed[0] = wish.speed * factor_a;
  speed[1] = wish.speed * factor_b;
  speed[2] = wish.speed * factor_a;
  speed[3] = wish.speed * factor_b;
  
  setMotorSpeed(0, speed[0] * reverse_switch[0]);
  setMotorSpeed(1, speed[1] * reverse_switch[1]);
  setMotorSpeed(2, speed[2] * reverse_switch[2]);
  setMotorSpeed(3, speed[3] * reverse_switch[3]);
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

  // Read in input from remote
  int8_t remote_speed[4] = {Ps3.data.analog.stick.lx,
                            Ps3.data.analog.stick.ly,
                            Ps3.data.analog.stick.rx,
                            Ps3.data.analog.stick.ry}; 

  for (uint8_t i = 0; i < 4; i++) {
    motor_speed[i] = lerp(motor_speed[i], remote_speed[i], LERP_FACTOR);
  }

  /*
  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, motor_speed[1]);
  }
  */
  
  trajectory drive_straight = { 70, 0};
  drive(drive_straight);
  
  updateLatch();
  showData();
  delay(50);
}
