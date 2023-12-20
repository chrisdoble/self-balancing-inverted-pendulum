/**
 * The A_LSB_U pin from the pendulum's rotary encoder.
 *
 * Needs to be on pin 2 so we can attach an interrupt.
 */
uint8_t pendulumAngleAPin = 2;

/**
 * The A_LSB_U pin from the motor's rotary encoder.
 *
 * Needs to be on pin 3 so we can attach an interrupt.
 */
uint8_t motorAngleAPin = 3;

/**
 * The B_Dir_V pin from the pendulum's rotary encoder.
 *
 * It's a quarter period shifted from the A_LSB_U pin from which we can
 * determine the direction of rotation.
 */
uint8_t pendulumAngleBPin = 4;

/**
 * The MagINCn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved closer to the rotary encoder. If both
 * this and the MagDECn pin are low the magnet isn't in range.
 */
uint8_t pendulumAngleMagIncPin = 5;

/**
 * The MagDECn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved further from the rotary encoder. If both
 * this and the MagINCn pin are low the magnet isn't in range.
 */
uint8_t pendulumAngleMagDecPin = 6;

/** The B_Dir_V pin from the motor's rotary encoder. */
uint8_t motorAngleBPin = 7;

/** The MagINCn pin from the motor's rotary encoder. */
uint8_t motorAngleMagIncPin = 8;

/** The MagDECn pin from the motor's rotary encoder. */
uint8_t motorAngleMagDecPin = 9;

/**
 * The output pin from the limit switch.
 *
 * If this is high something has pressed the limit switch.
 */
uint8_t limitSwitchPin = 10;

/**
 * The motor's speed pin.
 *
 * Accepts a PWM signal that controls the motor's speed.
 */
uint8_t motorSpeedPwmPin = 11;

/**
 * The motor's direction pin.
 *
 * LOW is towards the motor, HIGH is away from it.
 */
uint8_t motorSpeedDirPin = 12;

/**
 * Whether the system has been killed.
 *
 * When the system is killed the motor turns off and the system must be
 * restarted. This happens if, e.g., the cart gets too close to an end.
 */
bool killed = false;

/** The last measured angle of the pendulum in degrees. */
volatile double pendulumAngle = 0.0;

/** The last measured angle of the motor in degrees. */
volatile double motorAngle = 0.0;

void setup() {
  // Delay 50ms to let the rotary encoders wake up
  delay(50);

  attachInterrupt(digitalPinToInterrupt(pendulumAngleAPin), onPendulumAngleAPinChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorAngleAPin), onMotorAngleAPinChange, CHANGE);

  pinMode(pendulumAngleAPin, INPUT);
  pinMode(motorAngleAPin, INPUT);
  pinMode(pendulumAngleBPin, INPUT);
  pinMode(pendulumAngleMagIncPin, INPUT);
  pinMode(pendulumAngleMagDecPin, INPUT);
  pinMode(motorAngleBPin, INPUT);
  pinMode(motorAngleMagIncPin, INPUT);
  pinMode(motorAngleMagDecPin, INPUT);
  pinMode(limitSwitchPin, INPUT);
  pinMode(motorSpeedPwmPin, OUTPUT);
  pinMode(motorSpeedDirPin, OUTPUT);

  Serial.begin(9600);
}

void onPendulumAngleAPinChange() {
  pendulumAngle += getAngleChange(pendulumAngleAPin, pendulumAngleBPin);
}

void onMotorAngleAPinChange() {
  motorAngle += getAngleChange(motorAngleAPin, motorAngleBPin);
}

double getAngleChange(uint8_t aPin, uint8_t bPin) {
  // The rotary encoders run in 9 bit mode, so a change in a A_LSU_U pin
  // corresponds to a change of 0.704°.
  double resolution = 0.704;

  if (digitalRead(aPin) == HIGH) {
    if (digitalRead(bPin) == HIGH) {
      return resolution;
    } else {
      return -resolution;
    }
  } else {
    if (digitalRead(bPin) == HIGH) {
      return -resolution;
    } else {
      return resolution;
    }
  }
}

void loop() {
  if (killed) {
    return;
  }

  if (digitalRead(limitSwitchPin) == HIGH) {
    kill();
    return;
  }

  if (!isMagnetInRange(pendulumAngleMagIncPin, pendulumAngleMagDecPin)) {
    Serial.println("Pendulum magnet not in range");
    return;
  }

  if (!isMagnetInRange(motorAngleMagIncPin, motorAngleMagDecPin)) {
    Serial.println("Motor magnet not in range");
    return;
  }

  // Print the values 0 and 360 to act as bounds on the y axis
  Serial.print(0);
  Serial.print(" ");
  Serial.print(360);
  Serial.print(" ");

  // Print the pendulum and motor angles
  Serial.print(pendulumAngle);
  Serial.print(" ");
  Serial.println(motorAngle);
}

void kill() {
  digitalWrite(motorSpeedPwmPin, LOW);
  killed = true;
  Serial.println("Killed");
}

bool isMagnetInRange(uint8_t magIncPin, uint8_t magDecPin) {
  // If the magnet is in range both MagINCn and MagDECn are turned off. As they
  // are open drain and there's a pull up resistor we'll see the pins as high.
  return digitalRead(magIncPin) == HIGH && digitalRead(magDecPin) == HIGH;
}
