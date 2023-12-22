/**
 * The A_LSB_U pin from the pendulum's rotary encoder.
 *
 * Needs to be on pin 2 so we can attach an interrupt.
 */
const uint8_t pendulumAngleAPin = 2;

/**
 * The A_LSB_U pin from the motor's rotary encoder.
 *
 * Needs to be on pin 3 so we can attach an interrupt.
 */
const uint8_t motorAngleAPin = 3;

/**
 * The B_Dir_V pin from the pendulum's rotary encoder.
 *
 * It's a quarter period shifted from the A_LSB_U pin from which we can
 * determine the direction of rotation.
 */
const uint8_t pendulumAngleBPin = 4;

/**
 * The MagINCn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved closer to the rotary encoder. If both
 * this and the MagDECn pin are low the magnet isn't in range.
 */
const uint8_t pendulumAngleMagIncPin = 5;

/**
 * The MagDECn pin from the pendulum's rotary encoder.
 *
 * If this is low the magnet has moved further from the rotary encoder. If both
 * this and the MagINCn pin are low the magnet isn't in range.
 */
const uint8_t pendulumAngleMagDecPin = 6;

/** The B_Dir_V pin from the motor's rotary encoder. */
const uint8_t motorAngleBPin = 7;

/** The MagINCn pin from the motor's rotary encoder. */
const uint8_t motorAngleMagIncPin = 8;

/** The MagDECn pin from the motor's rotary encoder. */
const uint8_t motorAngleMagDecPin = 9;

/**
 * The output pin from the limit switch.
 *
 * If this is high something has pressed the limit switch.
 */
const uint8_t limitSwitchPin = 10;

/**
 * The motor's speed pin.
 *
 * Accepts a PWM signal that controls the motor's speed.
 */
const uint8_t motorSpeedPwmPin = 11;

/**
 * The motor's direction pin.
 *
 * LOW is towards the motor, HIGH is away from it.
 */
const uint8_t motorSpeedDirPin = 12;

/** The last measured angle of the pendulum (in rad). */
volatile double pendulumAngle = 0.0;

/** The last measured angle of the motor (in rad). */
volatile double motorAngle = 0.0;

/** The last measured distance between the cart and the limit switch (in m). */
volatile double cartPosition = 0.0;

/** The length of the track (in m). */
const double trackLength = 0.965;

/** The width of the cart (in m). */
const double cartWidth = 0.035;

/**
 * In order to calculate the cart's velocity we record its position over the n
 * last iterations of the main loop and the times at which those positions were
 * recorded. We then calculate its velocity as the difference between the newest
 * and oldest records divided by the time between them.
 *
 * This complicated approach is necessary because:
 *
 * - we can't calculate it from the two most recent cart positions as the time
 *   delta is too small and the velocity ends up being infinity, and
 * - we can't calculate it in the rotary encoder interrupt as that's only called
 *   when the angle changes, i.e. if it's stationary the velocity won't go to 0.
 */
double cartVelocity = 0.0;

/** The number of cart positions to record. */
const uint8_t cartVelocityMeasurementCount = 20;

/** The n most recent cart positions. */
double cartVelocityPositions[cartVelocityMeasurementCount];

/** The times at which the n most recent cart positions were recorded. */
double cartVelocityTimes[cartVelocityMeasurementCount];

/** The states the system moves through during operation. */
enum State {
  /**
   * The system has just started. Move the cart until it is pressing the
   * limit switch, at which point the motor angle can be zeroed. This ensures
   * the motor angle and the cart position are accurate going forward.
   */
  STARTED,
  /** The motor angle has been zeroed. Move the cart to the centre. */
  MOTOR_ANGLE_ZEROED,
  /**
   * The cart is in the centre. Wait for the pendulum to stop swinging at which
   * point the pendulum angle can be zeroed. Note that this means a pendulum
   * angle of 0° corresponds to the pendulum pointing down rather than up.
   */
  CART_CENTRED,
  /**
   * The pendulum angle has been zeroed. Wait for the pendulum angle to near
   * ±160° at which point the pendulum angle can be offset by ±180°. Now a
   * pendulum angle of 0° corresponds to the pendulum pointing up.
   */
  PENDULUM_ANGLE_ZEROED,
  /** The system is running, tryin to keep the pendulum upright. */
  RUNNING,
  /** The system has been killed. */
  KILLED
} state = STARTED;

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

  // The radius of the timing pulley on the axle of the motor (in m).
  // Multiplying the motor angle by this value gives the distance between the
  // timing pulley and the cart.
  const double timingPulleyRadius = 0.00965;
  cartPosition = motorAngle * timingPulleyRadius;
}

double getAngleChange(const uint8_t aPin, const uint8_t bPin) {
  // The rotary encoders run in 9 bit mode, so a change in a A_LSU_U pin
  // corresponds to a change of 0.704° or 0.0123 rad.
  const double resolution = 0.0123;

  if (digitalRead(aPin) == HIGH) {
    if (digitalRead(bPin) == HIGH) {
      return -resolution;
    } else {
      return resolution;
    }
  } else {
    if (digitalRead(bPin) == HIGH) {
      return resolution;
    } else {
      return -resolution;
    }
  }
}

void loop() {
  checkLimitSwitch();
  checkMagnetsInRange();
  updateCartVelocity();

  switch (state) {
    case STARTED: {
      // Move the cart until it's pressing the limit switch. We do this inside
      // the case statement because the limit switch normally acts as a kill
      // switch and we temporarily want to disable that behaviour.
      while (digitalRead(limitSwitchPin) == LOW) {
        setMotorSpeed(-10);
      }

      motorAngle = 0.0;

      // Move the cart until it's not pressing the limit switch (otherwise it
      // will kill the system once we move to the next state).
      while (digitalRead(limitSwitchPin) == HIGH) {
        setMotorSpeed(10);
      }

      Serial.println("Motor angle zeroed");
      state = MOTOR_ANGLE_ZEROED;
      break;
    }

    case MOTOR_ANGLE_ZEROED: {
      // Move the cart to the centre of the track
      const double target = (trackLength - cartWidth) / 2.0;
      if (cartPosition < target) {
        setMotorSpeed(10);
      } else {
        Serial.println("Cart centred");
        setMotorSpeed(0);
        state = CART_CENTRED;
      }
      break;
    }

    case CART_CENTRED: {
      // Measure the pendulum angle a number of times, 100ms apart
      const uint8_t angleCount = 20;
      double angles[angleCount];

      for (uint8_t i = 0; i < angleCount; i++) {
        angles[i] = pendulumAngle;
        delay(100);
      }

      // Continue measuring the pendulum angle every 100ms until all of them
      // are within 1° (0.0175 rad) of each other, i.e. the pendulum is still.
      uint8_t i = 0;
      while (getIntervalSize(angles, angleCount) > 0.0175) {
        angles[i] = pendulumAngle;
        i = (i + 1) % angleCount;
        delay(100);
      }

      pendulumAngle = 0.0;

      // Move the cart slightly to let the user know it's ready for the pendulum
      // to be rotated upright. This won't be necessary once it can swing the
      // pendulum upright itself.
      setMotorSpeed(-5);
      delay(100);
      setMotorSpeed(5);
      delay(100);
      setMotorSpeed(0);

      Serial.println("Pendulum angle zeroed");
      state = PENDULUM_ANGLE_ZEROED;
      break;
    }

    case PENDULUM_ANGLE_ZEROED: {
      // Wait for the user to rotate the pendulum within 10° of upright (±170°
      // or ±2.967 rad), then offset the pendulum angle such that upright is 0.0.
      if (abs(pendulumAngle) > 2.967) {
        pendulumAngle += -M_PI * sign(pendulumAngle);
        Serial.println("Running");
        state = RUNNING;
      }
      break;
    }

    case RUNNING: {
      if (!checkCartPosition()) {
        break;
      }

      setMotorSpeed((short) (10.0 * sin(millis() / 500.0)));

      Serial.print(0);
      Serial.print(" ");
      Serial.print(trackLength);
      Serial.print(" ");
      Serial.print(cartPosition);
      Serial.print(" ");
      Serial.println(cartVelocity);
      break;
    }

    case KILLED: {
      setMotorSpeed(0);
      break;
    }
  }
}

void checkLimitSwitch() {
  if (digitalRead(limitSwitchPin) == HIGH) {
    Serial.println("Limit switch pressed");
    state = KILLED;
  }
}

void checkMagnetsInRange() {
  if (!isMagnetInRange(pendulumAngleMagIncPin, pendulumAngleMagDecPin)) {
    Serial.println("Pendulum magnet not in range");
    state = KILLED;
  }

  if (!isMagnetInRange(motorAngleMagIncPin, motorAngleMagDecPin)) {
    Serial.println("Motor magnet not in range");
    state = KILLED;
  }
}

bool isMagnetInRange(const uint8_t magIncPin, const uint8_t magDecPin) {
  // If the magnet is in range both MagINCn and MagDECn are turned off. As they
  // are open drain and there's a pull up resistor we'll see the pins as high.
  return digitalRead(magIncPin) == HIGH && digitalRead(magDecPin) == HIGH;
}

void setMotorSpeed(const short speed) {
  digitalWrite(motorSpeedDirPin, speed < 0 ? LOW : HIGH);
  analogWrite(motorSpeedPwmPin, min(abs(speed), 10));
}

/**
 * Computes the interval spanning a range of values and returns its size.
 *
 * For example, the interval size of [1.0, 3.0, 2.0] is 3.0 - 1.0 = 2.0.
 */
double getIntervalSize(const double *data, uint8_t length) {
  double maximum = -INFINITY;
  double minimum = INFINITY;

  for (uint8_t i = 0; i < length; i++) {
    double datum = data[i];
    maximum = max(datum, maximum);
    minimum = min(datum, minimum);
  }

  return maximum - minimum;
}

double sign(double value) {
  return value < 0.0 ? -1.0 : 1.0;
}

bool checkCartPosition() {
  const double buffer = 0.1;
  if (cartPosition < buffer || cartPosition > trackLength - cartWidth - buffer) {
    Serial.println("Cart too close to the end");
    state = KILLED;
    return false;
  }

  return true;
}

void updateCartVelocity() {
  const uint8_t n = cartVelocityMeasurementCount;
  for (uint8_t i = 0; i < n - 1; i++) {
    cartVelocityPositions[i] = cartVelocityPositions[i + 1];
    cartVelocityTimes[i] = cartVelocityTimes[i + 1];
  }

  cartVelocityPositions[n - 1] = cartPosition;
  cartVelocityTimes[n - 1] = micros() / 1000000.0;

  const double dx = cartVelocityPositions[n - 1] - cartVelocityPositions[0];
  const double dt = cartVelocityTimes[n - 1] - cartVelocityTimes[0];
  cartVelocity = dx / dt;
}
