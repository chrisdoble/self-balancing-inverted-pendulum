#include "pins.h"

/**
 * Mathematical constants
 */

/** The constant c in the acceleration equation a = c vin + d v. */
const double c = 6.50454;

/** The constant d in the acceleration equation a = c vin + d v. */
const double d = -33.0202;

/**
 * The state feedback matrix.
 *
 * When multiplied by the state vector gives the force to apply.
 */
const double k[] = {9.35777, 2.05238, -0.968795, -1.56884};

/**
 * Physical constants
 */

/** The mass of the cart (in kg). */
const double cartMass = 0.103;

/** The width of the cart (in m). */
const double cartWidth = 0.035;

/** The maximum voltage of the motor (in V). */
const double motorVoltage = 24.0;

/**
 * The radius of the timing pulley on the axle of the motor (in m).
 *
 * Multiplying the motor angle by this value gives the cart position.
 */
const double timingPulleyRadius = 0.00965;

/** The length of the track (in m). */
const double trackLength = 0.96;

/** The cart position that corresponds to the middle of the track. */
const double centreOfTrack = (trackLength - cartWidth) / 2.0;

/**
 * System state variables
 */

/** The last measured angle of the motor (in rad). */
volatile double motorAngle = 0.0;

/**
 * The last measured distance between the right edge of the cart and the limit
 * switch (in m). Derived from motorAngle.
 */
volatile double cartPosition = 0.0;

/** The last calculated velocity of the cart (in m/s). */
double cartVelocity = 0.0;

/**
 * The last measured angle of the pendulum (in rad).
 *
 * Upright is 0, clockwise rotations as viewed from the front of the system are
 * considered positive.
 */
volatile double pendulumAngle = 0.0;

/** The last calculated angular velocity of the pendulum (in rad/s). */
double pendulumAngularVelocity = 0.0;

/**
 * The number of data points to collect in order to calculate the cart's
 * velocity and the pendulum's angular velocity.
 */
const uint8_t velocityDataCount = 10;

/**
 * The most recent pendulum angles. Used to calculate the pendulum's angular
 * velocity.
 */
double velocityDataAngles[velocityDataCount];

/** The most recent cart positions. Used to calculate the cart's velocity. */
double velocityDataPositions[velocityDataCount];

/**
 * The times at which the most recent cart positions and pendulum angles were
 * recorded. Used to calculate the cart's velocity and the pendulum's angular
 * velocity.
 */
double velocityDataTimes[velocityDataCount];

/** The current state of the system. */
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
   * ±175° at which point the pendulum angle can be offset by ±180°. Now a
   * pendulum angle of 0° corresponds to the pendulum pointing up.
   */
  PENDULUM_ANGLE_ZEROED,
  /** The system is running, trying to keep the pendulum upright. */
  RUNNING,
  /** The system has been killed. */
  KILLED
} state = STARTED;

void setup() {
  attachInterrupt(digitalPinToInterrupt(motorAngleAPin), onMotorAngleAPinChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pendulumAngleAPin), onPendulumAngleAPinChange, CHANGE);

  pinMode(motorAngleAPin, INPUT);
  pinMode(pendulumAngleAPin, INPUT);
  pinMode(motorAngleBPin, INPUT);
  pinMode(pendulumAngleBPin, INPUT);
  pinMode(limitSwitchPin, INPUT);
  pinMode(motorSpeedDirPin, OUTPUT);
  pinMode(motorSpeedPwmPin, OUTPUT);

  Serial.begin(115200);
}

void onPendulumAngleAPinChange() {
  pendulumAngle += getAngleChange(pendulumAngleAPin, pendulumAngleBPin);
}

void onMotorAngleAPinChange() {
  motorAngle += getAngleChange(motorAngleAPin, motorAngleBPin);
  cartPosition = motorAngle * timingPulleyRadius;
}

double getAngleChange(const uint8_t aPin, const uint8_t bPin) {
  // The maximum resolution of the rotary encoder is 2π / (2,048 * 4) but that
  // generates too many interrupts at high speeds and starves the main loop.
  // I've reduced its PPM to 512 and observe both rises and falls of the A
  // channel which results in the following effective resolution.
  const double resolution = 2 * M_PI / (512 * 2);
  return digitalRead(aPin) == HIGH
    ? digitalRead(bPin) == HIGH ? resolution : -resolution
    : digitalRead(bPin) == HIGH ? -resolution : resolution;
}

void loop() {
  checkLimitSwitch();
  updateVelocities();

  switch (state) {
    case STARTED: {
      // Move the cart until it's pressing the limit switch. We do this inside
      // the case statement because the limit switch normally acts as a kill
      // switch and we temporarily want to disable that behaviour.
      while (digitalRead(limitSwitchPin) == LOW) {
        setMotorSpeed(10);
      }

      cartPosition = trackLength - cartWidth;
      motorAngle = cartPosition / timingPulleyRadius;

      // Move the cart until it's not pressing the limit switch (otherwise it
      // will kill the system once we move to the next state).
      while (digitalRead(limitSwitchPin) == HIGH) {
        setMotorSpeed(-10);
      }

      Serial.println("Motor angle zeroed");
      state = MOTOR_ANGLE_ZEROED;
      break;
    }

    case MOTOR_ANGLE_ZEROED: {
      // Move the cart to the centre of the track
      if (cartPosition > centreOfTrack) {
        setMotorSpeed(-10);
      } else {
        Serial.println("Cart centred");
        setMotorSpeed(0);
        state = CART_CENTRED;
      }
      break;
    }

    case CART_CENTRED: {
      // Measure the pendulum angle several times, 100ms apart
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
      // Wait until the pendulum is within 5° of upright (±175° or ±3.054 rad)
      if (abs(pendulumAngle) > 3.054) {
        // Offset the pendulum angle such that upright is 0.0
        pendulumAngle -= M_PI * (pendulumAngle < 0.0 ? -1.0 : 1.0);

        // Zero the pendulum angle measurements so its velocity is correct
        for (uint8_t i = 0; i < velocityDataCount; i++) {
          velocityDataAngles[i] = pendulumAngle;
        }

        Serial.println("Running");
        state = RUNNING;
      }
      break;
    }

    case RUNNING: {
      if (!checkCartPosition() || !checkPendulumAngle()) {
        break;
      }

      const double f = -(k[0] * pendulumAngle + k[1] * pendulumAngularVelocity + k[2] * (cartPosition - centreOfTrack) + k[3] * cartVelocity);
      const double voltage = (f / cartMass - d * cartVelocity) / c;
      const short speed = (short)(voltage / motorVoltage * 255.0);
      setMotorSpeed(speed);

      // Without this the main loop takes 0.5ms to execute and, for some reason,
      // the pendulum drifts to the right very quickly. This delay fixes it.
      delayMicroseconds(1500);

      break;
    }

    case KILLED: {
      setMotorSpeed(0);

      // Hold here so nothing else happens
      while (true) {}

      break;
    }
  }
}

void checkLimitSwitch() {
  if (digitalRead(limitSwitchPin) == HIGH && state != KILLED) {
    Serial.println("Limit switch pressed");
    state = KILLED;
  }
}

/**
 * Calculates the cart's velocity and the pendulum's angular velocity using a
 * first order backward difference over recently collected data points.
 *
 * https://en.wikipedia.org/wiki/Numerical_differentiation#Finite_differences
 */
void updateVelocities() {
  const uint8_t n = velocityDataCount;
  for (uint8_t i = 0; i < n - 1; i++) {
    velocityDataAngles[i] = velocityDataAngles[i + 1];
    velocityDataPositions[i] = velocityDataPositions[i + 1];
    velocityDataTimes[i] = velocityDataTimes[i + 1];
  }

  velocityDataAngles[n - 1] = pendulumAngle;
  velocityDataPositions[n - 1] = cartPosition;
  velocityDataTimes[n - 1] = micros() / 1000000.0;

  const double dt = velocityDataTimes[n - 1] - velocityDataTimes[0];
  cartVelocity = (velocityDataPositions[n - 1] - velocityDataPositions[0]) / dt;
  pendulumAngularVelocity = (velocityDataAngles[n - 1] - velocityDataAngles[0]) / dt;
}

void setMotorSpeed(const short speed) {
  digitalWrite(motorSpeedDirPin, speed < 0 ? HIGH : LOW);

  // The 24V motor is overkill. Limit the speed so nothing breaks.
  analogWrite(motorSpeedPwmPin, min(abs(speed), 100));
}

/**
 * Computes the interval spanning a range of values and returns its size.
 *
 * For example, the interval spanning the values [1.0, 3.0, 2.0] is [1.0, 3.0]
 * and its size is 3.0 - 1.0 = 2.0.
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

/** If the cart gets within 10cm of either end of the track, kill it. */
bool checkCartPosition() {
  const double buffer = 0.1;
  if (cartPosition < buffer || cartPosition > trackLength - cartWidth - buffer) {
    Serial.println("Cart too close to the end");
    state = KILLED;
    return false;
  }

  return true;
}

/** If the pendulum is more than 45° away from verticle, kill it. */
bool checkPendulumAngle() {
  if (abs(pendulumAngle) > M_PI / 4) {
    Serial.println("Pendulum angle too great");
    state = KILLED;
    return false;
  }

  return true;
}
