#include <BasicLinearAlgebra.h>
#include "pins.h"

using namespace BLA;

/**
 * Mathematical constants
 */

/** The constant c in the acceleration equation a = c vin + d v. */
const double c = 4.24182;

/** The constant d in the acceleration equation a = c vin + d v. */
const double d = -21.1272;

/** The state feedback matrix. */
const Matrix<1, 4, double> k = {67.8753, 16.8168, -9.40109, -14.6775};

/**
 * Physical constants
 */

/** The mass of the cart (in kg). */
const double cartMass = 0.103;

/** The width of the cart (in m). */
const double cartWidth = 0.035;

/** The maximum voltage of the motor (in V). */
const double motorVoltage = 24.0;

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
Matrix<velocityDataCount, 1, double> velocityDataAngles;

/** The most recent cart positions. Used to calculate the cart's velocity. */
Matrix<velocityDataCount, 1, double> velocityDataPositions;

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

  Serial.begin(38400);
}

void onPendulumAngleAPinChange() {
  pendulumAngle += getAngleChange(pendulumAngleAPin, pendulumAngleBPin);
}

void onMotorAngleAPinChange() {
  motorAngle += getAngleChange(motorAngleAPin, motorAngleBPin);

  // The radius of the timing pulley on the axle of the motor (in m).
  //
  // Multiplying the motor angle by this value gives the cart position.
  const double timingPulleyRadius = 0.00965;
  cartPosition = motorAngle * timingPulleyRadius;
}

double getAngleChange(const uint8_t aPin, const uint8_t bPin) {
  // The rotary encoders run in 9 bit mode, so a change in an A_LSU_U pin
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
  updateVelocities();

  switch (state) {
    case STARTED: {
      // Move the cart until it's pressing the limit switch. We do this inside
      // the case statement because the limit switch normally acts as a kill
      // switch and we temporarily want to disable that behaviour.
      while (digitalRead(limitSwitchPin) == LOW) {
        setMotorSpeed(-10);
      }

      cartPosition = 0.0;
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
      if (cartPosition < centreOfTrack) {
        setMotorSpeed(10);
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
          velocityDataAngles(i, 0) = pendulumAngle;
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

      Matrix<4, 1, double> state = {pendulumAngle, pendulumAngularVelocity, cartPosition, cartVelocity};
      const double f = (-k * state)(0, 0);
      const double voltage = (f / cartMass - d * cartVelocity) / c;
      const short speed = (short)(voltage / motorVoltage * 255.0);
      setMotorSpeed(speed);

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

/**
 * Performs a quadratic regression over the last n data points, differentiates
 * the result, and uses that to predict the current (angular) velocity.
 *
 * See https://en.wikipedia.org/wiki/Polynomial_regression#Matrix_form_and_calculation_of_estimates
 */
void updateVelocities() {
  // Using the notation of the Wikipedia page above, first calculate (X^T X)^-1
  // as it may not be invertible in which case the velocities will be wrong.
  const uint8_t n = velocityDataCount;
  Matrix<n, 3, double> t;
  for (uint8_t i = 0; i < n - 1; i++) {
    t(i, 0) = 1;
    t(i, 1) = velocityDataTimes[i + 1];
    t(i, 2) = pow(velocityDataTimes[i + 1], 2);
  }

  const double now = micros() / 1000000.0;
  t(n - 1, 0) = 1;
  t(n - 1, 1) = now;
  t(n - 1, 2) = pow(now, 2);

  Matrix<3, 3, double> t2 = ~t * t;
  if (!Invert(t2)) {
    // t2 isn't invertible so skip updating the velocities this iteration.
    return;
  }

  for (uint8_t i = 0; i < n - 1; i++) {
    velocityDataAngles(i, 0) = velocityDataAngles(i + 1, 0);
    velocityDataPositions(i, 0) = velocityDataPositions(i + 1, 0);
    velocityDataTimes[i] = velocityDataTimes[i + 1];
  }

  velocityDataAngles(n - 1, 0) = pendulumAngle;
  velocityDataPositions(n - 1, 0) = cartPosition;
  velocityDataTimes[n - 1] = now;

  // Cart velocity
  Matrix<3, 1, double> positionCoefficients = t2 * ~t * velocityDataPositions;
  cartVelocity = positionCoefficients(1, 0) + 2 * positionCoefficients(2, 0) * now;

  // Pendulum angular velocity
  Matrix<3, 1, double> angleCoefficients = t2 * ~t * velocityDataAngles;
  pendulumAngularVelocity = angleCoefficients(1, 0) + 2 * angleCoefficients(2, 0) * now;
}

void setMotorSpeed(const short speed) {
  digitalWrite(motorSpeedDirPin, speed < 0 ? LOW : HIGH);
  analogWrite(motorSpeedPwmPin, min(abs(speed), 50));
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

/** If the pendulum is more than 30° away from verticle, kill it. */
bool checkPendulumAngle() {
  if (abs(pendulumAngle) > 0.524) {
    Serial.println("Pendulum angle too great");
    state = KILLED;
    return false;
  }

  return true;
}
