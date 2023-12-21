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

/**
 * Whether the system has been killed.
 *
 * When the system is killed the motor turns off and the system must be
 * restarted. This happens if, e.g., the cart gets too close to an end.
 */
bool killed = false;

/** The last measured angle of the pendulum (in radians). */
volatile double pendulumAngle = 0.0;

/** The last measured angle of the motor (in radians). */
volatile double motorAngle = 0.0;

/** The last measured distance between the cart and the limit switch (in m). */
volatile double cartPosition = 0.0;

const double pi = 3.14159265;

/** The length of the track (in m). */
const double trackLength = 0.965;

/** The width of the cart (in m). */
const double cartWidth = 0.035;

/**
 * The system's configuration state.
 *
 * The system starts UNCONFIGURED, moves the cart until the limit switch is
 * pressed and the motor angle can be ZEROED, then moves the cart to the centre
 * of the track at which point it is CONFIGURED.
 */
enum ConfigurationState {
  UNCONFIGURED,
  ZEROED,
  CONFIGURED
} configurationState = UNCONFIGURED;

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
  // Because of the way the motor's rotary encoder is mounted (facing away from
  // the front of the system), the angles are opposite what you would expect.
  // Invert them to make things a bit easier.
  motorAngle -= getAngleChange(motorAngleAPin, motorAngleBPin);

  // The radius of the timing pulley on the axle of the motor (in m).
  // Multiplying the motor angle by this value gives the distance between the
  // timing pulley and the cart.
  const double timingPulleyRadius = 0.00965;
  cartPosition = motorAngle * timingPulleyRadius;
}

double getAngleChange(const uint8_t aPin, const uint8_t bPin) {
  // The rotary encoders run in 9 bit mode, so a change in a A_LSU_U pin
  // corresponds to a change of 0.704° or 0.0123 radians.
  const double resolution = 0.0123;

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
    digitalWrite(motorSpeedPwmPin, LOW);
    return;
  }

  // The rotary encoder magnets must be in range to do anything. Check them first.
  if (!isMagnetInRange(pendulumAngleMagIncPin, pendulumAngleMagDecPin)) {
    killed = true;
    Serial.println("Pendulum magnet not in range");
    return;
  }

  if (!isMagnetInRange(motorAngleMagIncPin, motorAngleMagDecPin)) {
    killed = true;
    Serial.println("Motor magnet not in range");
    return;
  }

  // We use the limit switch to zero the motor angle, so run that step of the
  // configuration before using the limit switch as a kill switch.
  if (configurationState == UNCONFIGURED) {
    configure();
    return;
  }

  // The motor angle has been zeroed. Now use the limit switch as a kill switch.
  if (digitalRead(limitSwitchPin) == HIGH) {
    killed = true;
    Serial.println("Kill switch pressed");
    return;
  }

  if (configurationState != CONFIGURED) {
    configure();
    return;
  }

  // If the cart is too close to either end of the track, kill it.
  const double buffer = 0.1;
  if (cartPosition < buffer || cartPosition > trackLength - cartWidth - buffer) {
    killed = true;
    Serial.println("Cart too close to end");
    return;
  }
}

bool isMagnetInRange(const uint8_t magIncPin, const uint8_t magDecPin) {
  // If the magnet is in range both MagINCn and MagDECn are turned off. As they
  // are open drain and there's a pull up resistor we'll see the pins as high.
  return digitalRead(magIncPin) == HIGH && digitalRead(magDecPin) == HIGH;
}

void configure() {
  switch (configurationState) {
    case UNCONFIGURED:
      if (digitalRead(limitSwitchPin) == LOW) {
        // Move the cart until it's pressing the limit switch
        setMotorSpeed(-10);
      } else {
        motorAngle = 0.0;

        // Move the cart until it's not pressing the limit switch
        while (digitalRead(limitSwitchPin) == HIGH) {
          setMotorSpeed(10);
          delay(100);
        }

        configurationState = ZEROED;
      }
      break;

    case ZEROED:
      // Move the cart to the middle of the track
      const double target = (trackLength - cartWidth) / 2.0;
      if (cartPosition < target) {
        setMotorSpeed(10);
      } else {
        setMotorSpeed(0);
        configurationState = CONFIGURED;
      }
      break;
    
    case CONFIGURED:
      killed = true;
      Serial.println("Unexpected control flow");
      break;
  }
}

void setMotorSpeed(const short speed) {
  digitalWrite(motorSpeedDirPin, speed < 0 ? LOW : HIGH);
  analogWrite(motorSpeedPwmPin, min(abs(speed), 10));
}
