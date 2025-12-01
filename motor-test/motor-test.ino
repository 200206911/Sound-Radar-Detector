#include <Servo.h>

Servo myServo;

const int servoPin = 9;

// Servo PWM values
const int pwmStop = 90;
const int pwmForwardFast = 0;
const int pwmBackwardFast = 180;
const int pwmForwardSlow = 20;
const int pwmBackwardSlow = 160;

float currentAngle = 0;       // current approximate angle
const float maxSpeedDegPerSec = 700.0; // measured full-speed rotation
const float slowSpeedDegPerSec = 120.0; // slower speed for small moves

void setup() {
  myServo.attach(servoPin);
  myServo.write(pwmStop);
  Serial.begin(9600);
}

void loop() {
  // Example usage
  rotateToAngle(90);
  delay(2000);

  rotateToAngle(180);
  delay(2000);

  rotateToAngle(270);
  delay(2000);

  rotateToAngle(0);
  delay(2000);
}

// Rotate to approximate angle
void rotateToAngle(float targetAngle) {
  targetAngle = fmod(targetAngle, 360.0); // wrap around 360
  float delta = targetAngle - currentAngle;

  // Shortest rotation direction
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;

  int pwm;
  float speedDegPerSec;

  if (delta > 0) {
    // Forward
    if (delta < 60) { // small move → slow
      pwm = pwmForwardSlow;
      speedDegPerSec = slowSpeedDegPerSec;
    } else {          // big move → fast
      pwm = pwmForwardFast;
      speedDegPerSec = maxSpeedDegPerSec;
    }
  } else if (delta < 0) {
    // Backward
    delta = -delta; // make delta positive for timing
    if (delta < 60) { // small move → slow
      pwm = pwmBackwardSlow;
      speedDegPerSec = slowSpeedDegPerSec;
    } else {          // big move → fast
      pwm = pwmBackwardFast;
      speedDegPerSec = maxSpeedDegPerSec;
    }
  } else {
    return; // already at target
  }

  // Start rotation
  myServo.write(pwm);

  // Calculate approximate time to rotate
  unsigned long rotationTime = (unsigned long)((delta / speedDegPerSec) * 1000);
  delay(rotationTime);

  // Stop servo
  myServo.write(pwmStop);

  // Update current angle
  currentAngle = targetAngle;
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}
