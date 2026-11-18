// --- Controller Parameters ---
float Kp = 2.0;
float Ki = 0.01;
float Kd = 3.5;
float integralError = 0.0;
float lastError = 0.0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT); // Sends Control Signal (-100%..+100%) to Plant Arduino
  lastTime = millis();
}

float computeController(float targetAngle, float measuredAngle, float dt) {
  float error = targetAngle - measuredAngle;
  integralError = constrain(integralError + error * dt, -100.0, 100.0);
  float derivative = (error - lastError) / dt;
  lastError = error;

  float rawOutput = (Kp * error) + (Ki * integralError) + (Kd * derivative);
  return constrain(rawOutput, -100.0, 100.0);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    // 1. Read Pilot Knob (A0) -> Target Angle (-90 to +90)
    float targetAngle = ((analogRead(A0) / 1024.0) * 180.0) - 90.0;

    // 2. Read Machine Angle from Plant Arduino (A1) -> Measured Angle (-90 to +90)
    float measuredAngle = ((analogRead(A1) / 1024.0) * 180.0) - 90.0;

    // 3. Compute PID Control Signal (-100% to +100%)
    float controlSignal = computeController(targetAngle, measuredAngle, dt);

    // 4. Send Control Signal out as PWM (0..255) on Pin 5
    int pwmOut = (int)(((controlSignal + 100.0) / 200.0) * 255.0);
    analogWrite(5, constrain(pwmOut, 0, 255));

    // 5. Print numbers only for Tinkercad Serial Plotter (Target, Control, Angle)
    Serial.print(targetAngle);
    Serial.print(",");
    Serial.print(controlSignal);
    Serial.print(",");
    Serial.println(measuredAngle);
  }
}
