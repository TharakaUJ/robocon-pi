// Motor control pins (IN1, IN2 only)
const int motorPins[4][2] = {
  {21, 19},  // Front Left
  {33, 32},  // Front Right
  {23, 22},  // Rear Left 
  {26, 25}   // Rear Right
};

const int builtinLed = 2;

void setup() {
  Serial.begin(115200);

  // Initialize motor pins and LED
  for (int i = 0; i < 4; ++i) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
  }
  pinMode(builtinLed, OUTPUT);
  digitalWrite(builtinLed, LOW);

  // LED test sequence
  for (int i = 0; i < 3; ++i) {
    digitalWrite(builtinLed, HIGH); delay(200);
    digitalWrite(builtinLed, LOW);  delay(200);
  }
  Serial.println("System Ready - Send MOVE commands");
}

void loop() {
  if (Serial.available() >= 12) { // 3 floats * 4 bytes
    float vx, vy, theta;
    byte buffer[12];
    Serial.readBytes(buffer, 12);
    memcpy(&vx, buffer, 4);
    memcpy(&vy, buffer + 4, 4);
    memcpy(&theta, buffer + 8, 4);

    int values[4] = {
      constrain((int)(vx + vy + theta), -255, 255), // FL
      constrain((int)(vx - vy - theta), -255, 255), // FR
      constrain((int)(vx - vy + theta), -255, 255), // RL
      constrain((int)(vx + vy - theta), -255, 255)  // RR
    };

    processMovement(values);
  }
}

void processMovement(const int values[4]) {
  digitalWrite(builtinLed, HIGH);

  // Debug output
  Serial.print("Motors: ");
  bool allForward = true;
  for (int i = 0; i < 4; ++i) {
    Serial.print(values[i]); Serial.print(" ");
    if (values[i] <= 10) allForward = false;
    controlMotor(motorPins[i], values[i]);
  }
  Serial.print("| LED: "); Serial.println(allForward ? "ON" : "OFF");

  digitalWrite(builtinLed, allForward ? HIGH : LOW);
  delay(10);
  digitalWrite(builtinLed, LOW);
}

void controlMotor(const int pins[2], int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 10) {
    digitalWrite(pins[0], HIGH); digitalWrite(pins[1], LOW);
  } else if (speed < -10) {
    digitalWrite(pins[0], LOW); digitalWrite(pins[1], HIGH);
  } else {
    digitalWrite(pins[0], LOW); digitalWrite(pins[1], LOW);
  }
}
