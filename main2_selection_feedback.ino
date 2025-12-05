// Four-Element Windkessel BP Model to PWM Output
// Author: Miles Wilson
// Written: 11/5/2025
// Validated: 

// Numerically solves a four-element windkessel model for blood pressure over time using RK4, and outputs pressure mapped to [0,3.3V]
// External buttons to start/stop PMW waveform playback and LEDs to indicate playback state

// PlatformIO via Visual Studio Code
// [env:freenove_esp32_s3_wroom]
// platform = espressif32
// board = freenove_esp32_s3_wroom
// framework = arduino
// monitor_speed = 115200

#include <math.h>

// PWM setup
const int pwmPin = 9; // PMW out pin
const int pwmChannel = 0;
const int pwmFreq = 10000; // 10 kHz PWM frequency
const int pwmResolution = 10; // 10-bit resolution (0-1023)

// Button main
// Pull-up resistors used, tie buttons to ground
const int W1_PIN = 2; // Button to start waveform output
const int W2_PIN = 3; // Button to stop waveform output
const int D1_PIN = 5; // Diagnose W1
const int D2_PIN = 6; // Diagnose W2
const int RESET_PIN = 41; // Reset simulation

// LED pin
const int trueDiagPin = 4; // Green LED
const int falseDiagPin = 15; // Red LED

// Control state
int W_state = 0;
int D_state = 2;

// Windkessel parameters init
double I0 = 425.0; // BP range
double Tc = 60.0 / 72.0; // Cardiac period (sec)
const double Ts = (2.0 / 5.0) * Tc;
const double R = 1.0;
const double C = 1.0;
const double R1 = 0.05;
const double L = 0.005;

// Time init
const double dt = 0.001;  // 1 ms timestep
double t = 0.0;

// Pressure init
double pressure = 80.0;     // Pressure out
double pressureDot = 0.0;   // d(pressure)/dt

// Pressure range for mapping (0V to minPressure, 3.3V to maxPressure)
const double minPressure = 50.0;
const double maxPressure = 150.0;

// Define diffeq system
double I(double t) {
  double modt = fmod(t, Tc);
  return (modt <= Ts) ? I0 * pow(sin(PI * modt / Ts), 2) : 0.0;
}

double Idot(double t) {
  double modt = fmod(t, Tc);
  return (modt <= Ts) ? I0 * 2 * sin(PI * modt / Ts) * cos(PI * modt / Ts) * PI / Ts : 0.0;
}

double Idotdot(double t) {
  double modt = fmod(t, Tc);
  if (modt > Ts) return 0.0;
  return I0 * 2 * (pow(cos(PI * modt / Ts), 2) - pow(sin(PI * modt / Ts), 2)) * pow(PI / Ts, 2);
}

void RK4 (double t) {
  // Compute derivatives
  double It = I(t);
  double Itdot = Idot(t);
  double Itdotdot = Idotdot(t);
  
  // RK4 integration
  // k1
  double k1_p = pressureDot;
  double k1_pdot = (Itdotdot * (R * L * C * R1) + Itdot * (L * (R + R1)) + It * (R * R1) - (pressureDot * (C * R * R1 + L) + pressure * R1)) / (L * C * R);
  
  // k2
  double t2 = t + dt/2;
  double p_temp = pressure + k1_p * dt/2;
  double pdot_temp = pressureDot + k1_pdot * dt/2;
  double It2 = I(t2);
  double Itdot2 = Idot(t2);
  double Itdotdot2 = Idotdot(t2);
  double k2_p = pdot_temp;
  double k2_pdot = (Itdotdot2 * (R * L * C * R1) + Itdot2 * (L * (R + R1)) + It2 * (R * R1) - (pdot_temp * (C * R * R1 + L) + p_temp * R1)) / (L * C * R);
  
  // k3
  p_temp = pressure + k2_p * dt/2;
  pdot_temp = pressureDot + k2_pdot * dt/2;
  double k3_p = pdot_temp;
  double k3_pdot = (Itdotdot2 * (R * L * C * R1) + Itdot2 * (L * (R + R1)) + It2 * (R * R1) - (pdot_temp * (C * R * R1 + L) + p_temp * R1)) / (L * C * R);
  
  // k4
  double t4 = t + dt;
  p_temp = pressure + k3_p * dt;
  pdot_temp = pressureDot + k3_pdot * dt;
  double It4 = I(t4);
  double Itdot4 = Idot(t4);
  double Itdotdot4 = Idotdot(t4);
  double k4_p = pdot_temp;
  double k4_pdot = (Itdotdot4 * (R * L * C * R1) + Itdot4 * (L * (R + R1)) + It4 * (R * R1) - (pdot_temp * (C * R * R1 + L) + p_temp * R1)) / (L * C * R);
  
  // Update pressure
  pressure += (k1_p + 2*k2_p + 2*k3_p + k4_p) * dt / 6.0;
  pressureDot += (k1_pdot + 2*k2_pdot + 2*k3_pdot + k4_pdot) * dt / 6.0;
}

void setup() {
  // Initialize USB serial FIRST
  #if ARDUINO_USB_CDC_ON_BOOT
    Serial.begin();  // No baud rate needed for USB CDC
    while (!Serial && millis() < 5000) {
      delay(10);  // Wait up to 5 seconds for serial
    }
  #else
    Serial.begin(115200);
    delay(2000);  // Give UART time to initialize
  #endif
  
  Serial.println("=== ESP32-S3 BOOT ===");
  Serial.println("Setup complete");
  Serial.println("ESP32-S3 Windkessel Model -> PWM Output");
  Serial.println("Press START button to begin, STOP button to halt");
  
  // Setup buttons with internal pull-up resistors
  pinMode(W1_PIN, INPUT_PULLUP);
  pinMode(W2_PIN, INPUT_PULLUP);
  pinMode(D1_PIN, INPUT_PULLUP);
  pinMode(D2_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  
  // Setup status LED
  pinMode(trueDiagPin, OUTPUT);
  digitalWrite(trueDiagPin, LOW);
  pinMode(falseDiagPin, OUTPUT);
  digitalWrite(falseDiagPin, LOW);
  
  // Setup LEDC (hardware PWM) - NEW API
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  
  // Start with PWM off
  ledcWrite(pwmPin, 0);
}

void loop() {
  // Reset simulation
  if (digitalRead(RESET_PIN) == LOW) {
    W_state = 2;
    D_state = 2;
    digitalWrite(trueDiagPin, LOW);
    digitalWrite(falseDiagPin, LOW);
    ledcWrite(pwmPin, 0);
    Serial.println("System RESET");
    delay(200);
  }

  // Check W1 button (active LOW with pullup)
  if (digitalRead(W1_PIN) == LOW && W_state != 0) {
    W_state = 0;
    I0 = 425;
    pressure = 80.0;
    pressureDot = 0.0;
    t = 0.0; // Reset time
    Serial.println("Waveform 1 STARTED");
    delay(200);  // Debounce delay
  }
  // Check W2 button (active LOW with pullup)
  if (digitalRead(W2_PIN) == LOW && W_state != 1) {
    W_state = 1;
    I0 = 500;
    pressure = 80.0;
    pressureDot = 0.0;
    t = 0.0; // Reset time
    Serial.println("Waveform 2 STARTED");
    delay(200);  // Debounce delay
  }

  // Diagnosis logic
  if (digitalRead(D1_PIN) == LOW) {
    D_state = 0;
    delay(200);
  }
  if (digitalRead(D2_PIN) == LOW) {
    D_state = 1;
    delay(200);
  }

  if (W_state == D_state && D_state != 2) {
    digitalWrite(trueDiagPin, HIGH);
    digitalWrite(falseDiagPin, LOW);
    Serial.println("Correct Diagnosis!");
    delay(2000); // Show result
    D_state = 2; // Reset diagnosis
    digitalWrite(trueDiagPin, LOW);
  }
  else if (W_state != D_state && D_state != 2) {
    digitalWrite(falseDiagPin, HIGH);
    digitalWrite(trueDiagPin, LOW);
    Serial.println("Incorrect Diagnosis :(");
    delay(2000); // Show result
    D_state = 2; // Reset diagnosis
    digitalWrite(falseDiagPin, LOW);
  }

  // Calculate new pressure
  RK4(t);

  // Clamp pressure
  if (pressure < 0) pressure = 0;
  if (pressure > 150) pressure = 150;
  
  // Map pressure to PWM duty (0–1023) - manual mapping for doubles
  int duty = (int)((pressure - minPressure) / (maxPressure - minPressure) * 1023.0);
  duty = constrain(duty, 0, 1023);

  ledcWrite(pwmPin, duty); // Update duty cycle
  
  // Print every 10ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 10) {  // Print every 10ms
    Serial.print(t, 3);
    Serial.print(" , ");
    Serial.println(pressure, 2);
  lastPrint = millis();
}

  t += dt;

  delayMicroseconds(dt * 1e6);
}