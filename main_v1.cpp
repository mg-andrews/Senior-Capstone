// Four-Element Windkessel BP Model to PWM Output
// Author: Miles Wilson
// Published: 11/5/2025
// Validated: 

// Numerically solves a four-element windkessel model for blood pressure over time using RK4, and outputs pressure mapped to [0,3.3V]
// External buttons to start/stop PMW waveform playback and LEDs to indicate playback state

// PlatformIO via Visual Studio Code
// [env:freenove_esp32_s3_wroom]
// platform = espressif32
// board = freenove_esp32_s3_wroom
// framework = arduino
// monitor_speed = 115200

#include <Arduino.h>

// PWM setup
const int pwmPin = 9; // PMW out pin
const int pwmChannel = 0;
const int pwmFreq = 10000; // 10 kHz PWM frequency
const int pwmResolution = 10; // 10-bit resolution (0-1023)

// Button pins
// Pull-up resistors used, tie buttons to ground
const int startButtonPin = 2; // Button to start waveform output
const int stopButtonPin = 3; // Button to stop waveform output

// LED pin
const int statusOnPin = 4; // Green LED
const int statusOffPin = 15; // Red LED

// Control state
bool isRunning = false;

// Windkessel parameters
const double I0 = 500.0; // Do not change
const double Tc = 60.0 / 72.0; // Cardiac period (sec)
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
const double maxPressure = 180.0;

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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 Windkessel Model -> PWM Output");
  Serial.println("Press START button to begin, STOP button to halt");
  
  // Setup buttons with internal pull-up resistors
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  
  // Setup status LED
  pinMode(statusOnPin, OUTPUT);
  digitalWrite(statusOnPin, LOW);
  pinMode(statusOffPin, OUTPUT);
  digitalWrite(statusOffPin, HIGH);
  
  // Setup LEDC (hardware PWM)
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  // Start with PWM off
  ledcWrite(pwmChannel, 0);
}

void loop() {
  // Check start button (active LOW with pullup)
  if (digitalRead(startButtonPin) == LOW && !isRunning) {
    isRunning = true;
    // Reset simulation state
    t = 0.0;
    pressure = 80.0;
    pressureDot = 0.0;
    digitalWrite(statusOnPin, HIGH);
    digitalWrite(statusOffPin, LOW);
    Serial.println("Waveform STARTED");
    delay(200);  // Debounce delay
  }
  
  // Check stop button (active LOW with pullup)
  if (digitalRead(stopButtonPin) == LOW && isRunning) {
    isRunning = false;
    ledcWrite(pwmChannel, 0);  // Turn off PWM
    digitalWrite(statusOnPin, LOW);
    digitalWrite(statusOffPin, HIGH);
    Serial.println("Waveform STOPPED");
    delay(200);  // Debounce delay
    return;  // Skip rest of loop
  }
  
  // Only run simulation if started
  if (!isRunning) {
    delay(10);  // Small delay to prevent busy-waiting
    return;
  }
  
  // Compute derivatives
  double It = I(t);
  double Itdot = Idot(t);
  double Itdotdot = Idotdot(t);
  
  // RK4 integration
  // k1
  double k1_p = pressureDot;
  double k1_pdot = (Itdotdot * (R * L * C * R1)
                   + Itdot * (L * (R + R1))
                   + It * (R * R1)
                   - (pressureDot * (C * R * R1 + L) + pressure * R1))
                   / (L * C * R);
  
  // k2
  double t2 = t + dt/2;
  double p_temp = pressure + k1_p * dt/2;
  double pdot_temp = pressureDot + k1_pdot * dt/2;
  double It2 = I(t2);
  double Itdot2 = Idot(t2);
  double Itdotdot2 = Idotdot(t2);
  double k2_p = pdot_temp;
  double k2_pdot = (Itdotdot2 * (R * L * C * R1)
                   + Itdot2 * (L * (R + R1))
                   + It2 * (R * R1)
                   - (pdot_temp * (C * R * R1 + L) + p_temp * R1))
                   / (L * C * R);
  
  // k3
  p_temp = pressure + k2_p * dt/2;
  pdot_temp = pressureDot + k2_pdot * dt/2;
  double k3_p = pdot_temp;
  double k3_pdot = (Itdotdot2 * (R * L * C * R1)
                   + Itdot2 * (L * (R + R1))
                   + It2 * (R * R1)
                   - (pdot_temp * (C * R * R1 + L) + p_temp * R1))
                   / (L * C * R);
  
  // k4
  double t4 = t + dt;
  p_temp = pressure + k3_p * dt;
  pdot_temp = pressureDot + k3_pdot * dt;
  double It4 = I(t4);
  double Itdot4 = Idot(t4);
  double Itdotdot4 = Idotdot(t4);
  double k4_p = pdot_temp;
  double k4_pdot = (Itdotdot4 * (R * L * C * R1)
                   + Itdot4 * (L * (R + R1))
                   + It4 * (R * R1)
                   - (pdot_temp * (C * R * R1 + L) + p_temp * R1))
                   / (L * C * R);
  
  // Update pressure
  pressure += (k1_p + 2*k2_p + 2*k3_p + k4_p) * dt / 6.0;
  pressureDot += (k1_pdot + 2*k2_pdot + 2*k3_pdot + k4_pdot) * dt / 6.0;
  t += dt;
  
  // Clamp pressure
  if (pressure < 0) pressure = 0;
  if (pressure > 200) pressure = 200;
  
  // Map pressure to PWM duty (0–1023) - manual mapping for doubles
  int duty = (int)((pressure - minPressure) / (maxPressure - minPressure) * 1023.0);
  duty = constrain(duty, 0, 1023);
  
  ledcWrite(pwmChannel, duty); // Update duty cycle
  
  // Output "t[s], BP[mmHg]"
  Serial.print(t, 3);
  Serial.print(" , ");
  Serial.println(pressure, 2);
  
  delayMicroseconds(dt * 1e6); // Output updates every 0.001s
}