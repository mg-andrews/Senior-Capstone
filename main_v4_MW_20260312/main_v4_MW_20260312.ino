// Main
// Author: Miles Wilson
// Published: 03/04/2026

// Uses FreeRTOS to schedule serial input, heart rate, pressure simulation, and stepper motor update tasks.
// Added motor pulse on EVENT_HEARTBEAT, limited pulse to pressure range between systole and diastole
// Added serial input for systole/diastole

// Arduino IDE
// Windows 11
// board: esp32s3

#include "KorotkoffAudio.h"
#include "pindef.h"
#include "MotorSetup.h"
#include <stdlib.h>
#include <math.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// Cuff simulation param init
float cuffPressure = 0.0f;
float pulseOffset = 0.0f;
float outputPressure = 0.0f;
float tau = 5.0f;     // seconds (default)
float inflationRate = 50.0f;

volatile float pulseAmplitude = 3.0f;   // small bump
const float pulseLength = 0.05f; // 50 ms timer
volatile float pulseTimer = 0.0f;
volatile bool firstCycle = true;

float systole = 120.0f;
float diastole = 80.0f;
float MAP = diastole + (systole - diastole)/3; // Mean arterial pressure

portMUX_TYPE cuffMux = portMUX_INITIALIZER_UNLOCKED;

// Heart rate queue and timer init
TimerHandle_t heartbeatTimer;
TimerHandle_t LEDOffTimer;

int HR_MIN = 40;
int HR_MAX = 200;
int currentHR = 60;

// Motor init
portMUX_TYPE motorMUX = portMUX_INITIALIZER_UNLOCKED;

// Define queues
QueueHandle_t motor_queue;
static const uint8_t motor_queue_len = 1; // Keep 1 so motor always follows latest position update
QueueHandle_t event_queue;
static const uint8_t event_queue_len = 10;

// Task handles
TaskHandle_t StateManagerHandle = NULL;
TaskHandle_t MotorHandle = NULL;
TaskHandle_t HRHandle = NULL;
TaskHandle_t CuffHandle = NULL;
TaskHandle_t SerialHandle = NULL;
TaskHandle_t AudioHandle = NULL;

// Define system states
typedef enum {
  STATE_SETUP,
  STATE_RUNNING
} SystemState_t;

// Define events
typedef enum {
  EVENT_STARTSIM,
  EVENT_STOPSIM,
  EVENT_HR_UPDATE,
  EVENT_SYSTOLE_UPDATE,
  EVENT_DIASTOLE_UPDATE,
  EVENT_HEARTBEAT
} SystemEventType_t;

// Define event object structure
typedef struct {
  SystemEventType_t type;
  float value;
} SystemEvent_t;

volatile SystemState_t currentState = STATE_SETUP;

//--------------------------------------------------------------------------------------------------
// Helper functions
//--------------------------------------------------------------------------------------------------

// Heartbeat timer param update
void updateHeartRate() {

    int hrPeriod = (60000 / currentHR);

    xTimerChangePeriod(
      heartbeatTimer,
      pdMS_TO_TICKS(hrPeriod),
      portMAX_DELAY
    );

    Serial.print("HR updated to ");
    Serial.print(currentHR);
    Serial.println(" BPM");
}

// Gaussian Scaling Function, returns scaling factor
float gaussian(float P, float mean_pressure)
{
    float sigma = 15.0f;   // tune this

    float exponent = -((P - mean_pressure) * (P - mean_pressure)) / (2.0f * sigma * sigma);

    return expf(exponent);   // returns 0–1 smooth curve
}

//--------------------------------------------------------------------------------------------------
// Timer callbacks
//--------------------------------------------------------------------------------------------------

// Heartbeat timer callback
void heartbeatCallback(TimerHandle_t xTimer) {
  digitalWrite(HRLED, HIGH);
  xTimerStart(LEDOffTimer, 0);

  SystemEvent_t event = {};
  event.type = EVENT_HEARTBEAT;
  xQueueSend(event_queue, &event, portMAX_DELAY);

}

// Turns led off at end of LEDOffTimer
void LEDTimerCallback(TimerHandle_t xTimer) {
  digitalWrite(HRLED, LOW);
}

//--------------------------------------------------------------------------------------------------
// Tasks
//--------------------------------------------------------------------------------------------------

// State manager task
void StateManagerTask(void *parameter) {

  SystemEvent_t event;

  for(;;) {

    if(xQueueReceive(event_queue, &event, portMAX_DELAY)) {

      switch(currentState) {
        case STATE_SETUP:

          if(event.type == EVENT_STARTSIM) {

            Serial.println("State swapped: (SETUP --> RUNNING)");

            updateHeartRate();
            xTimerStart(heartbeatTimer, 0);

            firstCycle = true;
            vTaskResume(CuffHandle);
            vTaskResume(MotorHandle);
            vTaskResume(AudioHandle);

            currentState = STATE_RUNNING;

          } else if(event.type == EVENT_HR_UPDATE) {
            
            currentHR = (int)event.value;
            Serial.print("HR set to ");
            Serial.print(currentHR);
            Serial.println(" BPM");
          }

          break;

        case STATE_RUNNING:

          if(event.type == EVENT_STOPSIM) {

            Serial.println("State swapped: (RUNNING --> SETUP)");

            xTimerStop(heartbeatTimer, 0);
            vTaskSuspend(CuffHandle);
            vTaskSuspend(AudioHandle);

            // Sets sim pressures to 0 while blocking any outside updates
            portENTER_CRITICAL(&cuffMux);
            cuffPressure = 0;
            pulseOffset = 0;
            outputPressure = 0;
            portEXIT_CRITICAL(&cuffMux);

            // Zero motor on simulation stop
            float zero = 0.0f;
            xQueueOverwrite(motor_queue, &zero);
            xTaskNotify(MotorHandle, 0, eNoAction);

            // 3-second timeout before motor zeroing is cancelled
            TickType_t startTime = xTaskGetTickCount();
            while (motor_get_position() != 0) {
              if (xTaskGetTickCount() - startTime > pdMS_TO_TICKS(3000)) {
                Serial.println("Warning: Motor zero timeout");
                break;
              }
              vTaskDelay(pdMS_TO_TICKS(10));
            }

            Serial.println("Motor zeroed");
            vTaskSuspend(MotorHandle);

            currentState = STATE_SETUP;

          } else if(event.type == EVENT_HEARTBEAT) {

              // Logic for korotkoff sounds and motor pulse on heartbeat
              xTaskNotifyGive(CuffHandle);
              xTaskNotifyGive(AudioHandle);
              Serial.println("beat send");

          } else if(event.type == EVENT_HR_UPDATE) {

              currentHR = (int)event.value;
              updateHeartRate();
          }

          break;

      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Wait for serial parameter input and put in queue
void SerialTask(void *parameter) {

  for(;;) {

    if(Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      SystemEvent_t event ={};

      if(input == "start") {

        event.type = EVENT_STARTSIM;
        xQueueSend(event_queue, &event, portMAX_DELAY);

      } else if(input == "stop") {

        event.type = EVENT_STOPSIM;
        xQueueSend(event_queue, &event, portMAX_DELAY);

      } else if(input.startsWith("hr ")) {

        float hrInput = input.substring(3).toFloat();
        if(hrInput >= HR_MIN && hrInput <= HR_MAX) {
          event.type = EVENT_HR_UPDATE;
          event.value = hrInput;
          xQueueSend(event_queue, &event, portMAX_DELAY);

        } else {

          Serial.print("HR input out of range. HR must be between ");
          Serial.print(HR_MIN);
          Serial.print(" and ");
          Serial.print(HR_MAX);
          Serial.println(" BPM");
        }
        
      } else if(input.startsWith("sys ")) {   // Added systole input command

        systole = input.substring(4).toFloat();
        /*
        event.type = EVENT_SYSTOLE_UPDATE;
        event.value = systole;
        xQueueSend(event_queue, &event, portMAX_DELAY);
        */

        Serial.print("Systole set to ");
        Serial.print(systole);
        Serial.println(" mmHg");

      } else if(input.startsWith("dia ")) {   // Added diastole input command

        diastole = input.substring(4).toFloat();
        /*
        event.type = EVENT_DIASTOLE_UPDATE;
        event.value = diastole;
        xQueueSend(event_queue, &event, portMAX_DELAY);
        */

        Serial.print("Diastole set to ");
        Serial.print(diastole);
        Serial.println(" mmHg");

      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Check for input every 50 ms
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Play Korotkoff audio
void AudioTask(void *parameter) {

  for(;;) {
    if(cuffPressure < systole && cuffPressure > diastole) {
      if(ulTaskNotifyTake(pdTRUE, 0)) {
        start_beat();
      }
    }
    audio_process();
  }
}



// TODO: currently beat audio may play twice when cuffPressure enters/exits the systole/diastole range due to queued EVENT_HEARTBEAT in event_queue
// FIX(?): create separate heartbeat_queue with length of 1 to ensure ulTaskNotifyTake can only take one event per beat


//--------------------------------------------------------------------------------------------------
// Task: Update cuff pressure model
void CuffTask(void *parameter) {
  const float dt = 0.005f;   // 5 ms --> 200 Hz update
  TickType_t lastWakeTime = xTaskGetTickCount();
  firstCycle = true;

  for(;;) {
    if(firstCycle) {
      lastWakeTime = xTaskGetTickCount();
      firstCycle = false;
    }

    // Read potentiometer
    int potValue = analogRead(POT_PIN);

    // Map to tau range (1s to 40s)
    tau = 1.0f + (potValue / 4095.0f) * 39.0f;

    // Read button to input pressure in system
    if(!digitalRead(BUTTON_PIN)) {
      cuffPressure += inflationRate * dt;   // Cuff inflation rate
      if (cuffPressure > 360.0f)
        cuffPressure = 360.0f;
    }

    // Exponential decay model
    cuffPressure *= expf(-dt / tau);    

    // Start pulse timer if EVENT_HEARTBEAT occurs
    if(ulTaskNotifyTake(pdTRUE, 0) > 0) {
      if(cuffPressure < systole && cuffPressure > diastole) {

        pulseTimer = pulseLength;
      }
    }

    // Calculate pulseOffset if pulse timer is active and decrement pulseTimer until 0
    if(pulseTimer > 0.0f) {

      pulseOffset = gaussian(cuffPressure, MAP);
      pulseTimer -= dt;

    } else if(pulseTimer <= 0.0f) {

      pulseTimer = 0.0f;
      pulseOffset = 0.0f;

    }

    outputPressure = cuffPressure + (pulseAmplitude * pulseOffset);

    // Clamp small values
    if(outputPressure < 0.1f)
      outputPressure = 0.0f;

    xQueueOverwrite(motor_queue, &outputPressure);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(dt * 1000));
  }
}

//--------------------------------------------------------------------------------------------------
// Task: Update stepper position based on pressure in motor_queue
void MotorTask(void *parameter) {
  float pressure;
  int lastPosition = 0;

  for(;;) {

    if(xTaskNotifyWait(0, 0, NULL, 0) == pdTRUE) {
      
      motor_move_to(0);
      lastPosition = 0;
    }

    if (xQueueReceive(motor_queue, &pressure, portMAX_DELAY) == pdTRUE) {
      // Convert pressure to motor position
      int targetPosition = (int)((pressure / 360.0f) * motor_steps_per_rev());

      if (abs(targetPosition - lastPosition) > motor_deadband()) {
        motor_move_to(targetPosition);
        lastPosition = targetPosition;
      }
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Setup

void setup() {
  
  Serial.begin(115200);
  delay(500 / portTICK_PERIOD_MS);

  // Cuff pressure setup
  pinMode(V_PLUS, OUTPUT);
  pinMode(VIO_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(HRLED, OUTPUT);

  digitalWrite(V_PLUS, HIGH); // Sets input of pot to 3.3V
  digitalWrite(VIO_PIN, HIGH); // Sets input to VIO on driver to 3.3V

  // Stepper driver setup
  setup_motor();

  // Audio setup
  setup_i2s();

  // Init instruction messages
  Serial.println("       ...");
  Serial.println("Setup initialized");
  Serial.println("       ...");
  Serial.println("Input heart rate by typing 'hr [BPM]'");
  Serial.println("Input systole by typing 'sys [mmHg]'");
  Serial.println("Input diastole by typing 'dia [mmHg]'\n");
  Serial.println("Start simulation by typing 'start'");
  Serial.println("Suspend simulation by typing 'stop'\n");

  /*
  Serial.print("Initial heart rate set to ");
  Serial.print(currentHR);
  Serial.println(" BPM");
  Serial.print("Initial systole set to ");
  Serial.print(systole);
  Serial.println(" mmHg");
  Serial.print("Initial diastole set to ");
  Serial.print(diastole);
  Serial.println(" mmHg");
  */

  // Create queues
  motor_queue = xQueueCreate(motor_queue_len, sizeof(float));
  event_queue = xQueueCreate(event_queue_len, sizeof(SystemEvent_t));

  // Create software timers
  heartbeatTimer = xTimerCreate(
    "HeartbeatTimer",
    pdMS_TO_TICKS(500),
    pdTRUE, // Retriggers on end
    NULL,
    heartbeatCallback
  );

  LEDOffTimer = xTimerCreate(
    "LEDOffTimer",
    pdMS_TO_TICKS(100),
    pdFALSE, // One-shot
    NULL,
    LEDTimerCallback
  );

  // Create task instances
  xTaskCreatePinnedToCore(
    StateManagerTask,     // Task function
    "StateManagerTask",   // Task name
    4096,           // Stack size in bytes
    NULL,           // Parameters
    2,              // Priority
    &StateManagerHandle,  // Task handle
    1   
  );

  xTaskCreatePinnedToCore(
    SerialTask,     // Task function
    "SerialTask",   // Task name
    4096,           // Stack size in bytes
    NULL,           // Parameters
    1,              // Priority
    &SerialHandle,  // Task handle
    1               // Run on core 1
  );

  xTaskCreatePinnedToCore(
    AudioTask,     // Task function
    "AudioTask",   // Task name
    8192,         // Stack size
    NULL,         // Parameters
    1,            // Priority
    &AudioHandle,  // Task handle
    0             // Run on core 1
  );

  vTaskSuspend(AudioHandle);

  xTaskCreatePinnedToCore(
    CuffTask,     // Task function
    "CuffTask",   // Task name
    2048,         // Stack size
    NULL,         // Parameters
    4,            // Priority
    &CuffHandle,  // Task handle
    1             // Run on core 1
  );

  vTaskSuspend(CuffHandle);

  xTaskCreatePinnedToCore(
    MotorTask,        // Task function
    "MotorTask",      // Task name
    4096,             // Stack size in bytes
    NULL,             // Parameters
    4,                // Priority
    &MotorHandle,     // Task handle
    1                 // Run on core 1
  );

  vTaskSuspend(MotorHandle);
}

void loop() {
}