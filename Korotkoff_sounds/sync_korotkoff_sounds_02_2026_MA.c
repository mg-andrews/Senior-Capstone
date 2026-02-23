// Author: Marissa Andrews
// Date: 02-20-2026
// Description: This program synchronizes the Korotkoff sounds for blood pressure measurement. It reads the audio data from a file, processes it to identify the Korotkoff sounds, and outputs the results.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>





//  EXTERNAL VARIABLES (defined elsewhere)
// These variables must exist in another file (your BP model + cuff model)

extern double pressure;        // Current arterial pressure (mmHg)
extern double cuffPressure;    // Current cuff pressure (mmHg)
extern double systolic;        // Current detected systolic pressure
extern double diastolic;       // Current detected diastolic pressure
extern bool isDeflating;       // True only when cuff is deflating


//  CONFIGURATION DEFINES

// GPIO pin used for PWM audio output
#define AUDIO_GPIO          4       

// PWM carrier frequency (40 kHz so it is ultrasonic and filtered to audio)
#define PWM_FREQUENCY       40000   

// PWM resolution (10-bit → 0–1023 duty range)
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT

// Maximum duty value for 10-bit PWM
#define PWM_MAX_DUTY        1023

// How long each Korotkoff beat lasts (30 ms)
#define BEAT_DURATION_US    30000   


//  INTERNAL STATE VARIABLES 

// Stores previous arterial pressure to detect crossing event
static double prevPressure = 0;

// True while a beat is currently playing
static bool beatActive = false;

// Timestamp (microseconds) when beat started
static int64_t beatStartTime = 0;


// INITIALIZE PWM AUDIO HARDWARE
void korotkoff_audio_init()
{
    // Configure the PWM timer (sets frequency and resolution)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,   // Use low-speed PWM hardware
        .timer_num        = LEDC_TIMER_0,          // Use timer 0
        .duty_resolution  = PWM_RESOLUTION,        // 10-bit resolution
        .freq_hz          = PWM_FREQUENCY,         // 40 kHz carrier
        .clk_cfg          = LEDC_AUTO_CLK          // Auto select clock source
    };

    // Apply timer configuration
    ledc_timer_config(&ledc_timer);

    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = AUDIO_GPIO,              // Output pin
        .speed_mode     = LEDC_LOW_SPEED_MODE,     // Must match timer mode
        .channel        = LEDC_CHANNEL_0,          // Use channel 0
        .intr_type      = LEDC_INTR_DISABLE,       // No interrupts
        .timer_sel      = LEDC_TIMER_0,            // Use timer 0
        .duty           = 0,                       // Start at 0 duty (silent)
        .hpoint         = 0                        // PWM start point
    };

    // Apply channel configuration
    ledc_channel_config(&ledc_channel);
}



// SET PWM AMPLITUDE (volume control)

static void set_pwm_amplitude(double amplitude)
{
    // Clamp amplitude between 5% and 100%
    amplitude = fmax(0.05, fmin(1.0, amplitude));

    // Convert amplitude (0–1) to PWM duty (0–1023)
    uint32_t duty = (uint32_t)(amplitude * PWM_MAX_DUTY);

    // Apply duty to PWM hardware
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);

    // Update hardware with new duty value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}



// STOP AUDIO OUTPUT

static void stop_audio()
{
    // Set duty to zero (turn off PWM)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);

    // Apply change
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}



// MAIN UPDATE FUNCTION
// Call this frequently (e.g. every 1 ms)
void korotkoff_audio_update()
{
    // Get current time in microseconds
    int64_t now = esp_timer_get_time();

    // If a beat is active and duration has passed
    if (beatActive && (now - beatStartTime > BEAT_DURATION_US))
    {
        beatActive = false;  // Mark beat as finished
        stop_audio();        // Turn off sound
    }

    // Only generate sounds during cuff deflation
    if (!isDeflating)
    {
        prevPressure = pressure;  // Update previous pressure
        return;                   // Exit early
    }

    // Only allow sound between systolic and diastolic
    if (cuffPressure < systolic &&
        cuffPressure > diastolic)
    {
        // Detect arterial pressure crossing above cuff pressure
        if (pressure > cuffPressure &&
            prevPressure <= cuffPressure)
        {
            // Compute amplitude scaling:
            // Loud near systolic, softer near diastolic
            double amplitude =
                (cuffPressure - diastolic) /
                (systolic - diastolic);

            // Apply amplitude to PWM output
            set_pwm_amplitude(amplitude);

            // Mark beat as active
            beatActive = true;

            // Record start time
            beatStartTime = now;
        }
    }

    // Save current pressure for next loop comparison
    prevPressure = pressure;
}
