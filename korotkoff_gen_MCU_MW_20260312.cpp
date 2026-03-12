// Author: Miles Wilson
// Pushed to git on 3/12/2026

// Port of korotkoff_gen_test_MW_20260304.py
// Generates korotkoff sounds on ESP32s3 and plays them through I2C on audio exciter

#include <driver/i2s.h>
#include <math.h>

#define I2S_BCLK 15
#define I2S_LRC  7
#define I2S_DIN  16

#define SAMPLE_RATE 16000
#define CHUNK_SIZE 256
#define HEART_RATE 72

#define BEAT_DURATION 0.18f

float gain = 1.8;

// Beat state
bool beat_active = false;
int beat_sample = 0;
int beat_length;

// Noise filter
float noise_prev = 0;

// Oscillator
float phase = 0;

void setup_i2s() {

  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = CHUNK_SIZE
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
}

float korotkoff_sample() {

  if (!beat_active) return 0;

  float t = (float)beat_sample / beat_length;

  // Exponential decay envelope
  float env = expf(-3.0f * t);

  // Attack
  if (t < 0.02f)
    env *= t / 0.02f;

  // Filtered turbulence noise
  float noise = ((float)esp_random() / UINT32_MAX) * 2 - 1;

  noise_prev = 0.02f * noise + 0.98f * noise_prev;

  // Resonance
  phase += 2.0f * M_PI * 80.0f / SAMPLE_RATE;
  if (phase > 2*M_PI) phase -= 2*M_PI;

  float resonance = sinf(phase);

  float s = env * (0.8f * noise_prev + 0.3f * resonance);

  beat_sample++;

  if (beat_sample >= beat_length)
    beat_active = false;

  return s;
}

void start_beat() {

  beat_active = true;
  beat_sample = 0;
  beat_length = SAMPLE_RATE * BEAT_DURATION;
}

void setup() {

  Serial.begin(115200);
  setup_i2s();

  beat_length = SAMPLE_RATE * BEAT_DURATION;
}

void loop() {

  static int16_t chunk[CHUNK_SIZE];
  static unsigned long lastBeat = 0;

  unsigned long now = millis();

  if (now - lastBeat > 60000 / HEART_RATE) {
    start_beat();
    lastBeat += 60000 / HEART_RATE;
  }

  for (int i = 0; i < CHUNK_SIZE; i++) {

    float s = korotkoff_sample();

    float out = gain * s * 16000;

    if (out > 32767) out = 32767;
    if (out < -32768) out = -32768;

    chunk[i] = (int16_t)out;
  }

  size_t written;
  i2s_write(I2S_NUM_0, chunk, sizeof(chunk), &written, portMAX_DELAY);
}
