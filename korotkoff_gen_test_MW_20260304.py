# Proof of concept for Korotkoff sound generation

# Author: Miles Wilson
# Published: 03/04/2026

# Generates high and low frequency white noise, gates the noise, and adds attack, delay, reverb, bandpass
# Basic linear cuff deflation, sounds confined to between systole and diastole
# 16kHz audio with 512 bit buffer

# Noise beat generated during runtime, fix to preallocate noise for prod
# Different phases chosen for somewhat arbitrary pressures

import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
import soundfile as sf
import time
import os

# =========================
# PARAMETERS
# =========================
fs_audio = 16000       # audio sample rate
chunk_size = 512
fs_vis = 50

heart_rate = 72
systolic = 120
diastolic = 80
start_pressure = 130
deflation_rate = 10

korotkoff_duration = 0.12  # base beat duration
reverb_delay_ms = 40
reverb_decay = 0.3
default_attack_time = 0.03  # 50 ms

# =========================
# ACTIVE BEAT STRUCTURE
# =========================
class ActiveBeat:
    def __init__(self, waveform):
        self.waveform = waveform
        self.idx = 0

active_beats = []
recorded_audio = []

# =========================
# KOROTKOFF PHASES
# =========================
def get_phase_params(cuff_pressure, systolic, diastolic):
    p_norm = (cuff_pressure - diastolic) / (systolic - diastolic)
    if p_norm >= 0.85:
        return {'fc': 80, 'bandwidth': 70, 'decay_tau_high': 0.04,
                'decay_tau_low': 0.07, 'amp': 1, 'attack_time': 0.03}
    elif p_norm >= 0.7:
        return {'fc': 80, 'bandwidth': 70, 'decay_tau_high': 0.06,
                'decay_tau_low': 0.1, 'amp': 0.8, 'attack_time': 0.05}
    elif p_norm >= 0.4:
        return {'fc': 80, 'bandwidth': 70, 'decay_tau_high': 0.04,
                'decay_tau_low': 0.07, 'amp': 0.9, 'attack_time': 0.05}
    elif p_norm >= 0.2:
        return {'fc': 80, 'bandwidth': 72, 'decay_tau_high': 0.08,
                'decay_tau_low': 0.15, 'amp': 0.5, 'attack_time': 0.07}
    else:
        return {'fc': 0, 'bandwidth': 0, 'decay_tau_high': 0,
                'decay_tau_low': 0, 'amp': 0, 'attack_time': default_attack_time}

# =========================
# REVERB FUNCTION
# =========================
def add_reverb(signal_in, fs, delay_ms=50, decay=0.3):
    delay_samples = int(fs * delay_ms / 1000)
    output = np.zeros(len(signal_in) + delay_samples)
    output[:len(signal_in)] = signal_in
    for i in range(delay_samples, len(output)):
        output[i] += decay * output[i - delay_samples]
    # prevent clipping
    max_val = np.max(np.abs(output))
    if max_val > 1:
        output /= max_val
    return output

# =========================
# KOROTKOFF GENERATOR
# =========================
def generate_korotkoff_phase(fs=16000, duration=0.07, fc=110, bandwidth=80, decay_tau_high=0.04, decay_tau_low=0.07, amp=1.5, attack_time=default_attack_time):
    if amp == 0:
        return np.zeros(int(fs*duration))

    N = int(fs*duration)
    t = np.arange(N)/fs

    # High-frequency click
    noise = np.random.uniform(-1,1,N)
    low = max(0, (fc-bandwidth/2))/(fs/2)
    high = (fc+bandwidth/2)/(fs/2)
    from scipy.signal import butter, lfilter
    b,a = butter(2, [low,high], btype='band')
    high_filtered = lfilter(b,a,noise)
    envelope_high = np.exp(-t/decay_tau_high)

    # Low-frequency bass
    b,a = butter(2, 50/(fs/2), btype='low')
    bass_filtered = lfilter(b,a,np.random.uniform(-1,1,N))
    envelope_low = np.exp(-t/decay_tau_low)

    # Fade-in/out
    fade_samples = int(0.003*fs)
    envelope_high[:fade_samples] *= np.linspace(0,1,fade_samples)
    envelope_high[-fade_samples:] *= np.linspace(1,0,fade_samples)
    envelope_low[:fade_samples] *= np.linspace(0,1,fade_samples)
    envelope_low[-fade_samples:] *= np.linspace(1,0,fade_samples)

    waveform = amp*(high_filtered*envelope_high + 0.4*bass_filtered*envelope_low)

    # Apply attack
    attack_samples = int(attack_time*fs)
    if attack_samples > 0:
        attack_env = np.ones(N)
        attack_env[:attack_samples] = np.linspace(0,1,attack_samples)
        waveform *= attack_env

    # Apply reverb
    waveform = add_reverb(waveform, fs, delay_ms=reverb_delay_ms, decay=reverb_decay)

    # Scale down for headroom
    waveform *= 0.7

    return waveform

# =========================
# SIMULATION STATE
# =========================
cuff_pressure = start_pressure
last_beat_time = 0
beat_interval = 60 / heart_rate

# =========================
# AUDIO CALLBACK
# =========================
def audio_callback(outdata, frames, time_info, status):
    global cuff_pressure, last_beat_time, active_beats, recorded_audio

    chunk = np.zeros(frames)
    current_time = time.time()

    # Trigger new beat if in range
    if systolic > cuff_pressure > diastolic:
        if current_time - last_beat_time >= beat_interval:
            params = get_phase_params(cuff_pressure, systolic, diastolic)
            k_waveform = generate_korotkoff_phase(fs=fs_audio, duration=korotkoff_duration, **params)
            active_beats.append(ActiveBeat(k_waveform))
            last_beat_time = current_time

    # Sum overlapping beats
    for beat in active_beats[:]:
        remaining = len(beat.waveform) - beat.idx
        n = min(remaining, frames)
        chunk[:n] += beat.waveform[beat.idx:beat.idx+n]
        beat.idx += n
        if beat.idx >= len(beat.waveform):
            active_beats.remove(beat)

    # Clamp per chunk to avoid clipping
    chunk = np.clip(chunk, -1, 1)

    outdata[:] = chunk.reshape(-1,1)
    recorded_audio.append(chunk.copy())

# =========================
# START AUDIO STREAM
# =========================
stream = sd.OutputStream(channels=1, samplerate=fs_audio, blocksize=chunk_size, callback=audio_callback)
stream.start()

# =========================
# VISUALIZATION SETUP
# =========================
plt.ion()
fig, ax = plt.subplots(figsize=(10,4))
ax.set_xlim(0,30)
ax.set_ylim(diastolic-5,start_pressure+5)
pressure_line, = ax.plot([],[], color='blue')
ax.axhline(systolic, color='gray', linestyle='--')
ax.axhline(diastolic, color='gray', linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Cuff Pressure (mmHg)")

pressure_history = []
time_history = []

# =========================
# MAIN LOOP
# =========================
start_time = time.time()

while cuff_pressure > diastolic:
    cuff_pressure -= deflation_rate / fs_vis
    t_now = time.time() - start_time
    time_history.append(t_now)
    pressure_history.append(cuff_pressure)

    pressure_line.set_data(time_history, pressure_history)
    ax.set_xlim(max(0, t_now-30), t_now+1)
    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(1/fs_vis)

# =========================
# STOP STREAM + SAVE AUDIO
# =========================
stream.stop()
stream.close()

final_audio = np.concatenate(recorded_audio)
if np.max(np.abs(final_audio)) > 0:
    final_audio /= np.max(np.abs(final_audio))

# Save in script folder
script_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(script_dir, "korotkoff_simulation.wav")
sf.write(output_path, final_audio, fs_audio)
print(f"Simulation complete. Audio saved to: {output_path}")
