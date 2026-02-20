import random
import time

# Seed randomness (important for ESP32)
#random.seed(time.ticks_ms())

# Scenario 1: a healthy patient coming in for an annual health visit and no pre-existing heart issues or history of hypotension
# 1. Blood pressure will be set within a normotensive range (Systolic BP < 120mmHg and Diastolic BP < 80 mmHg)
# 2. Oxygen saturation will be set at a normal range (SpO2 95%-100%)

# Scenario 2: a patient with newly diagnosed hypertension that has not yet been managed with medication
# 1. Blood pressure will be set within the elevated to Stage I hypertensive range (Systolic BP 120-140 mmHg and Diastolic BP 80-90 mmHg)
# 2. Oxygen saturation will be set a normal range (SpO2 95%-100%)

# Scenario 3: an elderly patient with atherosclerosis and COPD arriving to the ED with difficulty breathing
# 1. Blood pressure will be in the Stage II Hypertensive range (Systolic BP > 140+ mmHg and Diastolic BP > 90+ mmHg) and an
# auscultatory gap will be present
# 2. Oxygen saturation will be set at a hypoxic range (<92% SpO2) to reflect patient’s history of chronic lung disease

# Scenario 4: a chronic kidney disease patient who just received dialysis who is feeling mildly nauseous and fatigued
# 1. Blood pressure will be set at a hypotensive range (Systolic BP <90 mmHg and Diastolic BP <60 mmHg)
# 2. Oxygen saturation will be set at a normal to low range to reflect physiological response to dialysis (SpO2 95%-100%)

# Overall change in script from low perfusion to low temperature (2/18/2026) 
def main():
    """
    Take user input for patient state, Fitzpatrick value, and temperature status, then calculate and display the simulated SpO2 reading along with bias components.

    true_state: Patient state (Healthy, Slightly below normal, Hypoxemia, Scenario(1-4))
    fitz_value: Fitzpatrick skin type (1–6)
    low_temp: Boolean indicating if low patient temperature is present (y/n)
    true_sao2: Simulated true arterial oxygen saturation based on patient state
    skin_bias: Randomized bias based on skin tone and patient state
    temp_bias: Randomized bias based on temperature/perfusion status
    """
    # Get user input with current 4 states
    true_state = input("Enter patient state (Healthy, Slightly below normal, Hypoxia, Scenario (1-4)): ").capitalize()
    # Get fitz value
    fitz_value = int(input("Enter Fitzpatrick value (1–6): "))
    # Get perfusion status
    low_temp_input = input("Low Patient Temperature? (y/n): ").lower()

    low_temp = (low_temp_input == 'y')

    # Randomly generate a true SaO2 value based on the patient state
    true_sao2= float(get_true_sao2(true_state))

    # Calculate output SpO2 with bias
    spo2, skin_bias, temp_bias = calculate_spo2(
        true_sao2,
        true_state,
        low_temp,
        fitz_value
    )

    print("\n--- SpO2 Simulation Result ---")
    print("True SaO2:", true_sao2)
    print("Skin Tone Bias:", round(skin_bias, 2))
    print("Temp/Perfusion Bias:", round(temp_bias, 2))
    print("Observed SpO2 (LCD Output):", spo2)

# Add truncated Gaussian function for potential future use if we want to sample from a normal distribution with bounds (e.g., for more realistic bias sampling instead of uniform)
# 2/19/2026 
def truncated_gaussian(mean, std, min_val, max_val):
    """
    Sample from a Gaussian distribution truncated to [min_val, max_val].
    Uses rejection sampling (simple + ESP32 friendly).
    """
    while True:
        value = random.gauss(mean, std)
        if min_val <= value <= max_val:
            return value

# Skin tone bias ranges by Fitz value and patient state
# Pulled from 25%/75% IQR bias values after analysis of the simulated data
# Taken from OS_Model_v1_MA_02_2026.ipynb
# Update ranges 2/18/2026 to reflect new patient state bins (Healthy, Slightly below normal, Hypoxia) and new bias values from the notebook analysis
# Updated range names to include new patient states (Scenario 1-4) 2/18/2026
FITZ_1_BIAS_RANGES = {
    "Healthy": {
                "mean": 0.625,
                "std": 1.14,
                "min": -2.27,
                "max": 3.40
                },
    "Scenario 1": {
                "mean": 0.625,
                "std": 1.14,
                "min": -2.27,
                "max": 3.40
                },
    "Scenario 2": {
                "mean": 0.625,
                "std": 1.14,
                "min": -2.27,
                "max": 3.40
                },
    "Scenario 4": {
                "mean": 0.625,
                "std": 1.14,
                "min": -2.27,
                "max": 3.40
                },
 "Slightly below normal": {
                            "mean": 0.981,
                            "std": 1.31,
                            "min": -2.20,
                            "max": 1.86
                            }, 
    "Hypoxia": {
                "mean": 0.565,
                "std": 1.52,
                "min": -3.60,
                "max": 4.60
                },
 "Scenario 3": {
                "mean": 0.565,
                "std": 1.52,
                "min": -3.60,
                "max": 4.60
                },
}
FITZ_2_BIAS_RANGES = {
    "Healthy": {
                "mean": 0.568,
                "std": 1.23,
                "min": -3.05,
                "max": 3.60
                },
    "Scenario 1": {
                "mean": 0.568,
                "std": 1.23,
                "min": -3.05,
                "max": 3.60
                },
    "Scenario 2": {
                "mean": 0.568,
                "std": 1.23,
                "min": -3.05,
                "max": 3.60
                },
    "Scenario 4": {
                "mean": 0.568,
                "std": 1.23,
                "min": -3.05,
                "max": 3.60
                },
 "Slightly below normal": {
                            "mean": 1.40,
                            "std": 1.31,
                            "min": -2.15,
                            "max": 4.40
                            }, 
    "Hypoxia": {
                "mean": 0.899,
                "std": 1.64,
                "min": -3.32,
                "max": 5.20
                },
 "Scenario 3": {
                "mean": 0.899,
                "std": 1.64,
                "min": -3.32,
                "max": 5.20
                },
}

FITZ_3_BIAS_RANGES = {
    "Healthy": {
                "mean": 0.570,
                "std": 1.09,
                "min": -2.30,
                "max": 3.40
                },
    "Scenario 1": {
                "mean": 0.570,
                "std": 1.09,
                "min": -2.30,
                "max": 3.40
                },
    "Scenario 2": {
                "mean": 0.570,
                "std": 1.09,
                "min": -2.30,
                "max": 3.40
                },
    "Scenario 4": {
                "mean": 0.570,
                "std": 1.09,
                "min": -2.30,
                "max": 3.40
                },
 "Slightly below normal": {
                            "mean": 1.18,
                            "std": 1.59,
                            "min": -2.45,
                            "max": 5.70
                            }, 
    "Hypoxia": {
                "mean": 1.31,
                "std": 1.60,
                "min": -2.90,
                "max": 5.60
                },
 "Scenario 3": {
                "mean": 1.31,
                "std": 1.60,
                "min": -2.90,
                "max": 5.60
                },
}

FITZ_4_BIAS_RANGES = {
    "Healthy": {
                "mean": 0.400,
                "std": 0.928,
                "min": -2.20,
                "max": 2.65
                },
    "Scenario 1": {
                "mean": 0.400,
                "std": 0.928,
                "min": -2.20,
                "max": 2.65
                },
    "Scenario 2": {
                "mean": 0.400,
                "std": 0.928,
                "min": -2.20,
                "max": 2.65
                },
    "Scenario 4": {
                "mean": 0.400,
                "std": 0.928,
                "min": -2.20,
                "max": 2.65
                },
 "Slightly below normal": {
                            "mean": 0.797,
                            "std": 1.19,
                            "min": -2.05,
                            "max": 3.80
                            }, 
    "Hypoxia": {
                "mean": 1.22,
                "std": 1.55,
                "min": -3.0,
                "max": 5.50
                },
 "Scenario 3": {
                "mean": 1.22,
                "std": 1.55,
                "min": -3.0,
                "max": 5.50
                },
}

FITZ_5_BIAS_RANGES = {
    "Healthy": {
                "mean": 0.996,
                "std": 1.35,
                "min": -2.20,
                "max": 4.50
                },
    "Scenario 1": {
                "mean": 0.996,
                "std": 1.35,
                "min": -2.20,
                "max": 4.50
                },
    "Scenario 2": {
                "mean": 0.996,
                "std": 1.35,
                "min": -2.20,
                "max": 4.50
                },
    "Scenario 4": {
                "mean": 0.996,
                "std": 1.35,
                "min": -2.20,
                "max": 4.50
                },
 "Slightly below normal": {
                            "mean": 1.37,
                            "std": 1.57,
                            "min": -2.47,
                            "max": 5.05
                            }, 
    "Hypoxia": {
                "mean": 2.16,
                "std": 2.10,
                "min": -3.80,
                "max": 8.0
                },
 "Scenario 3": {
                "mean": 2.16,
                "std": 2.10,
                "min": -3.80,
                "max": 8.0
                },
}

FITZ_6_BIAS_RANGES = {
    "Healthy": {
                "mean": 1.16,
                "std": 1.00,
                "min": -1.70,
                "max": 3.70
                },
    "Scenario 1": {
                "mean": 1.16,
                "std": 1.00,
                "min": -1.70,
                "max": 3.70
                },
    "Scenario 2": {
                "mean": 1.16,
                "std": 1.00,
                "min": -1.70,
                "max": 3.70
                },
    "Scenario 4": {
                "mean": 1.16,
                "std": 1.00,
                "min": -1.70,
                "max": 3.70
                },
 "Slightly below normal": {
                            "mean": 1.75,
                            "std": 1.26,
                            "min": -1.75,
                            "max": 4.90
                            }, 
    "Hypoxia": {
                "mean": 2.52,
                "std": 1.99,
                "min": -2.96,
                "max": 8.0
                },
 "Scenario 3": {
                "mean": 2.52,
                "std": 1.99,
                "min": -2.96,
                "max": 8.0
                },
}

# Temperature / perfusion bias ranges
# Updated ranges to reflect new bias values from the notebook analysis 2/18/2026
# Update values to be sampled from a truncated gaussian distribution instead of uniform to reflect the distribution of bias values seen in the simulated data (2/19/2026)
TEMP_BIAS_NORMAL = (0.0, 0.0)
TEMP_BIAS_LOW_TEMP = {
                "mean": 8.50,
                "std": 3.65,
                "min": 0.10,
                "max": 13.8
                }

# Add med student scenarios to state and put it into the correct OS state 2/18/2026
def get_true_sao2(state):
    """Return a randomized true SaO2 value from a range of OS values based on the patient state."""
    if state == 'Healthy' or state == 'Scenario 1' or state == 'Scenario 2' or state == 'Scenario 4':
        return random.uniform(95, 100)
    elif state == 'Slightly below normal':
        return random.uniform(92, 94)
    elif state == 'Hypoxia' or state == 'Scenario 3':
        return random.uniform(80, 91)
    else:
        raise ValueError("Invalid state. Must be one of: Healthy, Slightly below normal, Hypoxia.")
    
# Update function to reflect bins ranges for mean, std, min, and max values and adjust the output to be computed by the truncated_gaussian function. (2/19/2026)
def get_skin_tone_bias(true_state, fitz_value):
    if fitz_value == 1:
        params = FITZ_1_BIAS_RANGES.get(true_state)
    elif fitz_value == 2:
        params = FITZ_2_BIAS_RANGES.get(true_state)
    elif fitz_value == 3:
        params = FITZ_3_BIAS_RANGES.get(true_state)
    elif fitz_value == 4:
        params = FITZ_4_BIAS_RANGES.get(true_state)
    elif fitz_value == 5:
        params = FITZ_5_BIAS_RANGES.get(true_state)
    elif fitz_value == 6:
        params = FITZ_6_BIAS_RANGES.get(true_state)
    else:
        return 0.0

    if params:
        return truncated_gaussian(
            params["mean"],
            params["std"],
            params["min"],
            params["max"]
        )
    return 0.0

# Use truncated gaussian sampling for temperature bias as well to reflect the distribution of bias values seen in the simulated data (2/19/2026)
def get_temp_bias(low_temp):
    """Return a randomized temperature/perfusion bias."""
    if low_temp:
        return truncated_gaussian(   TEMP_BIAS_LOW_TEMP["mean"],
                                            TEMP_BIAS_LOW_TEMP["std"],
                                            TEMP_BIAS_LOW_TEMP["min"],
                                            TEMP_BIAS_LOW_TEMP["max"])
    else:
        return random.uniform(*TEMP_BIAS_NORMAL)

def calculate_spo2(true_sao2, true_state, low_temp, fitz_value):
    """
    Docstring for calculate_spo2
    
    :param true_sao2: float representing the true arterial oxygen saturation based on patient state
    :param true_state: string representing the patient state (Healthy, Slightly below normal, Hypoxia, Scenario 1-4)
    :param low_temp: boolean indicating if low patient temperature is present (y/n)
    :param fitz_value:  integer representing the Fitzpatrick skin type (1–6)
    """
    skin_bias = get_skin_tone_bias(true_state, fitz_value)
    temp_bias = get_temp_bias(low_temp)

    # Calculate with bias values 
    # Potentially add to this with weighted parameters for each bias type, but for now just sum them
    observed_spo2 = true_sao2 + skin_bias + temp_bias

    # Clamp to physiological bounds and round for LCD display
    # This ensures the output is between 0 and 100 and rounds to the nearest whole number for display purposes
    observed_spo2 = round(max(0, min(100, observed_spo2)))

    return observed_spo2, skin_bias, temp_bias


main()
