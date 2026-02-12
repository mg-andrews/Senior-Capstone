import random
import time

# Seed randomness (important for ESP32)
#random.seed(time.ticks_ms())

def main():
    """
    Take user input for patient state, Fitzpatrick value, and perfusion status, then calculate and display the simulated SpO2 reading along with bias components.

    true_state: Patient state (Healthy, Hyperventilation, Hypoxemia, Severe)
    fitz_value: Fitzpatrick skin type (1–6)
    low_perfusion: Boolean indicating if low perfusion is present (y/n)
    true_sao2: Simulated true arterial oxygen saturation based on patient state
    skin_bias: Randomized bias based on skin tone and patient state
    temp_bias: Randomized bias based on temperature/perfusion status
    """
    # Get user input with current 4 states
    true_state = input("Enter patient state (Healthy, Hyperventilation, Hypoxemia, Severe): ").capitalize()
    # Get fitz value
    fitz_value = int(input("Enter Fitzpatrick value (1–6): "))
    # Get perfusion status
    low_perf_input = input("Low perfusion? (y/n): ").lower()

    low_perfusion = (low_perf_input == 'y')

    # Randomly generate a true SaO2 value based on the patient state
    true_sao2= float(get_true_sao2(true_state))

    # Calculate output SpO2 with bias
    spo2, skin_bias, temp_bias = calculate_spo2(
        true_sao2,
        true_state,
        low_perfusion,
        fitz_value
    )

    print("\n--- SpO2 Simulation Result ---")
    print("True SaO2:", true_sao2)
    print("Skin Tone Bias:", round(skin_bias, 2))
    print("Temp/Perfusion Bias:", round(temp_bias, 2))
    print("Observed SpO2 (LCD Output):", spo2)

# Skin tone bias ranges by Fitz value and patient state
# Pulled from 25%/75% IQR bias values after analysis of the simulated data
# Taken from OS_Model_v1_MA_02_2026.ipynb
FITZ_1_BIAS_RANGES = {
    "Healthy": (-0.14166, 1.4937),
    "Hyperventilation": (-0.5, 1.0499),
    "Hypoxemia": (-0.09999, 1.79999),
    "Severe": (-0.52, 1.5),
}

FITZ_2_BIAS_RANGES = {
    "Healthy": (-0.34999, 1.797222),
    "Hyperventilation": (-0.27499, 0.4946428),
    "Hypoxemia": (-0.04999, 2.079166),
    "Severe": (-0.14722, 1.9000),
}

FITZ_3_BIAS_RANGES = {
    "Healthy": (-0.0666, 1.59999),
    "Hyperventilation": (-0.14166, 1.09999),
    "Hypoxemia": (0.08749, 2.3499),
    "Severe": (0.308, 2.2999),
}

FITZ_4_BIAS_RANGES = {
    "Healthy": (-0.200, 1.09999),
    "Hyperventilation": (-0.25, 0.455555),
    "Hypoxemia": (0.15, 1.7000),
    "Severe": (0.2, 2.3458),
}

FITZ_5_BIAS_RANGES = {
    "Healthy": (0.09999, 2.09999),
    "Hyperventilation": (-0.212499, 1.352777),
    "Hypoxemia": (0.4000, 2.9027777),
    "Severe": (0.7000, 3.8499),
}

FITZ_6_BIAS_RANGES = {
    "Healthy": (0.47777, 2.0066),
    "Hyperventilation": (0.312500, 1.79999),
    "Hypoxemia": (1.15000, 3.0),
    "Severe": (1.17232, 4.0833),
}

# Temperature / perfusion bias ranges
TEMP_BIAS_NORMAL = (0.0, 0.0)
TEMP_BIAS_LOW_PERFUSION = (-2.0, 0.0)


def get_true_sao2(state):
    """Return a randomized true SaO2 value from a range of OS values based on the patient state."""
    if state == 'Healthy':
        return random.uniform(95, 97)
    elif state == 'Hyperventilation':
        return random.uniform(98, 100)
    elif state == 'Hypoxemia':
        return random.uniform(85, 94)
    elif state == 'Severe':
        return random.uniform(70, 84)
    else:
        raise ValueError("Invalid state. Must be one of: Healthy, Hyperventilation, Hypoxemia, Severe.")
    

def get_skin_tone_bias(true_state, fitz_value):
    """Return a randomized skin tone bias based on Fitz value and the true state and its corresponding ranges from FITZ_#_BIAS_RANGES."""
    # Cycle through each fitz value.
    if fitz_value == 1:
        bias_min, bias_max = FITZ_1_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)
    elif fitz_value == 2:
        bias_min, bias_max = FITZ_2_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)  
    elif fitz_value == 3:
        bias_min, bias_max = FITZ_3_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)
    elif fitz_value == 4:
        bias_min, bias_max = FITZ_4_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)
    elif fitz_value == 5:
        bias_min, bias_max = FITZ_5_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)
    elif fitz_value == 6:
        bias_min, bias_max = FITZ_6_BIAS_RANGES.get(true_state, (0.0, 0.0))
        return random.uniform(bias_min, bias_max)
    else:
        return 0.0  # No bias for other Fitzpatrick values


def get_temp_bias(low_perfusion):
    """Return a randomized temperature/perfusion bias."""
    if low_perfusion:
        return random.uniform(*TEMP_BIAS_LOW_PERFUSION)
    else:
        return random.uniform(*TEMP_BIAS_NORMAL)

def calculate_spo2(true_sao2, true_state, low_perfusion, fitz_value):
    """
    Docstring for calculate_spo2
    
    :param true_sao2: float representing the true arterial oxygen saturation based on patient state
    :param true_state: string representing the patient state (Healthy, Hyperventilation, Hypoxemia, Severe)
    :param low_perfusion: boolean indicating if low perfusion is present (y/n)
    :param fitz_value:  integer representing the Fitzpatrick skin type (1–6)
    """
    skin_bias = get_skin_tone_bias(true_state, fitz_value)
    temp_bias = get_temp_bias(low_perfusion)

    # Calculate with bias values 
    # Potentially add to this with weighted parameters for each bias type, but for now just sum them
    observed_spo2 = true_sao2 + skin_bias + temp_bias

    # Clamp to physiological bounds and round for LCD display
    # This ensures the output is between 0 and 100 and rounds to the nearest whole number for display purposes
    observed_spo2 = round(max(0, min(100, observed_spo2)))

    return observed_spo2, skin_bias, temp_bias


main()
