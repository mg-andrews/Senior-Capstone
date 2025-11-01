# Pulse Oximetry Data Clean/ Prep
Data is gathered from PhysioNet and Database publication 'OpenOximetry Repository' (Fong, N., Lipnick, M., Bickler, P., Feiner, J., & Law, T. (2025). OpenOximetry Repository (version 1.1.1). PhysioNet. RRID:SCR_007345. https://doi.org/10.13026/be2e-cn29) and the original publication (Fong, N., Lipnick, M.S., Behnke, E. et al. Open Access Dataset and Common Data Model for Pulse Oximeter Performance Data. Sci Data 12, 570 (2025). https://doi.org/10.1038/s41597-025-04870-8). The purpose is to analyze the discrepancy between oxygen saturation values measured by a pulse oximeter and true values to determine the affect of skin tone on pulse oximeter accuracy. Then providing new dataframes that are grouped skin tone based on the fitzpatrick scale that can be linked to waveforms based on the `encounter_id`. 
## Data_Cleaning_V1.0.ipynb
Downloaded 6 csv files bloodgas, devices, encounter, patient, pulseoximeter, spectrophotometer. Then cleaned and retain only relevant columns. All filtered dataframes were converted to csv files and stored as Cleaned_datasets/dataset_filtered.csv. 
### devices.csv
Was not filtered and kept as the original.
### patient.csv
Filtered to only include `patient_id`,`assigned_sex`, and `race`. 
### encounter.csv
Filtered to only include `patient_id`, `encounter_id`, `age_at_encounter`, `monk_fingernail`,`monk_palmar`,`fitzpatrick`,`finger_r2_device`,`finger_r2_diameter`. Values we reduce due to the scope of our project focusing on pulse oximeters being attached to the finger. 
### pulseoximeter.csv
Filtered to include `encounter_id`, `deivce`,`sample_number`,`saturation`,`pi`. All saturation values out of the range 30 <= saturation <= 100.0 were dropped from the dataset as that is the reasonable range for values obtained during a desaturation study.`saturation` and `pi` were made into lists based on `sample_number` and `encounter_id`
### spectrophotmeter.csv
Filtered to include `patient_id`, `encounter_id`, `group`, `melanin_index`,`hb_index`,`hb_so2_index`,`lab_l`,`km660`,`km700`. `group` was filtered and only included the values 'Palmar (C)' and 'Fingernail (A)'. 
### bloodgas.csv
Filtered to include `patient_id`, `enocunter_id`, `sample`, `so2`, `ScalcO2`. `sample` was renamed to `sample_number`. Then `so2` and `ScalcO2` were filtered based on the same range as `saturation` in the pulseoximeter data and were made into lists based on `sample_number` and `encounter_id`. Finally missing values for `so2` were filled by `ScalcO2` and created a new column `so2_filled` and stayed in list format. 
## Skin_Tone_Analysis_V1.0.ipynb
The purpose is to analyze the affect skin tone has on pulse oximetry accuracy. Data was grouped based on the fitzpatrick scale (I-VI) represented as (1.0-6.0) in the datasets. /
Using the filtered datasets I merged the bloodgas and pulseox datasets to based on the corresponding `encounter_id` and `sample_number` to create a new dataframe to compare the `saturation` and `so2` values. `saturation` values are ones measured by the pulse oximeter while `so2` values were obtained from a blood sample. Both columns were in list format so each list was averaged to get one mean value that was written to `saturation_mean` and `so2_mean`. Then I compared the values and those values were stored in `oxy_sat_comparison`. /
Merged the dataset to the encounter dataset to obtain the `fitzpatrick` column. Finally I performed a statistical analysis that grouped the rows by `fitzpatrick` values and analyzed the corresponding `oxy_sat_comparison` value to determine if there is a statisitically significant difference between discrepacy oxygen saturation values based on skin tone. /
A data dictionary was created that grouped the `encounter_id` by fitzpatrick number. It was stored as `Skin Tone Analysis/fitz_dict.json`. Then dataframes with all data from the final merged dataframe `Skin Tone Analysis/fitz_merged.csv` based on the fitzpatrick value. Then stored as `Skin Tone Analysis/fitz_n.csv`.
