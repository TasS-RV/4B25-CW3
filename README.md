# Coursework Activity 4: Real-Time Parkinsonian Tremor Detection Using MMA8451Q Accelerometer

## Author
**Name:** Tasin Sayed  
**College:** Corpus Christi College  
**CRSid:** tes46

---

## 🧠 Project Overview

This project implements a lightweight, real-time classifier to detect Parkinsonian tremors based on dominant frequency analysis of 3-axis accelerometer data from the **MMA8451Q** sensor.

The algorithm samples acceleration at **40Hz**, calculates the **magnitude** of acceleration vectors, and uses the **Goertzel algorithm** to extract frequency-domain information in a rolling 0.5-second window. Frequencies in the range of **3–7Hz** are especially indicative of Parkinsonian tremors.

---

## 📊 System Flowchart

<p align="center">
  <img src="Flowchart_for_Implementation.png" alt="Flowchart for Parkinsonian Tremor Classifier" width="600"/>
</p>

The system follows a non-blocking polling loop and classifies tremors based on the dominant frequency every 0.5 seconds using a Bayesian posterior confidence score.

---

## 🗂️ File Structure

| File | Description |
|------|-------------|
| `boot.c` | Main execution file. Initializes the sensor, controls timing, and runs the main sampling loop. |
| `devMMA8451Q.c` | Contains low-level I2C communication and sensor configuration functions, as well as buffering and filtering logic. |
| `devMMA8451Q.h` | Header file for `devMMA8451Q.c` with function and constant declarations. |
| `detect.c` | Implements signal processing: Goertzel sequence update, power calculation, variance propagation, and Bayesian classification. |
| `detect.h` | Header file for `detect.c` with external declarations. |
| `config.h` | Global configuration, feature toggles, constants, and sensor activation flags. |

---

## 🧩 Function Inheritance & Key Call Graph
boot.c │ ├── initMMA8451Q() │ └── sets sensor I2C address and voltage │ ├── configureSensorMMA8451Q() │ └── writeSensorRegisterMMA8451Q() │ └── while (sampling loop @ 40Hz) └── byte_to_state_conversion() ├── readSensorRegisterMMA8451Q() ├── convertAcceleration() ├── get_sqrt() ├── update_buffers() │ ├── update_goertzel() │ │ └── compute_power_uncertainty() (if enabled) │ └── compute_goertzel_power() (every 0.5s) │ ├── compute_power_uncertainty() (if enabled) │ └── calculate_baysean() └── propagate_std_dev()


---

## 🔍 Detection Logic

- **Input**: Acceleration vectors (X, Y, Z) sampled at 40 Hz.
- **Buffer**: Magnitude buffer of 20 samples (0.5 seconds).
- **Frequency Analysis**: Goertzel algorithm applied at 11 bins (3–13 Hz).
- **Uncertainty**: Optional propagation of variance and covariances.
- **Classification**: Bayesian calculation using precomputed PDFs:
  - `PDF_parkinsonian[]`
  - `PDF_non_parkinsonian[]`
- **Output**: Prints dominant frequency and confidence score in detecting tremor.

---

## ⚙️ Compilation Flags (in `config.h`)

| Flag | Description |
|------|-------------|
| `MMA8451Q_RAW_DATA_COLLECT` | Enable raw accelerometer + power printouts. |
| `MMA8451Q_RAW_VarError_PROP` | Enable variance/covariance propagation. |
| `MMA8451Q_Powerprintouts` | Enable individual frequency bin power output. |

---

## 🧪 Example Output

Dominant Oscillation detected at: 5 Hz. Probability of this being Parkinsonian tremors: 873 /1000


---

## 📝 Notes

- All math is implemented in fixed-point integer format — scaled where needed — to accommodate microcontroller limitations.
- Optional variance propagation introduces latency but increases robustness in detection.
- Designed to run efficiently on low-power embedded systems (e.g., FRDM-KL03Z with Warp firmware stack).

---

## 📍 Future Improvements

- FFT alternative using more bins for better frequency resolution.
- Adaptive windowing or decimation to balance time vs. frequency resolution.
- Multi-axis frequency decomposition instead of magnitude-only.

---



