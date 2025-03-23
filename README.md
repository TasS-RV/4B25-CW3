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

## 📝 Notes

- All math is implemented in fixed-point integer format — scaled where needed — to accommodate SEGGER RTT print limitations. Internal computations have been verified to be correct through logging the terminal output on python script and running a digital twin of the system, exlcuding the sensor reading stage. This is mainly a data post processingf script to verify the calculations being performed in C.
- Instead of importing the ```c math.h ``` library for square root, due to thius demanding too large stack memory allocation compared to a simple iterative method. This can however, result in Epistemic uncertainty, as on board there is no way live validate the inaccuracy of the iteration.
- The report states the maximum polling rate is 55Hz - inside ```c byte_to_state_conversion ``` we are reading one of each 3 registers at once, resulting in Type B uncertaity alongside an epistemic error from the assumption of x, y, z magnitude being at the same snapshot of time. This should not nesessarily be a major issue for a limb-mounted sensor as intended, given the oscillations would be multiaxial (unlike the mono-axial vibrations on the IB Integrated Coursework building vibration transducer.

<p align="center">
  <img src="LabSetup1.png" alt="Flowchart for Parkinsonian Tremor Classifier" width="600"/>
</p>

The following procedure describes how data for the Baysean classification was obtained:

1. kdjnbd
2. dfg 
3. fg


- Optional variance propagation introduces latency but increases robustness in detection.
- Designed to run efficiently on low-power embedded systems (e.g., FRDM-KL03Z with Warp firmware stack).

---





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

## 🌲 Function Call Tree
Refer to the flowchart for a brief summary of the operaation of each function - this call tree is better at understanding the inheritance between function calls and order of processing.

```text
boot.c  
│  
├── devMMA8451Q.c  
│   └── initMMA8451Q()  
│       └── configureSensorMMA8451Q()  
│           ├── writeSensorRegisterMMA8451Q()  
│           └── (sets sensor I2C registers and check status)  

├── detect.c  
│   └── while iteration for fixed iteration count limit to last 10s (main sampling loop at 40 Hz)  
│       └── byte_to_state_conversion()  
│           ├── devMMA8451Q.c  
│           │   └── readSensorRegisterMMA8451Q()  
│           ├── detect.c  
│           │   ├── convertAcceleration()  
│           │   └── get_sqrt()  
│           └── devMMA8451Q.c  
│               └── update_buffers()  
│                   ├── detect.c  
│                   │   └── update_goertzel()  
│                   │       ├── compute_power_uncertainty()   (only if variance flag enabled)  
│                   │       └── propagate_std_dev()  
│                   └── compute_goertzel_power()              (triggered every 0.5 s)  
│                       ├── compute_power_uncertainty()       (only if variance flag enabled)  
│                       └── calculate_baysean()
```

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

Dominant Oscillation detected at: 5 Hz. Probability of this being Parkinsonian tremors: 873 /1000.






