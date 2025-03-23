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
  <img src="image.png" alt="Flowchart for Parkinsonian Tremor Classifier" width="600"/>
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

