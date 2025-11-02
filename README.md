# Neonatal Incubator With Real-Time Response to Central Apnea Episodes

This repository contains the development of a real-time monitoring and response system for preterm neonates experiencing central apnea episodes. The project integrates physiological signal acquisition, diaphragmatic electromyography (EMGdi) estimation, apnea detection through machine learning, a real-time monitoring interface, and a mechanical stimulation controller designed to restore respiratory drive when apnea is detected.

Additional context, background, and development process of this project can be reviewed in the publication available at:
https://repository.eia.edu.co/entities/publication/4da3f27d-3354-4225-935d-de69f279c306

---

## Project Overview

Central apnea in premature infants is characterized by the temporary cessation of respiratory neural output, resulting in the suppression of diaphragmatic activity. Continuous monitoring and timely mechanical stimulation can reduce the risk of hypoxemia and associated neurodevelopmental complications. The system implemented here performs apnea identification and activates a support mechanism in response.

---

## Jupyter Notebooks

### `EMGdi_Estimation.ipynb`
This notebook attempts to predict diaphragmatic EMG (EMGdi) windows from neonatal ECG recordings. The objective is to identify EMGdi suppression patterns associated with apnea and to evaluate whether ECG-derived signals contain sufficient information to infer respiratory drive activity.

### `Simulated_Central_Apnea_Episodes.ipynb`
This notebook describes the generation of synthetic EMGdi signals containing central apnea episodes. A dataset of 5000 labeled signal windows was created based on known physiology of neonatal diaphragmatic activation and suppression during apnea. These synthetic episodes provide controlled training conditions where apnea boundaries are explicitly defined.

### `Simulated_Central_Apnea_ID.ipynb`
This notebook trains and evaluates a neural network model to classify 8-second EMGdi windows as either apnea or normal respiration. The architecture used is:

- 1D convolutional layer with 32 filters (kernel size = 8, ReLU activation, padding enabled) to extract low-level temporal features.
- Normalization layer for stabilizing learning.
- Max pooling layer for temporal downsampling.
- Second 1D convolutional layer with 64 filters (kernel size = 5), followed by normalization and a second pooling layer.
- LSTM layer with 64 units to capture temporal patterns related to respiratory neural drive.
- Fully connected layer with 64 neurons and ReLU activation.
- Dropout layer to prevent overfitting.
- Output layer with a single sigmoid neuron for binary classification (apnea vs normal).

Model performance was evaluated using test datasets and 5-fold cross-validation, achieving consistent separation between apnea and non-apnea windows. Results should be interpreted with consideration that the training dataset included artificially generated apnea episodes.

---

## Real-Time Monitoring Interface (`userinterface/`)

The real-time interface receives EMGdi/ECG data transmitted wirelessly from an ESP32 integrated with an AFE4500 analog front end. Data is transmitted using UDP over Wi-Fi. The interface performs continuous signal analysis and apnea detection.

When an apnea episode is identified, the interface provides an alert to the caregiver and activates the mechanical stimulation control system.

---

## Mechanical Stimulation Control (`Frequency_control/`)

The mechanical stimulation subsystem is implemented on a Raspberry Pi Pico. A discrete-time frequency control algorithm modulates the stimulation output based on apnea detection events. The activation of the stimulant mechanism is performed only when apnea is confirmed by the classifier.

