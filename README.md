
### Leaf-Link: A battery-free energy harvesting forest sensing system with adaptive scheduling and LoRaWAN connectivity


<!-- [![Paper](https://img.shields.io/badge/paper-PDF-blue)](LINK_TO_PAPER) -->
<!-- [![DOI](https://zenodo.org/badge/DOI_PLACEHOLDER.svg)](https://doi.org/DOI_PLACEHOLDER) --> 

---

## Overview
Leaf-Link is a wireless sensing system designed for **environmental monitoring**. It operates **battery-free** using super-capacitors energy storage and solar energy harvesting, with adaptive power management hardware  and intelligent scheduling based on **AsTAR++ scheduler**, and communicates over **LoRaWAN**.  

> <sub>**Note:** This repository provides supplementary material to the Leaf-Link system described in our conference paper submission. It focuses on details that could not be included in the manuscript due to space constraints. <sub>

This repository contains:
- [**Software**](software/): Firmware and scheduler implementation, with configuration files and build instructions  
- [**Hardware**](hardware/): Schematics, BOM, wiring diagrams, etc.  
- [**Implementation and analysis**](analysis/): Docs to reproduce figures and metrics from datasets  
- [**Results**](results/): Deployment traces, logs and cleaned datasets
- Failure notes, limitations and lessons learned   
<!-- - 📄 **Paper Link** -->

---

## Hardware Details

### Bill of Materials (BOM)

The tables below lists all core components for both node variants:  

#### Battery-Free Node

| Component              | Specification                  | Qty |
|------------------------|--------------------------------|-----|
| STM32WLE5CC (RAK3172) | MCU + LoRaWAN radio            | 1   | 
| BME680 sensor          | Temp/Humidity/Pressure/VOC     | 1   | 
| Supercapacitors        | 15 F, 5.5 V each (~0.252 Wh)   | 4   |
| Solar panel            | 6.0 V, 250 mA                  | 1   | 
| Voltage divider        | Resistor pair                  | 1   | 
| Connectors & wiring    | JST + custom PCB               | -   |        |

#### Battery-Powered Node

| Component              | Specification                  | Qty | 
|------------------------|--------------------------------|-----|
| STM32WLE5CC (RAK3172) | MCU + LoRaWAN radio            | 1   | 
| BME680 sensor          | Temp/Humidity/Pressure/VOC     | 1   | 
| LiPo battery           | 3200 mAh, 3.7 V (~11.84 Wh)    | 1   | 
| Solar panel            | 6.0 V, 250 mA                  | 1   | 
| Charging IC            | CN3065                         | 1   | 
| Connectors & wiring    | JST + custom PCB               | -   | 

> 🔎 For detailed schematics, wiring diagrams and component images, see the [hardware folder](hardware/).
---

## Software Details

The firmware is based on **Zephyr RTOS** and integrates the **AsTAR++ scheduler** with LoRaWAN communication.

### Scheduler Parameters
- **Sensing rate:**  
  - Minimum: 2 hours  
  - Maximum (day/night): 5 minutes  
- **Voltage thresholds:**  
  - vMin = 3.9 V  
  - vOptimum = 4.15 V  
  - vMax = 5.4 V  

### LoRaWAN Configuration
- Region: EU868  
- Spreading Factor: 12  
- TX Power: 14 dBm  
- Activation: OTAA  
- Payload structure: sensor measurements (BME680), capacitor voltage and node status flags  

> 🔎 For firmware source, scheduler code and build instructions, see the [hardware folder](software/).
---

## Results and Figures

Key results are summarised below; full sets of traces are available in the [results folder](results/).

### Message Throughput
Battery-free nodes achieved stable operation with adaptive scheduling, while battery-powered nodes stalled after deep depletion.  

![Messages per Day](analysis/output/messages_per_day.png)  

### Reliability
End-to-end message reliability reached 97–98% after server fixes.  

![Reliability CDF](analysis/output/reliability_cdf.png)  

### Energy Traces
Capacitor voltages show smooth recovery even under low-light conditions.  

![Capacitor Voltage](analysis/output/cap_voltage.png) 
---

## Failures and Limitations

In addition to node-specific issues reported in the manuscript, we list further bugs observed during deployment.

- **Software crashes:** Rare resets (<1% of total operation time) traced to LoRaWAN stack instability.  
- **Backend bug:** Initial rejection of packets with NULL SNR values caused early data loss (~3% of total packets). Fixed mid-deployment.    
- **Canopy sensitivity:** Nodes under denser canopy harvested ~30–40% less energy, resulting in longer duty cycles.   

---




