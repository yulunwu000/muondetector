# Muon Detector – PIC18F87J50 Firmware (Assembly)

This repository contains custom assembly firmware for a muon detector based on the **PIC18F87J50** microcontroller mounted on a **MikroElektronika Clicker 2 for PIC18FJ** board.
The detector uses a **plastic scintillator** coupled to a **Silicon Photomultiplier (SiPM)** to detect muons and other charged particles. The project is inspired by the open-source **CosmicWatch Desktop Muon Detector** (MIT/NCBJ) and replaces the original Arduino-based electronics with a custom PIC18 firmware and ADC readout path.  
See the CosmicWatch instruction manual for the reference detector design: http://www.cosmicwatch.lns.mit.edu/detector

## 🎯 Project Goal

The primary goal is to reproduce the CosmicWatch functionality using a PIC18 microcontroller and
low-level assembly to achieve:

- fast, deterministic ADC sampling  
- direct control of signal-conditioning GPIOs  
- efficient thresholding and peak-capture logic  
- easy integration with Clicker 2 board peripherals  

The firmware reads out analogue pulses produced by a **SiPM** when a cosmic muon passes through a scintillator block. The pulses are conditioned by an external amplification and peak-detection circuit, then digitised by the PIC18 ADC.


## 🧱 Hardware Overview

### Detector Components
- **Scintillator:** stacked 5×5×1 cm plastic scintillator slabs
- **Photon sensor:** SensL Micro-C series SiPM (6×6 mm active area)  
- **Signal path:**  
  - SiPM bias at ~29.5 V  
  - Low-noise filtering network  
  - Amplifier (gain ≈ 20–25×)  
  - Peak detector (long decay, PIC-readable)  
  - Output pulse widths ≈ 0.5 ms  
- These stages are documented in the manual diagrams.

### Microcontroller Board
- **MikroElektronika Clicker 2 for PIC18FJ**
- **MCU: PIC18F87J50**
  - 12-bit ADC  
  - USB support  
  - Plenty of RAM/Flash for data buffering  
- Firmware written in **PIC18 assembly** using MPLAB X.

## 🖥️ Firmware Features

- **ADC acquisition** of the shaped SiPM peak signal  
- **Threshold detection** to identify real muon events  
- **Event timing** (relative time stamps)  
- **Optional USB serial output** for logging  
- **Support for multiple firmware branches** for different acquisition strategies  
- **Modular assembly structure** to allow low-level control of:  
  - ADC configuration  
  - Timer interrupts  
  - GPIOs (LED indicators, trigger outputs)  
  - SD-card and coincidence features

## 🛠️ Build Instructions

### Requirements
- **MPLAB X IDE v6.25**
- **XC8 assembler** (part of XC8 toolchain)
- **PIC18F87J50** device support installed

## AI Use Acknowledgement

Parts of the debugging process (e.g., SPI timing issues, SD card initialisation sequencing on PIC18F87J50) were assisted by ChatGPT as an interactive troubleshooting tool.  
All design decisions, code implementation, and verification were performed by me.

