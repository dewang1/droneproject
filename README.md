# AT&T Mentorship: Autonomous Camera-Controlled Drone

## Overview
This project explores an alternative method for precise drone control by integrating computer vision, control theory, and signal processing. The system utilizes a ground-based camera to provide real-time positioning data, improving accuracy and responsiveness compared to GPS. The goal was to enable the drone to hold a precise location for applications such as measuring cell tower signals.

## Features
### Hardware
- **Modified Drone Controller**: Custom circuitry to interface with a microcontroller.
- **Microcontroller Integration**: Receives commands from a computer and outputs voltages to the drone controller.
- **Oscilloscope & Soldering Kit**: Used for circuit analysis and modification.

### Software
- **Arduino Code**: Controls communication between the microcontroller and drone.
- **Python Program**: Processes camera input, computes control signals, and sends voltage commands using:
  - **Computer Vision**: Detects and tracks drone position.
  - **PID Control Algorithm**: Ensures precise positioning adjustments.


## Project Motivation
This project was inspired by our mentor based on his work at **AT&T**, where drones measure cell tower signals. Traditional drones use GPS for positioning, but we aimed to develop a faster, more precise method by utilizing a **ground-based camera** for real-time adjustments.

## Setup
1. Modify the drone controller and wire it to the microcontroller.
2. Add colored markers to drone.
3. Set up the camera to track the drone’s position.
4. Connect the microcontroller to the computer.
5. Run the main Python program.

## Usage
1. Power on the drone and fly it above the camera.
4. Adjust PID parameters for fine-tuned stability.
