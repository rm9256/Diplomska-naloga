# Motorcycle Visual Tilt Indicator System

This repository contains the code for a motorcycle tilt indicator system that uses an MPU6050 accelerometer and gyroscope sensor to measure
tilt angles and control LED strips for visual feedback. The system is designed to assist riders by providing real-time tilt angle visualization, enhancing awareness and safety.

## Features
- **Tilt Angle Calculation**: Uses the MPU6050 to measure acceleration and compute tilt angles relative to the ground.
- **LED Feedback**: LED strips light up sequentially based on the tilt angle, providing intuitive visual feedback.
- **Calibration Process**: The system includes a calibration routine to establish baseline tilt values for accurate measurements.
- **Debounce Logic**: Ensures stable button presses to toggle modes without unintended behavior.

## How It Works
1. **Sensor Initialization**: The MPU6050 is initialized and begins transmitting acceleration data.
2. **Calibration**: On startup or reset, the system captures initial sensor readings and calculates the reference tilt angle.
3. **Tilt Calculation**: Real-time tilt angles are computed using `atan2` with the Y and Z acceleration data.
4. **LED Control**: Based on the calculated tilt angle, corresponding LEDs light up to represent the lean direction and angle.
5. **Mode Switching**: A button allows toggling between different modes, such as normal mode, service mode and recalibration.

## Usage
1. Upload the code to an Arduino-compatible microcontroller.
2. Connect the MPU6050 sensor and LED strips.
3. Power the system and press the button to initiate calibration.
4. Tilt the setup to test the LED response to the sensor's readings.

## Notes
- The system was developed and tested in a controlled environment on a bike.
- For optimal accuracy, recalibrate periodically or after significant temperature changes.

## License
This project is open-source and can be modified or improved as needed. Contributions are welcome!

## Images of the led strips while tilted and prototype on a bike

![Maksimalen naklon v negativno smer](https://github.com/user-attachments/assets/6c8d6db4-83ca-49a7-aed3-02f5cb7a4a8d)
![Naklon v negativno smer](https://github.com/user-attachments/assets/9af7e136-bddf-4e37-a803-b09ccfa405fe)
![Sistem pritrjen na kolo 2](https://github.com/user-attachments/assets/7be5e0d0-bdc9-4473-b7ce-5d047a4cd0f3)
