# MPU6050-DMP Examples

This directory contains examples demonstrating various features of the MPU6050 sensor with its Digital Motion Processor (DMP).

## Examples

### [Basic Async](src/basic_async.rs)
Basic example showing how to:
- Initialize the sensor with async I2C
- Load and initialize the DMP firmware
- Perform sensor calibration
- Read accelerometer, gyroscope and temperature data

### [DMP Features](src/dmp_features.rs)
Example demonstrating sample rate configuration and high-frequency data sampling:
- Digital Motion Processor (DMP) initialization
- Configuring custom sample rate (100Hz)
- Reading combined 6-axis motion data
- High-frequency data sampling demonstration

### [FIFO Buffer](src/fifo_buffer.rs)
Example showing FIFO buffer operations:
- Initialize and calibrate the sensor
- Enable and configure FIFO buffer
- Monitor FIFO count and read data
- Handle buffer overflow conditions
- Process raw sensor data

### [Quaternion Data](src/quaternion.rs)
Example demonstrating 3D orientation tracking:
- Initialize and calibrate the sensor
- Configure DMP for quaternion output
- Read quaternion data for orientation
- Convert quaternions to Euler angles (roll, pitch, yaw)

### [Motion Detection](src/motion_detection.rs)
Example showing software-based motion detection:
- Initialize and calibrate the sensor
- Monitor acceleration changes
- Detect motion using configurable thresholds
- Identify periods of stillness
- Note: Uses software detection since hardware motion detection is not currently supported

## Building and Running

1. Connect your MPU6050 to a Raspberry Pi Pico:
   - SDA -> GP14
   - SCL -> GP15
   - VCC -> 3.3V
   - GND -> GND

2. Build and flash an example:
   ```bash
   # For the basic example
   cargo run --example basic_async

   # For DMP features example
   cargo run --example dmp_features

   # For FIFO buffer example
   cargo run --example fifo_buffer

   # For quaternion data example
   cargo run --example quaternion_async

   # For motion detection example
   cargo run --example motion_detection_async
   ```

## Features Overview

### Digital Motion Processor (DMP)
- Offloads complex motion processing from the main processor
- Enables more efficient and accurate motion tracking
- Provides filtered and processed sensor data

### FIFO Buffer
- Stores multiple sensor readings (up to 1024 bytes)
- Enables batch reading of sensor data
- Prevents data loss during processing
- Configurable for different sensor combinations

### Sample Rate and Filtering
- Configurable sample rates from 4Hz to 1000Hz
- Digital low-pass filter for noise reduction
- Balance between data quality and update rate
- Optimize for your specific application needs
