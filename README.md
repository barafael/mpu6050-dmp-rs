# `mpu6050-dmp`

Platform-independent i<sup>2</sup>c Driver for the InvenSense [MPU-6050 motion-processor](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/).

## Fork of [drogue-mpu-6050](https://github.com/drogue-iot/drogue-mpu-6050)
Reasons/differences:
* Dependency on `drogue-embedded-timer` and `embedded-time` removed
  - Timers are not required in regular operation, just for setup
  - drogue-embedded-timer is out of sync with current versions of embedded-time as of time-of-fork
* Dependency on `log` removed
* Slight difference in conversion of Quaternion to YPR (no `pitch -= PI`)
* Various fixes for e.g. clippy lints (also PR'd upstream)

## [Demo Project for STM32F1](https://github.com/barafael/mpu6050-dmp-demo-f1)
## [Demo Project for STM32F4](https://github.com/barafael/mpu6050-dmp-demo-f4)

## On-Chip DMP 'Digital Motion Processor'

This driver can load the appropriate firmware for quaternion-based on-chip DMP processing.

## Setup

### i<sup>2</sup>c

Initialize your i<sup>2</sup>c bus:

```rust
let gpiob = dp.GPIOB.split();
let scl = gpiob
    .pb8
    .into_alternate()
    .internal_pull_up(true)
    .set_open_drain();
let sda = gpiob
    .pb9
    .into_alternate()
    .internal_pull_up(true)
    .set_open_drain();
let i2c = dp.I2C1.i2c((scl, sda), 400.kHz(), &clocks);
```

### MPU driver

```rust
let sensor = Mpu6050::new(i2c, Address::default()).unwrap();
```

### Temperature Measurement

The MPU-6050 includes an on-chip temperature sensor. Temperature readings are available through the `temperature()` method when the `temperature` feature is enabled (enabled by default):

```rust
// Get temperature reading
let temp = sensor.temperature().unwrap();
println!("Temperature: {}°C", temp.celsius());
```

Setting up the DMP requires temporary exclusive access to a blocking delay implementation.
The `initialize_dmp(&mut delay)` method is provided to set up reasonable configurations and load the DMP firmware into the processor.

```rust
sensor.initialize_dmp(&mut delay).unwrap();
```

If using the advanced on-chip DMP logic, the FIFO will contain 28-byte packets of quaternion and other data.

The first 16 bytes are quaternions, which can be constructed using the `Quaternion` class.

```rust
let len = sensor.get_fifo_count().unwrap();
if len >= 28 {
    let buf = sensor.read_fifo(&mut buf).unwrap();
    let q = Quaternion::from_bytes(&buf[..16]).unwrap().normalize();
    let ypr = YawPitchRoll::from(quat);
    rprintln!("{:?}", ypr);
}
```
