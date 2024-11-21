YDrip - ESPHome Custom Sensor Component
=======================================

YDrip is an open-source water meter reader designed to monitor water usage and detect leaks. This software is designed to run on the YDrip hardware found at [y-drip.com](https://y-drip.com). It uses custom digital logic to monitor the magnet rotations inside of a water meter and wakes the ESP32-S3 periodically to transmit data.

Features
--------

- Configurable number of water meter magnet rotations to wake up the device
- Ultra low power when sleeping (80uA @ 4.5V)
- Easy integration with the ESPHome ecosystem

Requirements
------------

- YDrip hardware
- Brass style Water meter with internal magnet

Getting Started
---------------

1. Install ESPHome on your development machine. Visit the ESPHome documentation for installation instructions.

2. Clone or download the YDrip repository.

3. Configure the YDrip sensor in your ESPHome YAML configuration file (see ydrip.yaml), specifying the number of water meter magnet rotations to wake up the device. Larger numbers improve battery life.

   Flash the device.
   
   ```bash
   esphome  run --device /dev/ttyUSB0 ydrip.yaml
   ```

License
-------

YDrip is released under the GNU General Public License v3.0. You are free to use, modify, and distribute this software according to the terms of the license.