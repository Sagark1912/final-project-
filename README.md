
# FInal Project in G14 ES & IOT Batch
***

# Patient Health Tracking System

## Overview

The Patient Health Tracking System is a wearable IoT-based solution designed to continuously monitor a person’s heart rate, blood oxygen (SpO₂), and movement for fall detection. In emergency situations—such as abnormal vital signs or a detected fall—the device automatically sends an SMS alert with the patient’s real-time location to a caregiver or emergency contact. All readings and alerts are also shown on a small OLED display for the user’s convenience.

## Features

- Continuous heart rate and SpO₂ monitoring
- Accelerometer-based fall detection
- Real-time GPS location tracking
- Automatic SMS alerts and emergency calling via GSM
- Clear OLED display of live health data and alerts
- Power-saving updates (refreshes display only when values change)

## Hardware Used

- Seeed XIAO ESP32S3 Sense (microcontroller)
- MAX30102 pulse oximeter sensor (heart rate and SpO₂)
- LIS3DH accelerometer (movement and fall detection)
- NEO-6M GPS module (location)
- SIM800C GSM module (SMS and call)
- OLED display (health information and alerts)
- Breadboard, jumper wires, and USB-C power supply

## How It Works

1. The device constantly monitors heart rate, SpO₂, and motion.
2. If it detects abnormal vital signs or a fall, it gathers GPS position and current time.
3. It sends an SMS with the emergency details and Google Maps link to preset contacts.
4. Health readings and system status are visible on the OLED display.
5. The system is powered by a battery (with charging via USB-C).

## Getting Started

1. **Hardware assembly**: Connect all modules to the ESP32 as per schematic.
2. **Software setup**: Flash the provided code onto the ESP32 using Arduino IDE.
3. **SIM & power**: Insert a SIM card into the GSM module and connect USB-C or battery.
4. **Operation**: On power-up, the device will initialize and begin monitoring automatically.
5. **Testing**: Check OLED for output and verify SMS alerts with simulated emergencies.

## Example Emergency SMS

```
Urgent medical attention required
abnormal heart rate and SpO₂ detected.
Latitude: <value>
Longitude: <value>
Link: http://maps.google.com/maps?q=<lat>,<long>
Date: DD/MM/YYYY
Time: HH:MM:SS
```

## Images

Included photos of assembled hardware, OLED screen outputs, and sample SMS alerts.

## Future Improvements

- Add support for extra health parameters (blood pressure, temperature, etc.)
- Smarter alerts based on AI and personal patterns
- Mobile app integration and cloud data storage
- Smaller and more robust wearable design
- Medical device certification

## VIDEO LINK

https://drive.google.com/file/d/1TmWNfQIM6X9nfRNrqaurHN4Mw8nZf9Lj/view?usp=sharing 
