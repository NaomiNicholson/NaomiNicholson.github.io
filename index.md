## Introduction

Godspeed is a robot that follows running track lines and uses a fan to cool the user down. It additionally maintains a set distance between itself and the user and avoids collisions with oncoming obstructions.

## Design Process

How we got there.

## How to Setup This Robot

###### Necessary Parts:

- 1 [ARM Mbed](https://www.sparkfun.com/products/9564)
- 1 [Bluefruit LE UART Friend - BLE](https://www.adafruit.com/product/2479)
- 2 [H-Bridge Motor Drivers](https://www.sparkfun.com/products/14450)
- 2 [DC Barrel Jack Adapters](https://www.sparkfun.com/products/10811)
- 4 [Rubber Tire Wheels](https://www.sparkfun.com/products/13259)
- 4 [Gearmotors](https://www.sparkfun.com/products/13302)
- 2 [IR Reflective Sensor Modules](https://www.amazon.com/dp/B07FJLMLVZ?psc=1&ref=ppx_yo2ov_dt_b_product_details)
- 2 [IR Distance Sensors](https://www.sparkfun.com/products/242)
- 2 4xAA Battery Holder
- 8 AA Batteries
- 1 [Brushless Motor](https://www.amainhobbies.com/blade-torrent-110-11047600kv-fpv-racing-motor-blha1024/p633058) and Propeller
- 1 5 V Battery Pack
- 1 9 V Battery Pack
- 1 [Custom 3D Printed Chassis](https://github.com/NaomiNicholson/NaomiNicholson.github.io/files/8566920/4180_Chassis.zip)

<img width="473" alt="BottomChassis" src="https://user-images.githubusercontent.com/104459763/165387789-0ad63413-8375-432d-972e-4dc1cfaeee35.PNG"> <img width="391" alt="TopChassis" src="https://user-images.githubusercontent.com/104459763/165394988-1d8c31b5-9632-44e5-aa03-bf9e54e58a8e.PNG">

###### Power Requirements:

The gearmotors are powered by 2 different 4xAA battery packs (6 V each).

The brushless motor is powered by a 9 V battery pack.

The mbed is powered by a 5 V battery pack.

###### Wiring:

**Gearmotor Subsystem:**
| Mbed | Front Right Gearmotor | Back Right Gearmotor | Front Left Gearmotor | Back Left Gearmotor |
|-------|--------|---------|---------|---------|
| Pin 9 | Vin | Vin | Vin | Vin |
| Pin 10 | GND | GND | GND | GND |
| Pin 20 | OUT | OUT | OUT | OUT |

**Brushless Motor Subsystem:**
| Mbed | Brushless Motor |
|-------|--------|
| Pin 9 | Vin |
| Pin 10 | GND |
| Pin 20 | OUT |

**Mbed Subsystem:**
| Mbed | IR Reflective Sensor | IR Distance Sensor | Bluefruit | Front H-Bridge | Back H-Bridge |
|-------|--------|---------|---------|---------|---------|
| Pin 9 | Vin | Vin | Vin | Vin | Vin |
| Pin 10 | GND | GND | GND | GND | GND |
| Pin 20 | OUT | OUT | OUT | OUT | OUT |

## Code

```
code snippet here
```

## Final Results

Pictures and demo videos here


## References

All URLs referenced during the project
