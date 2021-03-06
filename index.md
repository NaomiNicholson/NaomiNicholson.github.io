## Introduction

Godspeed is a robot that follows running track lines and uses a fan to cool the user down. It additionally maintains a set distance between itself and the user and avoids collisions with oncoming obstructions. This robot allows the user to enjoy a cool walk along a terrain marked with a line, such as the Georgia Tech tracks.

## Design Process

This project started off with three unique goals:
1. The device would need to adjust itself and follow a predetermined line.
2. The device would need to detect incoming collisions and avoid them.
4. The device would need to have user-ajustable fan speed and distance between itself and the user through Bluetooth.

###### 1. Line Following:

Through designing the robot's line following capabilities, we experimented with different types of sensors. At the beginning of this process' implemention we were using an RGB sensor to detect the changes in colors on the floor. We decided to go with a IR reflective sensor instead after finishing the algorithm as the RGB sensor was updating the change from black to white, or vice versa, too slowly to be efficient. The IR reflective sensor proved to be more sensitive and reactive to the change in color we were observing.

Additionally, we had some issues earlier on with the speed at which the robot would turn when trying to adjust itself. We were initially setting the speeds in the code to be a set value speed for "slow" or "fast". This led to instances where the robot would turn too fast and completely derail itself from the track. We solved this problem by setting the speeds to the same numerical values but setting "slow" as negative. The H-Bridge drivers allowed us to reverse the gearmotors as necessary and this change provided more efficient movement during the line following.

###### 2. Crash Detection:

The crash detection functionality was accomplished using an IR distance sensor. Although requiring a lot of troubleshooting, this functionality was relatively easy to setup. The process to design this functionality mainly focused on testing the sensor and figure out what values received from the analog sensor would represent approximately 10 cm at which point the robot should stop.

###### 3. Bluetooth User-Adjustable Fan Speed and Leading Distance:

The bluetooth capabilities of the fan and leading distance were straightforward to implement, thanks to Adafruit's Bluefruit module and application. 

The fan was tested with different power sources during the design process and we concluded that it would require 9V in order to work. 6V and 5V batteries used for other components of the system did not provide enough power and so it requires its own battery. 

The leading distance was implemented using a sonar sensor. There were some difficulties interpreting the sonar's readings at the beginning, but after much troubleshooting we were able to reasonably integrate the values to maintain four different user-picked leading distances between the user and the robot. The only issue this functionality still possesses comes into play when the robot meets a sharp edge on the track. At this point, the robot may not be aiming the sonar towards the user and this creates room for discrepencies with the expected leading distance behavior of the robot.

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

<img width="839" alt="POWERPNT_2022-05-02_14-08-40" src="https://user-images.githubusercontent.com/104459763/166301408-706779a7-02b5-480d-9b2f-7d52884f4858.png">


**Front Gearmotor Subsystem:**

| Mbed | H-Bridge Driver | Front Left Motor | Front Right Motor | 6V Battery Pack |
|-------|--------|---------|---------|---------|
| GND | GND |  |  | - |
|  | Vm |  |  | + |
| Vout | Vcc |  |  |  |
| Pin 5 | AI1 |  |  |  |
| Pin 6 | AI2 |  |  |  |
| Pin 11 | BI1 |  |  |  |
| Pin 12 | BI2 |  |  |  |
|  | AO1 | + |  |  |
|  | AO2 | - |  |  |
|  | BO1 |  | + |  |
|  | BO2 |  | - |  |
| Pin 21 | pwma |  |  |  |
| Pin 23 | pwmb |  |  |  |
| Vout | stby |  |  |  |

**Back Gearmotor Subsystem:**

| Mbed | H-Bridge Driver | Back Left Motor | Back Right Motor | 6V Battery Pack |
|-------|--------|---------|---------|---------|
| GND | GND |  |  | - |
|  | Vm |  |  | + |
| Vout | Vcc |  |  |  |
| Pin 7 | AI1 |  |  |  |
| Pin 8 | AI2 |  |  |  |
| Pin 30 | BI1 |  |  |  |
| Pin 29 | BI2 |  |  |  |
|  | AO1 | + |  |  |
|  | AO2 | - |  |  |
|  | BO1 |  | + |  |
|  | BO2 |  | - |  |
| Pin 22 | pwma |  |  |  |
| Pin 24 | pwmb |  |  |  |
| Vout | stby |  |  |  |

**Brushless Motor Subsystem:**

| Mbed | Brushless Motor | 9V Battery Pack |
|-------|--------|--------|
|  | Vcc | + |
| GND | GND | - |
| Pin 25 | Signal |  |

**Sensors Subsystem:**

| Mbed | Left IR Reflective Sensor | Right IR Reflective Sensor | Left IR Distance Sensor | Right IR Distance Sensor | Sonar Sensor |
|-------|--------|---------|---------|---------|---------|
| Vout | Vcc | Vcc |  |  |  |
| Vu |  |  | Vcc | Vcc | Vcc |
| GND | GND | GND | GND | GND | GND |
| Pin 16 | OUT |  |  |  |  |
| Pin 17 |  | OUT |  |  |  |
| Pin 18 |  |  | OUT |  |  |
| Pin 19 |  |  |  | OUT |  |
| Pin 13 |  |  |  |  | Trig |
| Pin 14 |  |  |  |  | Echo |

**Mbed Subsystem:**

| Mbed | Bluefruit | 6V Battery Pack | 5V Battery Pack |
|-------|--------|---------|---------|
| Vin |  |  | + |
| GND | GND, CTS | - | - |
| Pin 27 | TXO |  |  |
| Pin 28 | RXI |  |  |
|  | Vin | + |  |

## Code

The following block contains our final iteration of the code. This multithreaded application has a thread for each of the following functionalities:
1. Robot's movement depending on the IR reflective sensors' measurements.
2. Crash detection depending on the IR distance sensors' measurements.
3. Leading distance of the robot to the user depending on the sonar sensor's measurements.
4. Leading distance of the robot as controlled by Bluetooth depending on the Bluetooth input provided by the user.
5. Brushless motor's (the fan) speed depending on the Bluetooth input provided by the user.

The block may need to be scrolled through from left to right to read the longer lines of code.

```
#include "mbed.h"
#include "Motor.h"
#include "rtos.h"
#include "ultrasonic.h"
#include "Servo.h"

#define BLACK_THRESHOLD 0.5
#define MAX_SERVO 0.2
#define INCREMENT 0.025

AnalogIn ir_r (p17);
AnalogIn ir_l (p16);

AnalogIn front_right_sens (p18);
AnalogIn front_left_sens (p19);
Motor left_front(p21, p5, p6);
Motor left_back(p22, p7, p8);
Motor right_front(p23, p11, p12);
Motor right_back(p24, p30, p29);
Servo fan(p25);
Serial pc(USBTX, USBRX); 
Serial blue(p28,p27);
void update_dist(int);
ultrasonic back_sens(p13, p14, .2, 1, &update_dist); 
volatile float speed;
volatile bool obstacle = false;
volatile bool turning = false;
volatile int leading_dist = 100;
volatile int curr_distance;
volatile float current_fan_speed = 0.0f;
void move_robot_thread(){
    
    // Read data from color sensor (Clear/Red/Green/Blue)
    float ir_r_read;
    float ir_l_read;
    speed = 1.0;

  while(1) {
    while(obstacle);
    ir_r_read = ir_r.read();
    ir_l_read = ir_l.read();
    
    if(ir_l_read >= BLACK_THRESHOLD && ir_r_read < BLACK_THRESHOLD) { // if left detects black & right detects white, turn left
        turning = true;
        // speed_left = -0.4;
        // speed_right = 0.4;
        left_front.speed(-2*speed);
        left_back.speed(-2*speed);
        right_front.speed(2*speed);
        right_back.speed(2*speed);
    }
    else if(ir_l_read < BLACK_THRESHOLD && ir_r_read >= BLACK_THRESHOLD) { // if right detects black & left detects white, turn right
        // speed_right = -0.4;
        // speed_left = 0.4;
        left_front.speed(2*speed);
        left_back.speed(2*speed);
        right_front.speed(-2*speed);
        right_back.speed(-2*speed);
    }
    else { // else go straight
        // speed_left = 0.4;
        // speed_right = 0.4;
        left_front.speed(speed);
        left_back.speed(speed);
        right_front.speed(speed);
        right_back.speed(speed);
    }
    
    Thread::wait(100);
  }
}

void crash_detect_thread() {
    while(1) {
//        pc.printf("right sensor: %f\n\r", front_right_sens.read());
//        pc.printf(" left sensor: %f\n\r", front_left_sens.read());
        if((front_right_sens.read()*3.3) >= 3 || (front_left_sens.read()*3.3) >= 3) {
            obstacle = true;
            left_front.speed(0);
            left_back.speed(0);
            right_front.speed(0);
            right_back.speed(0);
//            Thread::wait(100);
        }
        else if(obstacle) {
            while(front_right_sens.read() > 0.45 || front_left_sens.read() > 0.45);
            obstacle = false;
        }
//        else {
//            obstacle = false;
//        }
        Thread::wait(100);
    }
}

 void leading_distance_picking_thread() {
    char bnum=0;
    char bhit=0;
     while(1) {
         while(!blue.readable()){Thread::wait(200);}
         if (blue.getc()=='!') {
             if (blue.getc()=='B') { //button data packet
                 bnum = blue.getc(); //button number
                 bhit = blue.getc(); //1=hit, 0=release
                 if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                     switch (bnum) {
                         case '1': // 1 button
                             if (bhit=='1') {
                                 leading_dist = 100;
                             }
                             break;
                         case '2': //button 6 down arrow
                             if (bhit=='1') {
                                 leading_dist = 130;
                             } 
                             break;
                         case '3': //button 6 down arrow
                             if (bhit=='1') {
                                 leading_dist = 150;
                             } 
                             break;
                         case '4': //button 6 down arrow
                             if (bhit=='1') {
                                 leading_dist = 180;
                             } 
                             break;
                         default:
                             break;
                     }
                 }
             }
         }
     Thread::wait(500);
     }
 }
void update_dist(int distance) {
//    pc.printf(" distance in handler: %d\r\n", distance);
    curr_distance = distance;
}
 void lead_distance_thread() {
     while(1) {
         pc.printf("Distance: %d\r\n", curr_distance);
         pc.printf(" Speed: %f\r\n", speed);
         if(!turning) {
             if(curr_distance < leading_dist-20) {
                 if(speed + 0.2 <= 1.2) {
                     speed+=0.2;
                    }
             }
             if(curr_distance > leading_dist + 20) {
                 Thread::wait(250);
                 if(curr_distance > leading_dist + 20) {
                    if(speed - 0.2 >= 0.0) {
                        speed-=0.2;
                    }
                }
             }
         }
         Thread::wait(300);
     }
 }

void fan_thread() {
    fan = current_fan_speed;
    char bnum=0;
    char bhit=0;
    while(1) {
        while(!blue.readable()){Thread::wait(200);}
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data packet
                bnum = blue.getc(); //button number
                bhit = blue.getc(); //1=hit, 0=release
                if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    switch (bnum) {
                        case '5': //button 5 up arrow
                            if (bhit=='1') {
                                if(current_fan_speed + INCREMENT <= MAX_SERVO) {
                                    current_fan_speed += INCREMENT;
                                    fan = current_fan_speed;
                                }
                            }
                            break;
                        case '6': //button 6 down arrow
                            if (bhit=='1') {
                                if(current_fan_speed - INCREMENT >= 0) {
                                    current_fan_speed -= INCREMENT;
                                    fan = current_fan_speed;
                                }
                            } 
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        Thread::wait(500);
    }

}
int main() {
    pc.baud(9600);
    back_sens.startUpdates();
    Thread fan_move;
    Thread move_robot;
    Thread crash_detect;
    Thread leading_distance;
    Thread pick_speed;
    crash_detect.start(crash_detect_thread);
    move_robot.start(move_robot_thread);
    leading_distance.start(lead_distance_thread);
    pick_speed.start(leading_distance_picking_thread);
    fan_move.start(fan_thread);
    while(1){
        back_sens.checkDistance();
//        Thread::wait(300);
    }
}
```

## Final Results

[**Line Following**](https://youtu.be/VvSngZXzXIo)
<iframe width="560" height="315" src="https://www.youtube.com/embed/VvSngZXzXIo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

[**Bluetooth Capabilities and Crash Detection**](https://youtu.be/qbLFUiYAt4Q)
<iframe width="560" height="315" src="https://www.youtube.com/embed/qbLFUiYAt4Q" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


As a conclusion to our testing, we found that the robot worked best and more consistently on a semi-elliptical track. This setup allows for consistent line following, sonar readings, and obstacle detection. The full elliptical track setup we tested, which can be seen in the Line Following video above, did not work as well. In fact, other than the obstacle detection working as intended, there were some difficulty with the SONAR readings and the line following would only work 50% of the time.

We believe that the following changes would lead to a smoother and reliable experience:
1. Utilizing PID control with additional IR Reflective sensors to detect black and white on the ground. This would allow for smoother control. The error signal could be based on the exact readings of each IR sensor.
2. Redesigning the robot chassis so that it does not sag inwards with additional weight. This will allow the wheels to have enough friction to turn properly.
3. Having a collapsible pole will make the robot easier to transport.
4. Having a GUI for the user to interact with robot settings

## References

[1] A. Jarek, ???Line Follower Arduino,??? ForbiddenBit, 10-Jun-2020. [Online]. Available: https://forbiddenbit.com/en/arduino-projects/line-flower-arduino/. [Accessed: 09-Apr-2022]. 

[2] J. Hamblen, ???Adafruit Bluefruit LE UART Friend,??? Mbed. [Online]. Available: https://os.mbed.com/components/Adafruit-Bluefruit-LE-UART-Friend/. [Accessed: 11-Apr-2022]. 
