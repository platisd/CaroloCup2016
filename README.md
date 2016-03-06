# CaroloCup2016 physical layer repository
The repository contains the code of the physical layer of the Team Pegasus entry in the Carolo Cup competition of 2016. 

### Description
The software was designed to control an Autonomous Vehicle, that can follow lanes, park and overtake vehicles, as tasked by the Carolo Cup 2016 competition. The vehicle is controlled by a Linux single board computer (Odroid XU3) that is mounted on top of it and accomplishes its tasks, using image processing and utilizing data from sensors mounted on the car.
The [Smartcar shield](https://github.com/platisd/smartcar_shield) library is adopted, that provides an easy to use and simple to understand interface, allowing to control the vehicle's movement and read its sensors data.
In [/CaroloCar](/CaroloCar) the arduino sketch running on the vehicle can be found. In [/testing](/testing) there are many test sketches that can be run on the vehicle, in order to verify its functions.

Moreover, in [/LED_DRIVER](LED_DRIVER) a sketch running on an ATtiny85 microcontroller can be found, which is tasked to control the various LED lights that are mounted on the vehicle, which are indicating the current operation mode of the car (turning, stopping, override, running) as per specifications of the Carolo Cup.

Finally, the repository for the application that is running in the single board computer (Odroid) and permits the accomplishment of the various tasks (lane following, parking) can be found [here](https://github.com/se-research/CaroloCup/tree/master/2016-CaroloCup). (It is unfortunately private)

### Components
- Electronic Speed Controller (ESC)
- Servo motor (Steering wheel)
- Speed encoder
- Ultrasonic sensors (SRF08)
- Infrared distance sensors (SHARP GP2D120)
- L3G4200D based gyroscope (GY-50)

### Dependencies
- [Wire library](http://arduino.cc/en/reference/Wire) used for getting data from the SRF08 ultrasonic sensors and the gyroscope, via I2C
- [Servo library](http://www.arduino.cc/en/Reference/Servo) used for controlling the ESC and the steering wheel
- [Smartcar shield library](https://github.com/platisd/smartcar_shield) enables the user to easily control the car, abstracting the low level functions

### Documentation
- Smartcar shield library [Wiki](https://github.com/platisd/smartcar_shield/wiki)

[Smartcar sensors]:https://github.com/platisd/smartcar_sensors
[AndroidCar]:https://github.com/platisd/AndroidCar

### Project documentation
- [The world's first Android autonomous vehicle](https://platis.solutions/blog/2015/06/29/worlds-first-android-autonomous-vehicle/)
- [Carolo Cup 2016 - Team Pegasus Project](https://github.com/platisd/CaroloCup2016/)

### License
GPLv3
