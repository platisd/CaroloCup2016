# LED DRIVER
The LED driver board, is based on an ATTiny85 microcontroller and is used in order to drive the LEDs that can be found on the vehicle and particularly the direction indicators. In the [/res](res) folder, you can find the board's schema, how it looks like and also the Eagle files if you want to modify it or fabricate it yourself. Moreover, if you want just to reproduce the board, you can order it from [OSH Park](https://oshpark.com/shared_projects/Hk4up6hJ).

### Materials used
* 6 screw terminals ([Sparkfun](https://www.sparkfun.com/products/8432))
* 4 NPN transistors (BC547)
* 4 1.5KΩ resistors
* 1 ATtiny85-20PU microcontroller
* 1 socket 8 DIP (optional but strongly recommended)

### API
The LED driver board receives serial commands, through pin 2, of the ATtiny85 microcontroller, at 9600 BAUD rate. The following characters, when received, will trigger the various modes of the vehicle:
* 'l': The left side LEDs start blinking at a frequency of 1 Hz
* 'r': The right side LEDs start blinking at a frequency of 1 Hz
* 'm': The RC mode indicator starts blinking at a frequency of 1Hz
* 's': The stop LEDs are turned on
* 'p': The left and right side LEDs blink 3 times, at a frequency of 3Hz
Any other character, causes the LED lights to be turned off.

In order for a mode to be initiated, the equivalent command needs to be sent _once_. For example, if the left blinkers need to be turned on, then the character 'l' needs to be sent just once and the onboard microcontroller will take care of the rest.

### Documentation
[Simple LED driver board](https://platis.solutions/blog/2015/10/15/simple-led-driver-board/)
