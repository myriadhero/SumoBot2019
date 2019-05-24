# SumoBot2019

Microcontroller:        ATMEGA328P in an Arduino Nano package
Motor drivers:          2x VNH3SP30
Time of Flight sensors: 2x VL53L0X
IR sensors:             4x TCRT5000, has both a digital and an analog output
Motors:                 2x JGB37-520B, Magnetic quadrature encoder included

___________________________
I/O

Microcontroller has these external inputs:
- 4 IR line sensors, 2 on the front and 2 on the back, 
  - IR sensor digital outputs are connected to Arduino on PCINT pins
- 2 Time of Flight sensors on the front, I2C
- 2 Encoders on the motors, PCINT as well, same bank of pins as IR
- 1 Button, in pullup config (when not pressed, reads 1)

- Additionally it controls a PWM extender module via I2C that returns back timing signal on interrupt INT0

10 Outputs are controlled through PWM extender module, namely:
- 6 motor channels, 3 left and 3 right
- 4 LEDs

To be able to initialise the two ToFs, one of them needs to be put in reset state, 
while the other ToF is given a non-default address 
- 1 digital output pin is used for that


pic of robot here:
https://www.instagram.com/p/Bx1mnPgBLqa
