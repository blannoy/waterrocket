# Water Rocket electronics Arduino code
This is the code of an Arduino project that contains 
* Arduino Pro Mini
* SD card module
* 10 DOF module with L3GD20, LSM303D, BMP180 (Gyro,Accelerometer,Compass,Altimeter)
* Servo

Aim of the project is to
- detect launch
- detect highest altitude
- trigger parachute release mechanism (based on servo)
- log everything to SD

The project is based on:
- MinIMU-9-Arduino-AHRS (https://github.com/pololu/minimu-9-ahrs-arduino)
- FAT16 library (mainly for low memory consumption + easy way to append to files) (https://github.com/greiman/Fat16/tree/master/Fat16)
- Sparkfun BMP 180 library (https://github.com/sparkfun/BMP180_Breakout_Arduino_Library)

