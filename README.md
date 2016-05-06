# OpenDynamixel
Robotis Dynamixel support for Teensy 3x and more

This is a straight port from the Robotis-Pandora OpenCM libraries (https://github.com/robotis-pandora/ROBOTIS-OpenCM), optimised for the Teensy 3x. Current release only supports the 2.0 Protocol. When I have some hardware to test the 1.0 protocol, this will be released as a seperate branch. It does not have DE triggering implemented since this is already supported by the Teensyduino library. I will make it possible to be run by an Arduino, that do not have DE pin support, in a future release.

Currently, I only reccommend to use your Dynamixels at max. 115200 baud. I have done some tests on the Teensy using 1Mbps, but it had some failed transactions.
