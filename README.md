# G5500-Rotator
Ported codes and mods for my rotator controller
---------------------------------------------------------------
Allright, so basically it's just ported code from other people, which I modded (badly) for my custom G5500 rotator controller.
Someday with more time on the TODO list is to do some sort of order and format to this mess....

SimpleG5500 is my implementation of W9KE Tom Doyle original code for a simple G5500 controller.
I tried to keep as much of his original code as possible with minimal changes to support my display and buttons control board.
Note: Older.... use the SoftSerial for latest changes

SimpleG5500-SoftSerial includes an aditional soft-serial UART port for interfacing with the ESP8266 Serial to TCP/IP bridge.
Some minor changes also to add extra functionalities and adjustments, including saving calibration in EEPROM and self-calibrating
routine in case the potentiometers of the G5500 becomes unreliable (as mine). As this one is now my current setup probably this 
one is more updated and has some bug fixes and improvements over the other one.
