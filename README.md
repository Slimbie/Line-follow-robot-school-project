#line follower without down force at high speed and mapping over several runs
how does it work the linefollower has 2 modes a mapping mode which in he goes slower and reads the map very accurate and a high speed mode in which he uses this data to predict turns and go even faster 
it also has accceleration checks to slow down in turns or corners and it also has an ultrasonedistance sensor to read the distancee to object to avoid colision
it is based around a esp32 using the dual cores one for sensor reading and the other for controls there is a start/stop button the 3th push sends out serial data like a map from the course for extra debuging info
and the reading sensor is a simple 8sensor ir array for like 2 euros i also made a pcb to avoid any trouble with wiring 
if you are currious about how to connect just look at the config.h file it is very clear in the connections etc 
