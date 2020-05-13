# About the project

The project is designed in such a way that, the spi-driver will be probing for spi-device. We can register device/driver in any order and the probe function will get called to initialize the device/drivers

I've disabled the debug information, and printing only the important information as ALERT which can as well be controlled by adding/removing the macros.

# Pattern Observation

I have desigend a human like pattern making a retro dance based on the distance. If the distance is near, human will dance faster and vice versa.


Commands to execute:
 - Compile the program using "make" command.
 - Move the driver.ko, device.ko and app_tester file to the board.
 - Execute "insmod driver.ko" and "insmod device.ko". [This can be in any order]
 - To perform test, run "./app_tester".

 Steps executed in the test file:

 Sample Output: (LED Matrix output will be a retro dancing human)

 ************ USER OUTPUT *************

Received message: 
				About to configure matrix led 
Received message: 
				About to Initiate measurement 
Thread creation successful to receive messages...
				About to relay the display patterns 
Thread creation successful to send the patterns...
				About to relay the pattern 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 36 
Received message: 
				Received the distance measure: 53 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 31 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 36 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 42 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 40 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received the distance measure: 36 
Received message: 
				Received the distance measure: 49 
Received message: 
				Received the distance measure: 35 
Received message: 
				Received a termination command. Exiting... 
Exiting Application 
