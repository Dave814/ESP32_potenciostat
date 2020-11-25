```
CELL ON\n 		- response "OK"
CELL OFF\n 		- response "OK"
POTENTIOSTATIC\n 	- response "OK"
GALVANOSTATIC\n 	- response "OK"
RANGE 1\n 		- response "OK"
RANGE 2\n 		- response "OK"
RANGE 3\n 		- response "OK"

DACSET xxx\n 		- 3 cfg bytes (don't forget the space!) response "OK"
DACAL\n			- response "OK"
ADCREAD\n		- response 6bytes	[0],[1],[2] - first ADC ; [3],[4],[5] - second ADC

OFFSETREAD\n		- response 6bytes 
OFFSETSAVE xxxxxx\n	- 6cfg bytes (don't forget the space!) response "OK"

DACCALGET\n		- response 6bytes 
DACCALSET xxxxxx\n 	- 6cfg bytes (don't forget the space!) response "OK"

SHUNTCALREAD\n		- response 6bytes
SHUNTCALSAVE xxxxxx\n 	- 6cfg bytes (don't forget the space!) response "OK"
```
