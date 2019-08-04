#include <stdio.h>
 
#include <pigpio.h>
 
int main(int argc, char *argv[])
{ 
	if (gpioInitialise() < 0)
	{
	  fprintf(stderr, "pigpio initialisation failed\n");
	  return 1;
	}
    
    /* Set GPIO modes */
    gpioSetMode(2, PI_OUTPUT);
    gpioSetMode(3, PI_OUTPUT);
    gpioSetMode(4, PI_OUTPUT); 
    gpioSetMode(9, PI_OUTPUT); 
    gpioSetMode(10, PI_OUTPUT); 
    gpioSetMode(14, PI_OUTPUT); 
    gpioSetMode(15, PI_OUTPUT); 
    gpioSetMode(17, PI_OUTPUT); 
    gpioSetMode(18, PI_OUTPUT); 
    gpioSetMode(22, PI_OUTPUT); 
    gpioSetMode(23, PI_OUTPUT); 
    gpioSetMode(24, PI_OUTPUT);       gpioSetMode(23, PI_OUTPUT); 

    gpioSetMode(27, PI_OUTPUT);

	gpioServo(2, 1500); //leftThighX
	gpioServo(3, 1520); //leftThighZ
	gpioServo(4, 1510); //leftLegTwist
	gpioServo(9, 1750); //rightToe
	gpioServo(10, 1500); //rightFootX
	gpioServo(14, 1420); //leftKnee
	gpioServo(15, 1515); //rightThighX
	gpioServo(17, 1500); //rightThighZ
	gpioServo(18, 1467); //rightLegTwist
	gpioServo(22, 1350); //leftFootZ
	gpioServo(23, 1530); //leftFootX
	gpioServo(24, 1000); //leftToe
	gpioServo(25, 1500); //rightFootZ
	gpioServo(27, 1115); //rightKnee
	
	time_sleep(0.4);
 
	/* Stop DMA, release resources */
	gpioTerminate();

	return 0;
}
