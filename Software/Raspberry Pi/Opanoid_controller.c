#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
 
int main(int argc, char *argv[])
{ 
   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return 1;
   }
 
   /* Set GPIO modes */
   gpioServo(2, 1500); //leftThighX
   gpioServo(3, 1520); //leftThighZ
   gpioServo(4, 1510); //leftLegTwist
   gpioServo(14, 1420); //leftKnee
   gpioSetMode(22, PI_OUTPUT); //leftFootZ
   gpioSetMode(23, PI_OUTPUT); //leftFootX
   gpioSetMode(24, PI_OUTPUT); //leftToe
   gpioServo(15, 1515); //rightThighX
   gpioServo(17, 1500); //rightThighZ
   gpioServo(18, 1467); //rightLegTwist
   gpioSetMode(9, PI_OUTPUT); //rightToe
   gpioSetMode(10, PI_OUTPUT); //rightFootX
   gpioSetMode(25, PI_OUTPUT); //rightFootZ
 
   FILE *myFile;
   myFile = fopen("data_log_full_legs_walk.txt", "r");
   int i;
   int ch, character = 0, line = 0;
   
   /* Count number of values in log file */
   while ((ch = fgetc(myFile)) != EOF)
        {
            character++;
            if (ch == '\n')
                line++;
        }
        
   rewind(myFile); // Return to start of file to prepare for scanning
   
   /* Declare size of arrays based on number of values in log file */
   float numberArray0[1][line];
   float numberArray1[1][line];
   float numberArray2[1][line];
   float numberArray3[1][line];
   float numberArray4[1][line];
   float numberArray5[1][line];
   float numberArray6[1][line];
   float numberArray7[1][line];
   float numberArray8[1][line];
   float numberArray9[1][line];
   float numberArray10[1][line];
   float numberArray11[1][line];
   float numberArray12[1][line];
   float numberArray13[1][line];
   int leftThighX[line];
   int leftThighXRaw[line];
   int leftThighZ[line];
   int leftThighZRaw[line];   
   int leftLegTwist[line];
   int leftLegTwistRaw[line];   
   int leftKnee[line];
   int leftKneeRaw[line];   
   int rightThighX[line];
   int rightThighXRaw[line];   
   int rightThighZ[line];
   int rightThighZRaw[line];   
   int rightLegTwist[line];
   int rightLegTwistRaw[line];   
   int rightKnee[line];
   int rightKneeRaw[line];   
   int leftFootX[line];   
   int leftFootZ[line];   
   int leftToe[line];   
   int rightFootX[line];   
   int rightFootZ[line];   
   int rightToe[line];
   int leftThighXZero = 1455;
   int leftThighZZero = 1520;
   int leftLegTwistZero = 1510;
   int leftKneeZero = 1420;
   int rightThighXZero = 1515;
   int rightThighZZero = 1500;
   int rightLegTwistZero = 1467;
   int rightKneeZero = 1115;
   int leftFootXZero = 1530;
   int leftFootZZero = 1350;
   int leftToeZero = 1000;
   int rightFootXZero = 1500;
   int rightFootZZero = 1500;
   int rightToeZero = 1750;
   
   
   /* Scan values into numberArray */
   for (i = 0; i < line + 1; i++)
   {
       fscanf(myFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f", &numberArray0[0][i], &numberArray1[0][i], &numberArray2[0][i], &numberArray3[0][i], &numberArray4[0][i], &numberArray5[0][i], &numberArray6[0][i], &numberArray7[0][i], &numberArray8[0][i], &numberArray9[0][i], &numberArray10[0][i], &numberArray11[0][i], &numberArray12[0][i], &numberArray13[0][i]);
       /* Convert values into usable range for servo and put into finalArray */
       leftThighXRaw[i] = ((numberArray0[0][i]/140) * 950) + leftThighXZero;
       leftThighX[i] = (leftThighXRaw[i] + leftThighXRaw[i - 1] + leftThighXRaw[i - 2] + leftThighXRaw[i - 3])/4;
       if(leftThighX[i] > (leftThighXZero + 210)) {
		   leftThighX[i] = leftThighXZero + 210;
	   }
       else if(leftThighX[i] < leftThighXZero) {
		   leftThighX[i] = leftThighXZero;
	   }
	   
       leftThighZRaw[i] = -((numberArray1[0][i]/90) * 950) + leftThighZZero;
       leftThighZ[i] = (leftThighZRaw[i] + leftThighZRaw[i - 1] + leftThighZRaw[i - 2] + leftThighZRaw[i - 3])/4;
       if(leftThighZ[i] > (leftThighZZero + 700)) {
		   leftThighZ[i] = leftThighZZero + 700;
	   }
       else if(leftThighZ[i] < (leftThighZZero - 700)) {
		   leftThighZ[i] = leftThighZZero - 700;
	   }
       
	   leftLegTwistRaw[i] = ((numberArray2[0][i]/90) * 950) + leftLegTwistZero;
	   leftLegTwist[i] = (leftLegTwistRaw[i] + leftLegTwistRaw[i - 1] + leftLegTwistRaw[i - 2] + leftLegTwistRaw[i - 3])/4;
	   if(leftLegTwist[i] > (leftLegTwistZero + 100)) {
		   leftLegTwist[i] = leftLegTwistZero + 100;
	   }
       else if(leftLegTwist[i] < (leftLegTwistZero - 400)) {
		   leftLegTwist[i] = leftLegTwistZero - 400;
	   }
	   
	   leftKneeRaw[i] = ((numberArray3[0][i]/90) * 950) + leftKneeZero;
	   leftKnee[i] = (leftKneeRaw[i] + leftKneeRaw[i - 1] + leftKneeRaw[i - 2] + leftKneeRaw[i - 3])/4;
	   if(leftKnee[i] > (leftKneeZero + 1000)) {
		   leftKnee[i] = leftKneeZero + 1000;
	   }
       else if(leftKnee[i] < leftKneeZero) {
		   leftKnee[i] = leftKneeZero;
	   }
	   
	   rightThighXRaw[i] = -((numberArray4[0][i]/140) * 950) + rightThighXZero;
	   rightThighX[i] = (rightThighXRaw[i] + rightThighXRaw[i - 1] + rightThighXRaw[i - 2] + rightThighXRaw[i - 3])/4;
	   if(rightThighX[i] > rightThighXZero) {
		   rightThighX[i] = rightThighXZero;
	   }
       else if(rightThighX[i] < (rightThighXZero - 210)) {
		   rightThighX[i] = rightThighXZero - 210;
	   }
	   
	   rightThighZRaw[i] = ((numberArray5[0][i]/90) * 950) + rightThighZZero;
	   rightThighZ[i] = (rightThighZRaw[i] + rightThighZRaw[i - 1] + rightThighZRaw[i - 2] + rightThighZRaw[i - 3])/4;
	   if(rightThighZ[i] > (rightThighZZero + 700)) {
		   rightThighZ[i] = rightThighZZero + 700;
	   }
       else if(rightThighZ[i] < (rightThighZZero - 700)) {
		   rightThighZ[i] = rightThighZZero - 700;
	   }
	   
	   rightLegTwistRaw[i] = -((numberArray6[0][i]/90) * 950) + rightLegTwistZero;
	   rightLegTwist[i] = (rightLegTwistRaw[i] + rightLegTwistRaw[i - 1] + rightLegTwistRaw[i - 2] + rightLegTwistRaw[i - 3])/4;
	   if(rightLegTwist[i] > (rightLegTwistZero + 400)) {
		   rightLegTwist[i] = rightLegTwistZero + 400;
	   }
       else if(rightLegTwist[i] < (rightLegTwistZero - 100)) {
		   rightLegTwist[i] = rightLegTwistZero - 100;
	   }
	   
	   rightKneeRaw[i] = -((numberArray7[0][i]/90) * 950) + rightKneeZero;
	   rightKnee[i] = (rightKneeRaw[i] + rightKneeRaw[i - 1] + rightKneeRaw[i - 2] + rightKneeRaw[i - 3])/4;
	   if(rightKnee[i] > rightKneeZero) {
		   rightKnee[i] = rightKneeZero;
	   }
       else if(rightKnee[i] < (rightKneeZero - 1000)) {
		   rightKnee[i] = rightKneeZero - 1000;
	   }
	   
	   leftFootX[i] = -((numberArray8[0][i]/90) * 950) + leftFootXZero;
	   if(leftFootX[i] > (leftFootXZero + 140)) {
		   leftFootX[i] = leftFootXZero + 140;
	   }
	   else if(leftFootX[i] < (leftFootXZero - 160)) {
		   leftFootX[i] = leftFootXZero - 160;
	   }
	   
	   leftFootZ[i] = -((numberArray9[0][i]/140) * 950) + leftFootZZero;
	   if(leftFootZ[i] > (leftFootZZero + 400)) {
		   leftFootZ[i] = leftFootZZero + 400;
	   }
	   else if(leftFootZ[i] < (leftFootZZero - 180)) {
		   leftFootZ[i] = leftFootZZero - 180;
	   }
	   
	   leftToe[i] = ((numberArray10[0][i]/90) * 950) + leftToeZero;
	   if(leftToe[i] > (leftToeZero + 700)) {
		   leftToe[i] = leftToeZero + 700;
	   }
	   else if(leftToe[i] < (leftToeZero - 130)) {
		   leftToe[i] = leftToeZero - 130;
	   }
	   
	   rightFootX[i] = -((numberArray11[0][i]/90) * 950) + rightFootXZero;
	   if(rightFootX[i] > (rightFootXZero + 300)) {
		   rightFootX[i] = rightFootXZero + 300;
	   }
	   else if(rightFootX[i] < (rightFootXZero - 170)) {
		   rightFootX[i] = rightFootXZero - 170;
	   }	   
	   
	   rightFootZ[i] = ((numberArray12[0][i]/140) * 950) + rightFootZZero;
	   if(rightFootZ[i] > (rightFootZZero + 80)) {
		   rightFootZ[i] = rightFootZZero + 80;
	   }
	   else if(rightFootZ[i] < (rightFootZZero - 190)) {
		   rightFootZ[i] = rightFootZZero - 190;
	   }	
	   
	   rightToe[i] = -((numberArray13[0][i]/90) * 950) + rightToeZero;
	   if(rightToe[i] > (rightToeZero + 350)) {
		   rightToe[i] = rightToeZero + 350;
	   }
	   else if(rightToe[i] < (rightToeZero - 750)) {
		   rightToe[i] = rightToeZero - 750;
	   }	
   }	    
   /* Move servo based on values in finalArray */
   for (i = 0; i < line + 1; i = i + 25)
   {	  
	    gpioServo(2, leftThighX[i]);
	    gpioServo(3, leftThighZ[i]);
	    //gpioServo(4, leftLegTwist[i]);
	    gpioServo(14, leftKnee[i]);
	    gpioServo(15, rightThighX[i]);
	    gpioServo(17, rightThighZ[i]);
	    //gpioServo(18, rightLegTwist[i]);
	    gpioServo(27, rightKnee[i]);
	    gpioServo(22, leftFootZ[i]);
	    gpioServo(23, leftFootX[i]);
	    gpioServo(24, leftToe[i]);
	    gpioServo(9, rightToe[i]);
	    gpioServo(10, rightFootX[i]);
	    gpioServo(25, rightFootZ[i]);
	    printf("%f %f %f %f %f %f %f \n", numberArray7[0][i], numberArray8[0][i], numberArray9[0][i], numberArray10[0][i], numberArray11[0][i], numberArray12[0][i], numberArray13[0][i]);
	    time_sleep(0.03;	
   }
   
	
   /* Stop DMA, release resources */
   gpioTerminate();
 
   return 0;
}
