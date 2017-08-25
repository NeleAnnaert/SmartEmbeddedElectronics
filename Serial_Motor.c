/*
 * Serial communication and motor driver
 * Author : Beke Michiel
 * Description:
 * This program listens on the UART serial communication
 * according to the instructions the left and right track
 * will move with a given speed in a given direction
 */
 
// Includes
#include "RP6RobotBaseLib.h"		//The standard RP6 library
#include "Serial_Motor.h"		//The definition of the used struct

//Define variables
#define LEDFLASH 10		//Number of LED flashes
#define BUFFERSIZE 10		//The amount of chars read in one go
#define TEMPSIZE 3		//Used for the combination of 3 chars to form 1 value

/*
 * Communication part
 * Interrupt: p.80 of the manual --> the reception of uart data is automatically
 * interrupt based when data is received it is stored in a circular buffer
 * after 1 move or rotate, check for new data in the buffer
 * readChar will return the next availlable character in the buffer AND delete it
 *
 * Expected data: 	1 char for the type ('m'/'r')
 * 					1 char for the direction ('f'/'b'/'l'/'r')
 * 					3 chars for the speed ('200'/'005'/'016')
 * 					3 chars for the angle ('180'/'009"/'090')
 * 					3 chars for the distance ('964'/'007'/'067')
 * 					newline ('\n')
*/

void readString(char *buffer)
{
	int pos = 0;
	char check = '\0';
	memset(buffer,'\0',BUFFERSIZE);		//Clear the buffer
	
	while((check = readChar()))		//Read till newline
	{
		buffer[pos] = check;
		pos++;
		if(check == '\n')
		{
			break;
		}
	}
}

int recCommand(struct command *comm)
{
	setLEDs(0b000000);		//Reset the LED's
	char buffer[10];
	char temp[3];		//Temporarily store the 3 chars of a value
	int data = getBufferLength();		//Return to newData of the main function
	
	if(data != 0)		//Data in the buffer
	{
		//Type and direction are ascii --> just read
		//Speed, angle and distance are integer decimal
		readString(buffer);		//Read the buffer
		
		comm->type = buffer[0];		//Set the type
		comm->direction = buffer[1];		//Set the direction
		
		temp[0] = buffer[2];
		temp[1] = buffer[3];
		temp[2] = buffer[4];
		comm->speed = atoi(temp);		//Convert ascii to integer	set the speed
		
		memset(temp,'\0',TEMPSIZE);		//Clear the temp buffer
		temp[0] = buffer[5];
		temp[1] = buffer[6];
		temp[2] = buffer[7];
		comm->angle = atoi(temp);		//Convert ascii to integer	set the angle
		
		memset(temp,'\0',TEMPSIZE);		//Clear the temp buffer
		temp[0] = buffer[8];
		temp[1] = buffer[9];
		temp[2] = buffer[10];
		comm->distance = atoi(temp);		//Convert ascii to integer	set the distance
		
	}
	
	return data;
}


void robotControl(struct command *comm, int data)		//Controll the 2 tracks
{
	if(data != 0)		//There is new data for the motors to execute
	{
		if(comm->type == 'm')		//Move the robot
		{
			//void move(uint8_t desired_speed, uint8_t dir, uint16_t distance, uint8_t blocking)
			//Speed[0-255], direction[FWD->forward; BWD->backward], distance[mm], check for collisions
		
			if(comm->direction == 'f')
			{
				setLEDs(0b100100);		//2 forward LED's
				move(comm->speed, FWD, comm->distance, true);
			}
			else if(comm->direction =='b')
			{
				setLEDs(0b001001);		//2 backward LED's
				move(comm->speed, BWD, comm->distance, true);
			}
		
		}
	
		else if(comm->type == 'r')		//Rotate the robot
		{
			//void rotate(uint8_t desired_speed, uint8_t dir, uint16_t angle, uint8_t blocking)
			//Speed[0-255], direction[LEFT; RIGHT], angle[0°-180°], check for collisions
		
			if(comm->direction == 'l')
			{
				setLEDs(0b111000);		//3 left LED's
				rotate(comm->speed, LEFT, comm->angle, true);
			}
			else if(comm->direction =='r')
			{
				setLEDs(0b000111);		//3 right LED's
				rotate(comm->speed, RIGHT, comm->angle, true);
			}
		}
	}
}

//Main function
int main(void)
{
	initRobotBase();				//Initialize the base
	
	int countLED = 0;		//Counter for the LED test
	struct command comm;		//Struct for the received data
	int newData = 0;		//Is there new data received to run the motors
	
	while(countLED < LEDFLASH)				//Flash the LED's
	{
		setLEDs(0b111111);
		mSleep(100);
		setLEDs(0b000000);
		
		countLED++;
	}
	
	powerON();		//Turn encoders, motor sensors, ... on
	
	while(1)		//Keep checking for data and use the motors
	{
		newData = recCommand(&comm);		//Receive Command function
		robotControl(&comm, newData);		//Control the robot using the received data
	}
	
	return 0;
}
