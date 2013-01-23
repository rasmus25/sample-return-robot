#include <ncurses.h>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

const double WheelRadius 		= 0.95;
const double WheelBase 			= 0.2; 	// Distance of wheels from robot center axis
const double WheelAxisDist 		= 0.48; // Distance between wheel axes
const double TurnWheelJointBase = 0.2;	// 

const double MaxTurnRadius 	= 30.0;
const double MinTurnRadius 	= 3.0;

const double TurnRadiusStep 	= 3.0;
const double DriveVelocityStep 	= 1.0;
//
// Positive turn radius turns left, negative turn radius turns right
// Turn radius HUGE_VAL means going straight forward.
// changeDir should be +1.0 or -1.0.
// +1 increases turn radius when turning left; decreases turn radius when turning right
// +1 from 0 starts turning right, turn radius -MaxTurnRadius.
//
double changeTurnRadius(double currentTurnRadius, double changeDir)
{
	assert(changeDir == 1.0 || changeDir == -1.0);

	if(currentTurnRadius == HUGE_VAL)
	{
		if(changeDir > 0)
		{
			currentTurnRadius = -MaxTurnRadius;
		}
		else
		{
			currentTurnRadius = MaxTurnRadius;
		}
	}
	else
	{
		currentTurnRadius += changeDir*TurnRadiusStep;

		if(fabs(currentTurnRadius) <= MinTurnRadius)
		{
			currentTurnRadius = MinTurnRadius * (currentTurnRadius/fabs(currentTurnRadius));
		}
		else if (fabs(currentTurnRadius) > MaxTurnRadius)
		{
			currentTurnRadius = HUGE_VAL;
		}
	}

	return currentTurnRadius;
}

//
//
//
void turnRadiusToWheelAngles(const double turnRadius, double* out_wheelAngles)
{
	out_wheelAngles[0] = atan2(WheelAxisDist, turnRadius - TurnWheelJointBase);
	out_wheelAngles[1] = atan2(WheelAxisDist, turnRadius + TurnWheelJointBase);
}

//
//
//

void velocityToWheelAngVelocity(const double velocity, const double turnRadius, double* out_wheelAngVelocities)
{
	double w[2] = {velocity, velocity};

	if(turnRadius != HUGE_VAL)
	{
		w[0] = velocity * (turnRadius - WheelBase) / turnRadius;
		w[1] = velocity * (turnRadius + WheelBase) / turnRadius;
	}

	out_wheelAngVelocities[0] = w[0] / (2 * M_PI * WheelRadius);
	out_wheelAngVelocities[1] = w[1] / (2 * M_PI * WheelRadius);
}
//
//
//


int main(int argc, char **argv)
{
	initscr();
	timeout(100);
	keypad(stdscr, TRUE);
	
	ros::init(argc, argv, "kulgur1_teleop");
	ros::NodeHandle n;

	ros::Publisher 	wheelVelocityPublisher = n.advertise<std_msgs::Float64MultiArray>("/gazebo/set_wheel_velocities", 50);
	ros::Publisher 	turnWheelAnglesPublisher = n.advertise<std_msgs::Float64MultiArray>("/gazebo/set_rear_wheel_angles", 50);

	double 	turnRadius 	= INFINITY;
	double 	driveVel 	= 0;

	while(ros::ok())
	{
		bool newCommand = true;

		switch(getch())
		{
			case KEY_LEFT:
				turnRadius = changeTurnRadius(turnRadius, -1.0);
				break;
			case KEY_RIGHT:
				turnRadius = changeTurnRadius(turnRadius, 1.0);
				break;
			case KEY_UP:
				driveVel += DriveVelocityStep;
				break;
			case KEY_DOWN:
				driveVel -= DriveVelocityStep;
				break;
			default:
				newCommand = false;
				break;
		}

		if(newCommand)
		{
			// 
			// Calc and send drive wheel velocities
			std_msgs::Float64MultiArray wheelVelocityMsg;
			double wheelAngVelocities[2];

			velocityToWheelAngVelocity(driveVel, turnRadius, wheelAngVelocities);

			wheelVelocityMsg.data.push_back(wheelAngVelocities[0]);
			wheelVelocityMsg.data.push_back(wheelAngVelocities[1]);

			wheelVelocityPublisher.publish(wheelVelocityMsg);

			//
			// Calc and send turn wheel angles
			//
			std_msgs::Float64MultiArray turnWheelAnglesMsg;
			double turnWheelAngles[2];

			turnRadiusToWheelAngles(turnRadius, turnWheelAngles);

			turnWheelAnglesMsg.data.push_back(turnWheelAngles[0]);
			turnWheelAnglesMsg.data.push_back(turnWheelAngles[1]);

			turnWheelAnglesPublisher.publish(turnWheelAnglesMsg);			
		}

		ros::spinOnce();
	}

	endwin();

	return 0;
}