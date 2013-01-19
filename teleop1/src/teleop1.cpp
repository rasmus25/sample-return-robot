#include <ncurses.h>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
	initscr();
	timeout(100);
	keypad(stdscr, TRUE);
	
	ros::init(argc, argv, "teleop1");
	ros::NodeHandle n;

	ros::Publisher 	publisher = n.advertise<std_msgs::Float64MultiArray>("/gazebo/set_wheel_velocities", 50);

	ros::Rate 		loopRate(5);

	double 			drive;
	double 			turn;
	double 			stopped = true;

	const double 	turnVel 	= 2;
	const double 	driveVel 	= 5;

	while(ros::ok())
	{
		double wheelVel[2];

		switch(getch())
		{
			case KEY_LEFT:
				wheelVel[0] = -turnVel;
				wheelVel[1] = turnVel;
				break;
			case KEY_RIGHT:
				wheelVel[0] = turnVel;
				wheelVel[1] = -turnVel;
				break;
			case KEY_UP:
				wheelVel[0] = -driveVel;
				wheelVel[1] = -driveVel;
				break;
			case KEY_DOWN:
				wheelVel[0] = driveVel;
				wheelVel[1] = driveVel;
				break;
			default:
				wheelVel[0] = 0;
				wheelVel[1] = 0;
				break;
		}

		if(wheelVel[0] != 0 || !stopped)
		{
			std_msgs::Float64MultiArray msg;

			msg.data.push_back(wheelVel[0]); msg.data.push_back(wheelVel[1]);

			publisher.publish(msg);
			ros::spinOnce();

			if(drive != 0)
			{
				stopped = false;
			}
			else
			{
				stopped = true;
			}
		}
	}

	endwin();

	return 0;
}