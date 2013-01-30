#include <iostream>
#include <string>
#include <sstream>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "kulgur1/LandmarkMeasurementArray.h"
#include "kulgur1/LowOdometry.h"

using namespace ros;
using namespace std;
using namespace kulgur1;

string LandmarkMeasurementArrayToStr(LandmarkMeasurementArray::ConstPtr& lmMsg);
string LowOdometryToStr(LowOdometry::ConstPtr& odoMsg);

int main(int argc, char** argv)
{
	rosbag::Bag bag;

	bag.open("/home/ronald/bag/latest.bag", rosbag::bagmode::Read);

	vector<string> topics;
	topics.push_back("/gazebo/kulgur1/odometry");
	topics.push_back("/gazebo/kulgur1/visible_landmarks");

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	ros::Time prevTimestamp;

	// To make sure that odometry messages preceed measurement messages when parsing output with MATLAB, 
	// landmark measurements are buffered and output only after odometry with same timestamp has been output.
	// Or output before odometry or landmark msg with higher timestamp.
	string 		bufferedLmMsgStr;

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		ros::Time timestamp;

		if (m.getTopic() == topics[0])
		{
			// ODOMETRY MESSAGE
			LowOdometry::ConstPtr odomMsg = m.instantiate<LowOdometry>();
			timestamp = odomMsg->timestamp;

			string odomMsgStr = LowOdometryToStr(odomMsg);

			if (bufferedLmMsgStr.length() > 0)
			{
				if (prevTimestamp == timestamp)
				{
					// Odometry msg came after landmark, but they have the same timestamp, output odom first.
					cout<<odomMsgStr;
					cout<<bufferedLmMsgStr;
				}
				else
				{
					cout<<bufferedLmMsgStr;
					cout<<odomMsgStr;
				}

				bufferedLmMsgStr = "";
			}
			else
			{
				cout<<odomMsgStr;
			}
		} 
		else if(m.getTopic() == topics[1])
		{
			// LANDMARKS MESSAGE
			LandmarkMeasurementArray::ConstPtr lmMsg = m.instantiate<LandmarkMeasurementArray>();
			timestamp = lmMsg->timestamp;

			if (bufferedLmMsgStr.length() > 0)
			{
				// If there was a measurement in the buffer, it should be output.
				cout<<bufferedLmMsgStr;
			}

			bufferedLmMsgStr = LandmarkMeasurementArrayToStr(lmMsg);
		}

		assert(timestamp >= prevTimestamp);

		prevTimestamp = timestamp;
	}

	bag.close();

	return 0;
}

string LandmarkMeasurementArrayToStr(LandmarkMeasurementArray::ConstPtr& lmMsg)
{
	ostringstream lmStr;

	lmStr<<"L"<<" "<<lmMsg->timestamp.sec + 1e-9 * lmMsg->timestamp.nsec;

	for(vector<LandmarkMeasurement>::const_iterator it = lmMsg->measurements.begin(); it != lmMsg->measurements.end() ; ++it)
	{
		lmStr<<" "<<it->bearing<<" "<<it->distance<<" "<<it->angle;
	}

	lmStr<<endl;
	return lmStr.str();
}

string LowOdometryToStr(LowOdometry::ConstPtr& odoMsg)
{
	ostringstream odoStr;

	odoStr<<"O"<<" "<< odoMsg->timestamp.sec + 1e-9 * odoMsg->timestamp.nsec
		<<" "<< odoMsg->front_wheel_rotations[0] <<" "<< odoMsg->front_wheel_rotations[1]
		<<" "<< odoMsg->rear_wheel_angles[0] <<" "<< odoMsg->rear_wheel_angles[1] 
		<<" "<< odoMsg->true_pose.position.x <<" "<< odoMsg->true_pose.position.y <<" "<< odoMsg->true_pose.position.z
		<<" "<< odoMsg->true_pose.orientation.x <<" "<< odoMsg->true_pose.orientation.y 
		<<" "<< odoMsg->true_pose.orientation.z <<" "<< odoMsg->true_pose.orientation.w << endl;

	return odoStr.str();
}
