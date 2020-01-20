/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../common/include/neo_relayboard_v2_node.h"
#include "../common/include/WatchDog.h"

class NodeWatchDog : public WatchDog
{
public:
	NodeWatchDog(ros::Duration timeout_)
		: WatchDog(timeout_)
	{
		topicPub_RelayBoardState = n.advertise<neo_msgs::RelayBoardV2>("state", 1);
	}

protected:
	void handle_timeout() override
	{
		neo_msgs::RelayBoardV2 msg;
		msg.communication_state = neo_msgs::RelayBoardV2::CS_LOST;

		topicPub_RelayBoardState.publish(msg);
	}

private:
	ros::NodeHandle n;
	ros::Publisher topicPub_RelayBoardState;
};

//#######################
//#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_relayboard_v2_node");

	NeoRelayBoardNode node;

	// initialize node
	if (node.init() != 0)
		return 1;

	// get parameters
	double request_rate;   // [1/s]
	double timeout_cycles; // [1]
	node.n.param("request_rate", request_rate, 25.0);
	node.n.param("message_timeout", timeout_cycles, 3.0);

	// frequency of publishing states (cycle time)
	ros::Rate rate(request_rate);

	// setup watchdog with multiples of cycle time
	NodeWatchDog watch_dog(ros::Duration(1 / request_rate * timeout_cycles));
	watch_dog.start();

	while (node.n.ok())
	{
		const ros::Time cycleStartTime = ros::Time::now();

		// Notify watchdog
		watch_dog.tickle();

		// Communication
		node.HandleCommunication();

		// RelayBoard
		node.PublishRelayBoardState();
		node.PublishBatteryState();
		node.PublishEmergencyStopStates();

		// Motors
		node.PublishJointStates();

		// IOBoard
		node.PublishIOBoard();

		// USBoard
		node.PublishUSBoardData();

		ros::spinOnce();

		const ros::Time cycleEndTime = ros::Time::now();

		const ros::Duration cycleTime = cycleEndTime - cycleStartTime;

		//ROS_INFO("cycleTime: %f", cycleTime.toSec());

		rate.sleep();
	}

	watch_dog.stop();

	return 0;
}
