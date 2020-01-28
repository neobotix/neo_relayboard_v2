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

#include "../common/include/NeoRelayBoardNode.h"
#include "../common/include/WatchDog.h"


//#######################
//#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_relayboard_v2_node");

	// keep a node handle outside the loop to prevent auto-shutdown
	ros::NodeHandle nh;

	while (ros::ok())
	{
		NeoRelayBoardNode node;

		// initialize node
		if (node.init() != 0)
			return 1;

		// get parameters
		double request_rate;   // [1/s]
		node.n.param("request_rate", request_rate, 25.0);

		// frequency of publishing states (cycle time)
		ros::Rate rate(request_rate);

		while (nh.ok())
		{
			const ros::Time cycleStartTime = ros::Time::now();

			// Communication
			const int comState = node.HandleCommunication();

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

			const ros::Duration cycleTime = ros::Time::now() - cycleStartTime;

			//ROS_INFO("cycleTime: %f", cycleTime.toSec());

			// Check if to restart node in case of error
			bool restart = false;
			switch(comState)
			{
			case neo_msgs::RelayBoardV2::CS_OK:
			case neo_msgs::RelayBoardV2::CS_CONFIGURATION_FAILED:
				break;
			default:
				if(ros::ok())
				{
					ROS_WARN("Communication error, restarting node ...");
					ros::WallDuration(1).sleep();
				}
				restart = true;
			}

			if(restart) {
				break;
			}

			rate.sleep();
		}
	}

	return 0;
}
