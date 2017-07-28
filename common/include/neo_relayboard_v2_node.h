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

// basics
#include<sstream>

// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <RelayBoard_v2.h>

// ROS message includes
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <neo_msgs/EmergencyStopState.h>
#include <neo_msgs/USBoard.h>
#include <neo_msgs/RelayBoardV2.h>
#include <neo_msgs/IOBoard.h>

// ROS service includes
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <neo_srvs/IOBoardSetDigOut.h>
#include <neo_srvs/RelayBoardSetRelay.h>
#include <neo_srvs/RelayBoardSetLCDMsg.h>

// external includes
//--

//####################
//#### node class ####
class neo_relayboardV2_node
{
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;
                
		// topics:
		//basic topics:
        ros::Publisher topicPub_ComState;
        ros::Publisher topicPub_RelayBoardState;
        ros::Publisher topicPub_BatteryState;
		ros::Publisher topicPub_isEmergencyStop;
		ros::Publisher topicPub_batVoltage;
		ros::Publisher topicPub_chargeCurrent;
		ros::Publisher topicPub_chargeState;
		ros::Publisher topicPub_temperatur;
		ros::Publisher topicPub_keypad;
		ros::Subscriber topicSub_startCharging;
		ros::Subscriber topicSub_stopCharging;
		ros::Publisher topicPub_SendRelayStates;
		ros::Subscriber topicSub_SetRelayStates;

        ros::ServiceServer srv_StartCharging;
        ros::ServiceServer srv_StopCharging;

        ros::ServiceServer srv_SetRelay;
        ros::ServiceServer srv_SetLCDMsg;

        //Drives:
		ros::Publisher topicPub_drives;			//
		ros::Subscriber topicSub_drives;		//

        //USBoard:
		ros::Publisher topicPub_usBoard;
		ros::Publisher topicPub_USRangeSensor1;
		ros::Publisher topicPub_USRangeSensor2;
		ros::Publisher topicPub_USRangeSensor3;
		ros::Publisher topicPub_USRangeSensor4;
		ros::Publisher topicPub_USRangeSensor5;
		ros::Publisher topicPub_USRangeSensor6;
		ros::Publisher topicPub_USRangeSensor7;
		ros::Publisher topicPub_USRangeSensor8;
		ros::Publisher topicPub_USRangeSensor9;
		ros::Publisher topicPub_USRangeSensor10;
		ros::Publisher topicPub_USRangeSensor11;
		ros::Publisher topicPub_USRangeSensor12;
		ros::Publisher topicPub_USRangeSensor13;
		ros::Publisher topicPub_USRangeSensor14;
		ros::Publisher topicPub_USRangeSensor15;
		ros::Publisher topicPub_USRangeSensor16;

		ros::Subscriber topicSub_startUSBoard;
		ros::Subscriber topicSub_stopUSBoard;

		
        //IOBoard:
        ros::Publisher topicPub_IOBoard;
        ros::ServiceServer srv_SetDigOut;


		// Constructor
		neo_relayboardV2_node()
		{
			// Make sure member variables have a defined state at the beginning
			m_iEM_stop_state = ST_EM_FREE;

            m_bRelayBoardV2Available = false;

			m_dRelayBoard_timeout = 2.0;
			m_duration_for_EM_free = ros::Duration(1);
			m_iactive_motors = 0;
			m_imotor_count = 0;
			m_ihoming_motors = 0;
			m_iext_hardware = 0;

            m_iComState = COM_NOT_ESTABLISHED;

            //Battery
            m_sBatterySerialNumber = "NeoBattery999";
            m_sBatteryLocation = "Slot999";
            m_iBatteryChemistry = 1;
            m_fBatteryDesignCapacity = 100.0;

            //IOBoard
            m_bIOBoardActive = false;

            //USBoard
            m_bUSBoardActive = false;
            for(int sensor = 0; sensor < 16; sensor++)
                m_bUSBoardSensorActive[sensor] = false;

            //Drives
            for(int drive_cnt = 0; drive_cnt < 8; drive_cnt++)
                m_Drives[drive_cnt].sName = "Joint999";
		}
        
		// Destructor
		~neo_relayboardV2_node() 
		{
			delete m_SerRelayBoard;
		}

		//Comm Handler
		int init();
		void HandleCommunication();

		// Topic Callbacks
		void getNewVelocitiesFomTopic(const trajectory_msgs::JointTrajectory jt);
		
		// Pubisher functions
		//RelayBoard
		void PublishRelayBoardState();
        void PublishBatteryState();
		void PublishEmergencyStopStates();
		//Motors
		void PublishJointStates();
		//USBoard		
		void PublishUSBoardData();
		//IOBoard
        void PublishIOBoard();

        //Service Callbacks
        bool serviceRelayBoardSetRelay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res);
        bool serviceRelayBoardSetLCDMsg(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res);
        bool serviceStartCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool serviceStopCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool serviceIOBoardSetDigOut(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res);
		
        //Other
		double getRequestRate();
	private:

        //----communication--------
        enum
        {
            COM_NOT_ESTABLISHED,
            COM_CONFIG_FAILED,
            COM_OK,
            COM_LOST,
            COM_ERROR,
        };
        u_int16_t m_iComState;
		//----Configuration--------
		int m_iactive_motors;
		int m_imotor_count;
		int m_ihoming_motors;
		int m_iext_hardware;

        bool m_bRelayBoardV2Available; //the RelayBoard has confirmed the configuration and is ready

        //----Battery------------
        std::string m_sBatterySerialNumber;
        std::string m_sBatteryLocation;
        int m_iBatteryChemistry;
        float m_fBatteryDesignCapacity;

        //----IOBoard------------
        bool m_bIOBoardActive;

        //----USBoard------------
        bool m_bUSBoardActive;
        bool m_bUSBoardSensorActive[16];

		std::string m_sComPort;
		RelayBoardV2 * m_SerRelayBoard;
		
        //Motors
		DriveParam m_Drives[8];

		//----EM Stop Handling------
		int m_iEM_stop_state;
		ros::Duration m_duration_for_EM_free;
		ros::Time m_time_of_EM_confirmed;
		// possible states of emergency stop
		enum
		{
			ST_EM_FREE = 0,
			ST_EM_ACTIVE = 1,
			ST_EM_CONFIRMED = 2
		};

		//----Msg Handling------------
		double m_dRelayBoard_timeout;
		double m_dRequestRate;
		ros::Time m_time_last_message_received;
		

		//log
        bool m_bLog;  //enables or disables the log for neo_relayboard
};

