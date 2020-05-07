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

#include <math.h>
#include <ros/ros.h>

#include "../include/NeoRelayBoardNode.h"

NeoRelayBoardNode::NeoRelayBoardNode()
{
	// Drives
	for (int i = 0; i < 8; i++)
	{
		m_Drives[i].sName = "Joint999";
	}
}

NeoRelayBoardNode::~NeoRelayBoardNode()
{
	delete m_SerRelayBoard;
}

int NeoRelayBoardNode::init()
{
	//---------------------------------------- GET PARAMS -----------------------------------------------------------

	std::cout << "                                                                     \n";
	std::cout << "    NN    N  EEEEE   OOOOO   BBBBB    OOOOO   TTTTTTT  I  X   X      \n";
	std::cout << "    N N   N  E      O     O  B    B  O     O     T     I   X X       \n";
	std::cout << "    N  N  N  EEEEE  O     O  BBBBB   O     O     T     I    X        \n";
	std::cout << "    N   N N  E      O     O  B    B  O     O     T     I   X X       \n";
	std::cout << "    N    NN  EEEEE   OOOOO   BBBBB    OOOOO      T     I  X   X      \n";
	std::cout << "                                                                     \n";

	// Relayboard Config Parameter
	if (n.hasParam("port"))
	{
		n.getParam("port", m_sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s", m_sComPort.c_str());
	}
	else
	{
		ROS_ERROR("FAILED to load ComPort parameter from parameter server");
		return 1;
	}

	// Battery
	n.getParam("battery/serial_number", m_sBatterySerialNumber);
	n.getParam("battery/location", m_sBatteryLocation);
	n.getParam("battery/design_capacity", m_fBatteryDesignCapacity);
	n.getParam("battery/chemistry", m_iBatteryChemistry);

	// Logging
	n.getParam("log", m_bLog);

	// IOBOard Parameter
	n.param("ioboard/active", m_bIOBoardActive, false);

	// USBOard Parameter
	n.param("usboard/active", m_bUSBoardActive, false);
	n.param("usboard/sensor1_active", m_bUSBoardSensorActive[0], false);
	n.param("usboard/sensor2_active", m_bUSBoardSensorActive[1], false);
	n.param("usboard/sensor3_active", m_bUSBoardSensorActive[2], false);
	n.param("usboard/sensor4_active", m_bUSBoardSensorActive[3], false);
	n.param("usboard/sensor5_active", m_bUSBoardSensorActive[4], false);
	n.param("usboard/sensor6_active", m_bUSBoardSensorActive[5], false);
	n.param("usboard/sensor7_active", m_bUSBoardSensorActive[6], false);
	n.param("usboard/sensor8_active", m_bUSBoardSensorActive[7], false);
	n.param("usboard/sensor9_active", m_bUSBoardSensorActive[8], false);
	n.param("usboard/sensor10_active", m_bUSBoardSensorActive[9], false);
	n.param("usboard/sensor11_active", m_bUSBoardSensorActive[10], false);
	n.param("usboard/sensor12_active", m_bUSBoardSensorActive[11], false);
	n.param("usboard/sensor13_active", m_bUSBoardSensorActive[12], false);
	n.param("usboard/sensor14_active", m_bUSBoardSensorActive[13], false);
	n.param("usboard/sensor15_active", m_bUSBoardSensorActive[14], false);
	n.param("usboard/sensor16_active", m_bUSBoardSensorActive[15], false);

	// Motor Parameter
	for (int i = 0; i < 8; ++i)
	{
		const std::string parent = "drive" + std::to_string(i + 2) + "/";

		n.param(parent + "motor_active", m_Drives[i].bmotor_active, false);
		n.param(parent + "homing_active", m_Drives[i].bhoming_active, false);
		n.param(parent + "EncIncrPerRevMot", m_Drives[i].iEncIncrPerRevMot, 0);
		n.param(parent + "VelMeasFrqHz", m_Drives[i].dVelMeasFrqHz, 0.0);
		n.param(parent + "GearRatio", m_Drives[i].dGearRatio, 0.0);
		n.param(parent + "BeltRatio", m_Drives[i].dBeltRatio, 0.0);
		n.param(parent + "Sign", m_Drives[i].iSign, 0);
		n.param(parent + "VelMaxEncIncrS", m_Drives[i].dVelMaxEncIncrS, 0.0);
		n.param(parent + "VelPModeEncIncrS", m_Drives[i].dVelPModeEncIncrS, 0.0);
		n.param(parent + "AccIncrS2", m_Drives[i].dAccIncrS2, 0.0);
		n.param(parent + "DecIncrS2", m_Drives[i].dDecIncrS2, 0.0);
		n.param(parent + "Modulo", m_Drives[i].dModulo, 0.0);
		n.getParam(parent + "joint_name", m_Drives[i].sName);

		m_Drives[i].calcRadToIncr();
	}

	n.param("motor_delay", m_tMotorDelay, 0.0);
	n.param("trajectory_timeout", m_trajectory_timeout, 1.);

	// Check which motors are active
	if (m_Drives[0].bmotor_active)
	{
		m_iactive_motors += 1;
		m_imotor_count++;
	}
	if (m_Drives[1].bmotor_active)
	{
		m_iactive_motors += 2;
		m_imotor_count++;
	}
	if (m_Drives[2].bmotor_active)
	{
		m_iactive_motors += 4;
		m_imotor_count++;
	}
	if (m_Drives[3].bmotor_active)
	{
		m_iactive_motors += 8;
		m_imotor_count++;
	}
	if (m_Drives[4].bmotor_active)
	{
		m_iactive_motors += 16;
		m_imotor_count++;
	}
	if (m_Drives[5].bmotor_active)
	{
		m_iactive_motors += 32;
		m_imotor_count++;
	}
	if (m_Drives[6].bmotor_active)
	{
		m_iactive_motors += 64;
		m_imotor_count++;
	}
	if (m_Drives[7].bmotor_active)
	{
		m_iactive_motors += 128;
		m_imotor_count++;
	}

	// Check if homing is active
	if (m_Drives[0].bhoming_active)
		m_ihoming_motors += 1;
	if (m_Drives[1].bhoming_active)
		m_ihoming_motors += 2;
	if (m_Drives[2].bhoming_active)
		m_ihoming_motors += 4;
	if (m_Drives[3].bhoming_active)
		m_ihoming_motors += 8;
	if (m_Drives[4].bhoming_active)
		m_ihoming_motors += 16;
	if (m_Drives[5].bhoming_active)
		m_ihoming_motors += 32;
	if (m_Drives[6].bhoming_active)
		m_ihoming_motors += 64;
	if (m_Drives[7].bhoming_active)
		m_ihoming_motors += 128;

	// Check external hardware
	if (m_bIOBoardActive)
		m_iext_hardware += 1;
	if (m_bUSBoardActive)
		m_iext_hardware += 2;

	ROS_INFO("Parameters loaded");

	//----------------------------------------OPEN COMPORT---------------------------------------------------------

	m_SerRelayBoard = new RelayBoardClient();

	// Logging yes/no
	if (m_bLog)
	{
		ROS_INFO("Log: enabled");
		m_SerRelayBoard->enable_logging();
	}
	else
	{
		ROS_INFO("Log: disabled");
		m_SerRelayBoard->disable_logging();
	}

	// Initialize SerialBoard
	const int ret = m_SerRelayBoard->init(
		m_sComPort.c_str(), m_iactive_motors, m_ihoming_motors, m_iext_hardware,
		m_Drives[0].dModulo, m_Drives[1].dModulo, m_Drives[2].dModulo, m_Drives[3].dModulo,
		m_Drives[4].dModulo, m_Drives[5].dModulo, m_Drives[6].dModulo, m_Drives[7].dModulo);

	if (ret == RelayBoardClient::INIT_CONFIG_OK)
	{
		m_bRelayBoardV2Available = true;
		m_iComState = neo_msgs::RelayBoardV2::CS_OK;
	}
	else
	{
		ROS_ERROR("FAILED to open RelayboardV2 at ComPort %s", m_sComPort.c_str());
		m_bRelayBoardV2Available = false;

		switch(ret) {
			case RelayBoardClient::INIT_CONFIG_CHANGED:
			case RelayBoardClient::INIT_CONFIG_FAILED:
				m_iComState = neo_msgs::RelayBoardV2::CS_CONFIGURATION_FAILED;
				break;
			default:
				m_iComState = neo_msgs::RelayBoardV2::CS_ERROR;
		}

		if (ret == RelayBoardClient::INIT_OPEN_PORT_FAILED)
		{
			ROS_ERROR("INIT_OPEN_PORT_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_WRITE_FAILED)
		{
			ROS_ERROR("INIT_WRITE_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_CONFIG_CHANGED)
		{
			ROS_ERROR("INIT_CONFIG_CHANGED");
		}
		else if (ret == RelayBoardClient::INIT_CONFIG_FAILED)
		{
			ROS_ERROR("INIT_CONFIG_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_UNKNOWN_ERROR)
		{
			ROS_ERROR("INIT_UNKNOWN_ERROR");
		}
	}

	// Show Config
	for (int iMotorNr = 0; iMotorNr < 8; iMotorNr++)
	{
		m_Drives[iMotorNr].bmotor_avaliable = m_SerRelayBoard->getMotorAvailable(iMotorNr);
		m_Drives[iMotorNr].bmotor_homed = m_SerRelayBoard->getMotorHomed(iMotorNr);
		ROS_INFO("Drive %d: Active %s     Available %s", iMotorNr + 2, m_Drives[iMotorNr].bmotor_active ? "true " : "false", m_Drives[iMotorNr].bmotor_avaliable ? "true" : "false");
		ROS_INFO("         Homing %s     Homed %s", m_Drives[iMotorNr].bhoming_active ? "true " : "false", m_Drives[iMotorNr].bmotor_homed ? "true" : "false");
	}
	ROS_INFO("IOBoard: Active %s     Available %s", m_bIOBoardActive ? "true" : "false", m_SerRelayBoard->getIOBoardAvailable() ? "true" : "false");
	ROS_INFO("USBoard: Active %s     Available %s", m_bUSBoardActive ? "true" : "false", m_SerRelayBoard->getUSBoardAvailable() ? "true" : "false");

	//----------------------------------------Init Publisher/Subscriber--------------------------------------------

	// topics and subscriber which will allways get published
	topicPub_isEmergencyStop = n.advertise<neo_msgs::EmergencyStopState>("emergency_stop_state", 1);
	topicPub_RelayBoardState = n.advertise<neo_msgs::RelayBoardV2>("state", 1);
	topicPub_BatteryState = n.advertise<sensor_msgs::BatteryState>("battery_state", 1);

	srv_SetRelay = n.advertiseService("set_relay", &NeoRelayBoardNode::serviceRelayBoardSetRelay, this);
	srv_StartCharging = n.advertiseService("start_charging", &NeoRelayBoardNode::serviceStartCharging, this);
	srv_StopCharging = n.advertiseService("stop_charging", &NeoRelayBoardNode::serviceStopCharging, this);
	srv_SetLCDMsg = n.advertiseService("set_LCD_msg", &NeoRelayBoardNode::serviceRelayBoardSetLCDMsg, this);

	if (m_iactive_motors != 0)
	{
		topicPub_drives = n.advertise<sensor_msgs::JointState>("/drives/joint_states", 1);
		topicSub_drives = n.subscribe("/drives/joint_trajectory", 1, &NeoRelayBoardNode::getNewVelocitiesFomTopic, this);
	}

	if (m_bIOBoardActive)
	{
		topicPub_IOBoard = n.advertise<neo_msgs::IOBoard>("/ioboard/data", 1);
		srv_SetDigOut = n.advertiseService("/ioboard/set_digital_output", &NeoRelayBoardNode::serviceIOBoardSetDigOut, this);
	}

	if (m_bUSBoardActive)
	{
		topicPub_usBoard = n.advertise<neo_msgs::USBoard>("/usboard/measurements", 1);

		for (int i = 0; i < 16; ++i)
		{
			if(m_bUSBoardSensorActive[i]) {
				topicPub_USRangeSensor[i] = n.advertise<sensor_msgs::Range>("/usboard/sensor" + std::to_string(i + 1), 1);
			}
		}
	}

	return 0;
}

//--------------------------RelayBoardV2-----------------------------------------------------------------------

int NeoRelayBoardNode::HandleCommunication()
{
	// if relayboard is not available return
	if (!m_bRelayBoardV2Available)
		return m_iComState;

	const ros::Time now = ros::Time::now();

	// check for input timeout
	if ((now - m_last_trajectory_time).toSec() > m_trajectory_timeout)
	{
		if (!is_trajectory_timeout && !m_last_trajectory_time.isZero()) {
			ROS_WARN_STREAM("joint_trajectory input timeout! Stopping now.");
		}
		is_trajectory_timeout = true;
	}
	else {
		is_trajectory_timeout = false;
	}

	if (is_trajectory_timeout) {
		for (int i = 0; i < m_imotor_count; i++) {
			m_SerRelayBoard->setMotorDesiredEncS(i, 0);		// set velocities to zero
		}
	}

	// send current data to relayboard
	const int iTXReturn = m_SerRelayBoard->sendDataToRelayBoard();

	// check if sending was ok
	if (iTXReturn == RelayBoardClient::TX_OK)
	{
		// try to receive current data from relayboard
		const int iRXReturn = m_SerRelayBoard->evalRxBuffer();

		// record current time stamp for latest message data
		m_tCurrentTimeStamp = ros::Time::now();

		// check if data was received
		if (iRXReturn == m_iLastRXReturn)
		{
			// Do not show message again
		}
		else if (iRXReturn == RelayBoardClient::RX_UPDATE_MSG) // ok
		{
			ROS_INFO("Communicating with RelayBoard");
		}
		else if (iRXReturn == RelayBoardClient::RX_TIMEOUT) // No Answer => resend
		{
			ROS_ERROR("No answer from RelayBoard (Timeout) ... ");
		}
		else if (iRXReturn == RelayBoardClient::RX_WRONG_CHECKSUM)
		{
			ROS_ERROR("Wrong checksum");
		}
		else if (iRXReturn == RelayBoardClient::RX_NO_HEADER)
		{
			ROS_ERROR("No valid message header found");
		}
		else if (iRXReturn == RelayBoardClient::RX_ERROR_MSG)
		{
			ROS_ERROR("RelayBoard error");
		}
		else
		{
			// Unknown error
			ROS_ERROR_STREAM("Unknown error: " << iRXReturn);
		}

		m_iLastRXReturn = iRXReturn;
	}
	else
	{
		// sending failed
		m_iComState = neo_msgs::RelayBoardV2::CS_ERROR;
		ROS_ERROR("Failed to send data");
	}

	return m_iComState;
}

// -------Publisher------

void NeoRelayBoardNode::PublishRelayBoardState()
{
	if (!m_bRelayBoardV2Available)
	{
		neo_msgs::RelayBoardV2 relayboardv2_msg;
		relayboardv2_msg.header.stamp = ros::Time::now();

		// important part for COM_CONFIG_FAILED
		relayboardv2_msg.communication_state = m_iComState;

		// publish neo_msgs::RelayBoardV2
		topicPub_RelayBoardState.publish(relayboardv2_msg);
	}
	else
	{
		// RelayBoardV2 is available
		neo_msgs::RelayBoardV2 relayboardv2_msg;
		relayboardv2_msg.header.stamp = m_tCurrentTimeStamp;

		int iState = 0;
		m_SerRelayBoard->getRelayBoardState(&iState);

		relayboardv2_msg.relayboardv2_state.assign(false);		  // initialize all states with false
		relayboardv2_msg.relayboardv2_state[0] = (iState == 0);	  // no error
		relayboardv2_msg.relayboardv2_state[1] = (iState & 512);  // charging relay error
		relayboardv2_msg.relayboardv2_state[2] = (iState & 64);   // release brakes button failed
		relayboardv2_msg.relayboardv2_state[3] = (iState & 4);	  // motor error
		relayboardv2_msg.relayboardv2_state[4] = (iState & 8);    // safety relay error
		relayboardv2_msg.relayboardv2_state[5] = (iState & 16);   // Leistungsrelais error
		relayboardv2_msg.relayboardv2_state[6] = (iState & 32);   // EMStop system error

		relayboardv2_msg.shutdown = (iState & 0x400); // relayboard is powering of in < 30s

		relayboardv2_msg.communication_state = m_iComState;

		int iChargingState = 0;
		m_SerRelayBoard->getChargingState(&iChargingState);
		relayboardv2_msg.charging_state = iChargingState;

		int iTemperature = 0;
		m_SerRelayBoard->getTemperature(&iTemperature);
		relayboardv2_msg.temperature = iTemperature;

		int iBatteryVoltage = 0;
		m_SerRelayBoard->getBattVoltage(&iBatteryVoltage);
		relayboardv2_msg.battery_voltage = iBatteryVoltage / 1000.f; // [mV] => [v]

		int iChargingCurrent = 0;
		m_SerRelayBoard->getChargingCurrent(&iChargingCurrent);
		relayboardv2_msg.charging_current = iChargingCurrent / 10.f; // [A]

		relayboardv2_msg.relay_states[0] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_1);
		relayboardv2_msg.relay_states[1] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_2);
		relayboardv2_msg.relay_states[2] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_3);
		relayboardv2_msg.relay_states[3] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_4);

		relayboardv2_msg.keypad[0] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_INFO);
		relayboardv2_msg.keypad[1] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_HOME);
		relayboardv2_msg.keypad[2] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_START);
		relayboardv2_msg.keypad[3] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_STOP);
		relayboardv2_msg.keypad[4] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_RELEASE_BRAKE);
		relayboardv2_msg.keypad[5] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_1);
		relayboardv2_msg.keypad[6] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_2);
		relayboardv2_msg.keypad[7] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_3);

		// publish neo_msgs::RelayBoardV2
		topicPub_RelayBoardState.publish(relayboardv2_msg);

		// handle RelayBoardV2 shutdown
		// RelayBoardV2 will power off in < 30 sec
		if (iState & 0x400)
		{
			ROS_INFO("-----------SHUTDOWN Signal from RelayBoardV2----------");
			ros::shutdown();
			usleep(2000);
			system("sudo halt -p");
		}
	}
}

void NeoRelayBoardNode::PublishBatteryState()
{
	if (!m_bRelayBoardV2Available)
		return;

	sensor_msgs::BatteryState bstate_msg;
	bstate_msg.header.stamp = m_tCurrentTimeStamp;

	// get battery voltage from relayboardv2 msg
	int iBatteryVoltage = 0;
	m_SerRelayBoard->getBattVoltage(&iBatteryVoltage);

	// get charging state from relayboardv2 msg
	int iChargingState = 0;
	m_SerRelayBoard->getChargingState(&iChargingState);

	/* power_supply_status: (uint8   The charging status as reported.)
     * POWER_SUPPLY_STATUS_UNKNOWN
     * POWER_SUPPLY_STATUS_CHARGING
     * POWER_SUPPLY_STATUS_DISCHARGING
     * POWER_SUPPLY_STATUS_NOT_CHARGING
     * POWER_SUPPLY_STATUS_FULL
    */

	// power_supply_status only supports very few different states
	if (iChargingState == m_SerRelayBoard->CHS_CHARGING)
	{
		bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
		bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	// get charging current from relayboardv2 msg
	int iCurrent = 0;
	m_SerRelayBoard->getChargingCurrent(&iCurrent);

	bstate_msg.header.stamp = m_tCurrentTimeStamp;
	bstate_msg.header.frame_id = "";

	bstate_msg.voltage = iBatteryVoltage / 1000.f;							 // float32 Voltage in Volts (Mandatory)
	bstate_msg.current = iCurrent / 10.f;									 // float32 Negative when discharging (A)  (If unmeasured NaN)
	bstate_msg.charge = NAN;												 // float32 Current charge in Ah  (If unmeasured NaN)
	bstate_msg.capacity = NAN;												 // float32 Capacity in Ah (last full capacity)  (If unmeasured NaN)
	bstate_msg.design_capacity = m_fBatteryDesignCapacity;					 // float32 Capacity in Ah (design capacity)  (If unmeasured NaN)
	bstate_msg.percentage = NAN;											 // float32 Charge percentage on 0 to 1 range  (If unmeasured NaN)
	bstate_msg.power_supply_health = bstate_msg.POWER_SUPPLY_HEALTH_UNKNOWN; // uint8   The battery health metric.
	bstate_msg.power_supply_technology = m_iBatteryChemistry;				 // uint8   The battery chemistry.
	bstate_msg.present = true;												 // bool    True if the battery is present
	//bstate.cell_voltage[]                                 // float32[] An array of individual cell voltages for each cell in the pack
	// If individual voltages unknown but number of cells known set each to NaN
	bstate_msg.location = m_sBatteryLocation.c_str();		   // The location into which the battery is inserted. (slot number or plug)
	bstate_msg.serial_number = m_sBatterySerialNumber.c_str(); // The best approximation of the battery serial number

	topicPub_BatteryState.publish(bstate_msg);
}

void NeoRelayBoardNode::PublishEmergencyStopStates()
{
	if (!m_bRelayBoardV2Available)
		return;

	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;

	neo_msgs::EmergencyStopState EM_msg;
	EM_msg.header.stamp = m_tCurrentTimeStamp;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (m_iEM_stop_state)
	{
	case ST_EM_FREE:
	{
		if (EM_signal == true)
		{
			ROS_ERROR("Emergency stop was issued");
			m_iEM_stop_state = EM_msg.EMSTOP;
		}
		break;
	}
	case ST_EM_ACTIVE:
	{
		if (EM_signal == false)
		{
			ROS_INFO("Emergency stop was confirmed");
			m_iEM_stop_state = EM_msg.EMCONFIRMED;
			m_time_of_EM_confirmed = ros::Time::now();
		}
		break;
	}
	case ST_EM_CONFIRMED:
	{
		if (EM_signal == true)
		{
			ROS_ERROR("Emergency stop was issued");
			m_iEM_stop_state = EM_msg.EMSTOP;
		}
		else
		{
			duration_since_EM_confirmed = ros::Time::now() - m_time_of_EM_confirmed;
			if (duration_since_EM_confirmed.toSec() > m_duration_for_EM_free.toSec())
			{
				ROS_INFO("Emergency stop released");
				m_iEM_stop_state = EM_msg.EMFREE;
			}
		}
		break;
	}
	};

	EM_msg.emergency_state = m_iEM_stop_state;

	topicPub_isEmergencyStop.publish(EM_msg);
}

//#############################################
//       RelayBoardV2 Service Callbacks
//#############################################

bool NeoRelayBoardNode::serviceRelayBoardSetRelay(neo_srvs::RelayBoardSetRelay::Request &req,
												  neo_srvs::RelayBoardSetRelay::Response &res)
{
	if (!m_bRelayBoardV2Available)
	{
		res.success = false;
		return false;
	}
	else
	{
		// check if Relay ID is valid
		if (req.id >= 0 && req.id < 4)
		{
			m_SerRelayBoard->setRelayBoardDigOut(req.id, req.state);
			res.success = true;
			return true;
		}
	}
	res.success = false;
	return false;
}

bool NeoRelayBoardNode::serviceStartCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->startCharging();
		return true;
	}
	return false;
}

bool NeoRelayBoardNode::serviceStopCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->stopCharging();
		return true;
	}
	return false;
}

bool NeoRelayBoardNode::serviceRelayBoardSetLCDMsg(neo_srvs::RelayBoardSetLCDMsg::Request &req,
												   neo_srvs::RelayBoardSetLCDMsg::Response &res)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->writeLCD(req.message.c_str());
		res.success = true;
		return true;
	}
	res.success = false;
	return false;
}

//#############################################
//       RelayBoardV2 Motor Control
//#############################################

void NeoRelayBoardNode::PublishJointStates()
{
	if (!m_bRelayBoardV2Available)
		return;

	long lEnc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	long lEncS[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int iStatus[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static float sfLastPos[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sensor_msgs::JointState state;
	state.header.stamp = m_tCurrentTimeStamp - ros::Duration(m_tMotorDelay);

	// Publish Data for all possible Motors
	state.name.resize(8);
	state.position.resize(8);
	state.velocity.resize(8);

	// TODO Joint Names einfügen
	// for(int i = 0; i<anz_drives; i++)  state.name[i] = joint_names[i];

	// Motor Data from MSG Handler for each Motor
	// Enc (4 Byte), EncS (4 Byte) and Status (2 Byte) for each Motor
	for (int i = 0; i < 8; i++)
	{
		state.name[i] = m_Drives[i].sName.c_str();
		m_SerRelayBoard->getMotorEnc(i, &lEnc[i]);
		m_SerRelayBoard->getMotorEncS(i, &lEncS[i]);
		m_SerRelayBoard->getMotorState(i, &iStatus[i]);
		state.position[i] = lEnc[i] - sfLastPos[i];
		sfLastPos[i] = lEnc[i];
		//ROS_INFO("Motor %d: Enc: %f",i,(float)lEnc[i]);
		state.velocity[i] = m_Drives[i].iSign * m_Drives[i].convIncrPerPeriodToRadS(lEncS[i]);
	}

	topicPub_drives.publish(state);
}

void NeoRelayBoardNode::getNewVelocitiesFomTopic(const trajectory_msgs::JointTrajectory jt)
{
	if (!m_bRelayBoardV2Available)
		return;

	trajectory_msgs::JointTrajectoryPoint point = jt.points[0];

	for (int i = 0; i < m_imotor_count; i++)
	{
		// convert velocities [rad/s] -> [incr/period]
		//ROS_INFO("Motor: %d ; Vel: %d [rad]; Vel: %d [incr/period]",i,point.velocities[i],dvelocity);
		double dvelocity = m_Drives[i].iSign * m_Drives[i].convRadSToIncrPerPeriod(point.velocities[i]);

		// check if velocity is too high -> limit velocity
		/*if(MathSup::limit((int)&dvelocity, (int)Drives[i].getVelMax()) != 0)
        {
            ROS_ERROR("Velocity for motor %d limited",i+2);
        }*/

		// send Data to MSG Handler
		m_SerRelayBoard->setMotorDesiredEncS(i, dvelocity);
	}

	m_last_trajectory_time = ros::Time::now();
}

//-----------------------------USBoard-------------------------------------------------------------------------

void NeoRelayBoardNode::PublishUSBoardData()
{
	if (!m_bRelayBoardV2Available || !m_bUSBoardActive)
		return;

	int usSensors1[8];
	int usSensors2[8];
	int usAnalog[4];

	m_SerRelayBoard->getUSBoardData1To8(usSensors1);
	m_SerRelayBoard->getUSBoardData9To16(usSensors2);
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);

	neo_msgs::USBoard usBoard;
	usBoard.header.stamp = m_tCurrentTimeStamp;

	for (int i = 0; i < 8; i++)
	{
		usBoard.sensor[i] = usSensors1[i];
	}
	for (int i = 0; i < 8; i++)
	{
		usBoard.sensor[i + 8] = usSensors2[i];
	}
	for (int i = 0; i < 4; i++)
	{
		usBoard.analog[i] = usAnalog[i];
	}

	// Publish raw data in neo_msgs::USBoard format
	topicPub_usBoard.publish(usBoard);

	// Additionally publish data in ROS sensor_msgs::Range format
	for (int i = 0; i < 16; ++i)
	{
		if(!m_bUSBoardSensorActive[i]) {
			continue;
		}
		std_msgs::Header header;
		header.stamp = m_tCurrentTimeStamp;						   // time
		header.frame_id = "us_" + std::to_string(i + 1) + "_link"; 	// string

		sensor_msgs::Range us_range_msg;
		us_range_msg.header = header;
		us_range_msg.radiation_type = 0;				// uint8   => Enum ULTRASOUND=0; INFRARED=1
		us_range_msg.field_of_view = 1.05;				// float32 [rad]
		us_range_msg.min_range = 0.1;					// float32 [m]
		us_range_msg.max_range = 1.2;					// float32 [m]
		us_range_msg.range = usBoard.sensor[i] / 100.f; // float32 [cm] => [m]

		topicPub_USRangeSensor[i].publish(us_range_msg);
	}
}

/*void NeoRelayBoardNode::startUSBoard(const std_msgs::Int16& configuration)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->startUSBoard(configuration.data);
}

void NeoRelayBoardNode::stopUSBoard(const std_msgs::Empty& empty)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->stopUSBoard();
}*/

//---------------------------------IOBoard---------------------------------------------------------------------

void NeoRelayBoardNode::PublishIOBoard()
{
	if (!m_bRelayBoardV2Available || !m_bIOBoardActive)
		return;

	neo_msgs::IOBoard msg_IOBoard;
	msg_IOBoard.header.stamp = m_tCurrentTimeStamp;

	//bool[16] digital_inputs			# state for all digital inputs
	//bool[16] digital_outputs          # state for all digital outputs
	//uint8[4] analog_inputs			# analog input values

	int iDigInData = 0;
	m_SerRelayBoard->getIOBoardDigIn(&iDigInData);

	int iDigOutData = 0;
	m_SerRelayBoard->getIOBoardDigOut(&iDigOutData);

	for (int iIOCnt = 0; iIOCnt < 16; iIOCnt++)
	{
		const int iMask = 1 << iIOCnt;
		msg_IOBoard.digital_inputs[iIOCnt] = bool(iDigInData & iMask);
		msg_IOBoard.digital_outputs[iIOCnt] = bool(iDigOutData & iMask);
	}

	int analog_in[8];
	m_SerRelayBoard->getIOBoardAnalogIn(analog_in);
	for (int i = 0; i < 8; i++)
	{
		msg_IOBoard.analog_inputs[i] = analog_in[i];
	}

	topicPub_IOBoard.publish(msg_IOBoard);
}

bool NeoRelayBoardNode::serviceIOBoardSetDigOut(neo_srvs::IOBoardSetDigOut::Request &req,
												neo_srvs::IOBoardSetDigOut::Response &res)
{
	if (!m_bRelayBoardV2Available || !m_bIOBoardActive)
	{
		res.success = false;
		return false;
	}
	else
	{
		// check if DigOut ID is valid
		if (req.id >= 0 && req.id < 16)
		{
			m_SerRelayBoard->setIOBoardDigOut(req.id, req.state);
			res.success = true;
			return true;
		}
	}
	res.success = false;
	return false;
}

//-----------------------------END IOBoard---------------------------------------------------------------------
