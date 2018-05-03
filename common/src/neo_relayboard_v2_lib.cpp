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

#include <ros/ros.h>
#include <../include/neo_relayboard_v2_node.h>
#include <math.h>



int neo_relayboardV2_node::init() 
{

//----------------------------------------GET PARAMS-----------------------------------------------------------

    std::cout << "                                                                     \n";
    std::cout << "    NN    N  EEEEE   OOOOO   BBBBB    OOOOO   TTTTTTT  I  X   X      \n";
    std::cout << "    N N   N  E      O     O  B    B  O     O     T     I   X X       \n";
    std::cout << "    N  N  N  EEEEE  O     O  BBBBB   O     O     T     I    X        \n";
    std::cout << "    N   N N  E      O     O  B    B  O     O     T     I   X X       \n";
    std::cout << "    N    NN  EEEEE   OOOOO   BBBBB    OOOOO      T     I  X   X      \n";
    std::cout << "                                                                     \n";

	//Relayboard Config Parameter
    if (n.hasParam("port"))
	{
        n.getParam("port", m_sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",m_sComPort.c_str());
	}
    else
    {
        ROS_ERROR("FAILED to load ComPort parameter from parameter server");
        return 1;
    }

    //Node Parameter
    n.param("message_timeout", m_dRelayBoard_timeout, 2.0);
    n.param("request_rate", m_dRequestRate, 25.0);

    //Battery
    n.getParam("battery/serial_number", m_sBatterySerialNumber);
    n.getParam("battery/locaion", m_sBatteryLocation);
    n.getParam("battery/design_capacity", m_fBatteryDesignCapacity);
    n.getParam("battery/chemistry", m_iBatteryChemistry);

    //Logging
    n.getParam("log", m_bLog);

    //IOBOard Parameter
    n.param("ioboard/active", m_bIOBoardActive, false);

    //USBOard Parameter
    n.param("usboard/active", m_bUSBoardActive);
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

	//Motor Parameter
	//Drive2
    n.param("drive2/motor_active", m_Drives[0].bmotor_active, false);
    n.param("drive2/homing_active", m_Drives[0].bhoming_active, false);
	n.param("drive2/EncIncrPerRevMot", m_Drives[0].iEncIncrPerRevMot, 0);
	n.param("drive2/VelMeasFrqHz", m_Drives[0].dVelMeasFrqHz, 0.0);
	n.param("drive2/GearRatio", m_Drives[0].dGearRatio, 0.0);
	n.param("drive2/BeltRatio", m_Drives[0].dBeltRatio, 0.0);
	n.param("drive2/Sign", m_Drives[0].iSign, 0);
	n.param("drive2/VelMaxEncIncrS", m_Drives[0].dVelMaxEncIncrS, 0.0);
	n.param("drive2/VelPModeEncIncrS", m_Drives[0].dVelPModeEncIncrS, 0.0);
	n.param("drive2/AccIncrS2", m_Drives[0].dAccIncrS2, 0.0);
	n.param("drive2/DecIncrS2", m_Drives[0].dDecIncrS2, 0.0);
	n.param("drive2/Modulo", m_Drives[0].dModulo, 0.0);
    n.getParam("drive2/joint_name", m_Drives[0].sName);
	m_Drives[0].calcRadToIncr();
	//Drive3
    n.param("drive3/motor_active", m_Drives[1].bmotor_active, false);
    n.param("drive3/homing_active", m_Drives[1].bhoming_active, false);
	n.param("drive3/EncIncrPerRevMot", m_Drives[1].iEncIncrPerRevMot, 0);
	n.param("drive3/VelMeasFrqHz", m_Drives[1].dVelMeasFrqHz, 0.0);
	n.param("drive3/GearRatio", m_Drives[1].dGearRatio, 0.0);
	n.param("drive3/BeltRatio", m_Drives[1].dBeltRatio, 0.0);
	n.param("drive3/Sign", m_Drives[1].iSign, 0);
	n.param("drive3/VelMaxEncIncrS", m_Drives[1].dVelMaxEncIncrS, 0.0);
	n.param("drive3/VelPModeEncIncrS", m_Drives[1].dVelPModeEncIncrS, 0.0);
	n.param("drive3/AccIncrS2", m_Drives[1].dAccIncrS2, 0.0);
	n.param("drive3/DecIncrS2", m_Drives[1].dDecIncrS2, 0.0);
	n.param("drive3/Modulo", m_Drives[1].dModulo, 0.0);
    n.getParam("drive3/joint_name", m_Drives[1].sName);
	m_Drives[1].calcRadToIncr();
	//Drive4
    n.param("drive4/motor_active", m_Drives[2].bmotor_active, false);
    n.param("drive4/homing_active", m_Drives[2].bhoming_active, false);
	n.param("drive4/EncIncrPerRevMot", m_Drives[2].iEncIncrPerRevMot, 0);
	n.param("drive4/VelMeasFrqHz", m_Drives[2].dVelMeasFrqHz, 0.0);
	n.param("drive4/GearRatio", m_Drives[2].dGearRatio, 0.0);
	n.param("drive4/BeltRatio", m_Drives[2].dBeltRatio, 0.0);
	n.param("drive4/Sign", m_Drives[2].iSign, 0);
	n.param("drive4/VelMaxEncIncrS", m_Drives[2].dVelMaxEncIncrS, 0.0);
	n.param("drive4/VelPModeEncIncrS", m_Drives[2].dVelPModeEncIncrS, 0.0);
	n.param("drive4/AccIncrS2", m_Drives[2].dAccIncrS2, 0.0);
	n.param("drive4/DecIncrS2", m_Drives[2].dDecIncrS2, 0.0);
	n.param("drive4/Modulo", m_Drives[2].dModulo, 0.0);
    n.getParam("drive4/joint_name", m_Drives[2].sName);
	m_Drives[2].calcRadToIncr();
	//Drive5
    n.param("drive5/motor_active", m_Drives[3].bmotor_active, false);
    n.param("drive5/homing_active", m_Drives[3].bhoming_active, false);
	n.param("drive5/EncIncrPerRevMot", m_Drives[3].iEncIncrPerRevMot, 0);
	n.param("drive5/VelMeasFrqHz", m_Drives[3].dVelMeasFrqHz, 0.0);
	n.param("drive5/GearRatio", m_Drives[3].dGearRatio, 0.0);
	n.param("drive5/BeltRatio", m_Drives[3].dBeltRatio, 0.0);
	n.param("drive5/Sign", m_Drives[3].iSign, 0);
	n.param("drive5/VelMaxEncIncrS", m_Drives[3].dVelMaxEncIncrS, 0.0);
	n.param("drive5/VelPModeEncIncrS", m_Drives[3].dVelPModeEncIncrS, 0.0);
	n.param("drive5/AccIncrS2", m_Drives[3].dAccIncrS2, 0.0);
	n.param("drive5/DecIncrS2", m_Drives[3].dDecIncrS2, 0.0);
	n.param("drive5/Modulo", m_Drives[3].dModulo, 0.0);
    n.getParam("drive5/joint_name", m_Drives[3].sName);
	m_Drives[3].calcRadToIncr();
	//Drive6
    n.param("drive6/motor_active", m_Drives[4].bmotor_active, false);
    n.param("drive6/homing_active", m_Drives[4].bhoming_active, false);
	n.param("drive6/EncIncrPerRevMot", m_Drives[4].iEncIncrPerRevMot, 0);
	n.param("drive6/VelMeasFrqHz", m_Drives[4].dVelMeasFrqHz, 0.0);
	n.param("drive6/GearRatio", m_Drives[4].dGearRatio, 0.0);
	n.param("drive6/BeltRatio", m_Drives[4].dBeltRatio, 0.0);
    n.param("drive6/Sign", m_Drives[4].iSign, 0);
	n.param("drive6/VelMaxEncIncrS", m_Drives[4].dVelMaxEncIncrS, 0.0);
	n.param("drive6/VelPModeEncIncrS", m_Drives[4].dVelPModeEncIncrS, 0.0);
	n.param("drive6/AccIncrS2", m_Drives[4].dAccIncrS2, 0.0);
	n.param("drive6/DecIncrS2", m_Drives[4].dDecIncrS2, 0.0);
	n.param("drive6/Modulo", m_Drives[4].dModulo, 0.0);
    n.getParam("drive6/joint_name", m_Drives[4].sName);
	m_Drives[4].calcRadToIncr();
	//Drive7
    n.param("drive7/motor_active", m_Drives[5].bmotor_active, false);
    n.param("drive7/homing_active", m_Drives[5].bhoming_active, false);
	n.param("drive7/EncIncrPerRevMot", m_Drives[5].iEncIncrPerRevMot, 0);
	n.param("drive7/VelMeasFrqHz", m_Drives[5].dVelMeasFrqHz, 0.0);
	n.param("drive7/GearRatio", m_Drives[5].dGearRatio, 0.0);
	n.param("drive7/BeltRatio", m_Drives[5].dBeltRatio, 0.0);
    n.param("drive7/Sign", m_Drives[5].iSign, 0);
	n.param("drive7/VelMaxEncIncrS", m_Drives[5].dVelMaxEncIncrS, 0.0);
	n.param("drive7/VelPModeEncIncrS", m_Drives[5].dVelPModeEncIncrS, 0.0);
	n.param("drive7/AccIncrS2", m_Drives[5].dAccIncrS2, 0.0);
	n.param("drive7/DecIncrS2", m_Drives[5].dDecIncrS2, 0.0);
	n.param("drive7/Modulo", m_Drives[5].dModulo, 0.0);
    n.getParam("drive7/joint_name", m_Drives[5].sName);
	m_Drives[5].calcRadToIncr();
	//Drive8
    n.param("drive8/motor_active", m_Drives[6].bmotor_active, false);
    n.param("drive8/homing_active", m_Drives[6].bhoming_active, false);
	n.param("drive8/EncIncrPerRevMot", m_Drives[6].iEncIncrPerRevMot, 0);
	n.param("drive8/VelMeasFrqHz", m_Drives[6].dVelMeasFrqHz, 0.0);
	n.param("drive8/GearRatio", m_Drives[6].dGearRatio, 0.0);
	n.param("drive8/BeltRatio", m_Drives[6].dBeltRatio, 0.0);
    n.param("drive8/Sign", m_Drives[6].iSign, 0);
	n.param("drive8/VelMaxEncIncrS", m_Drives[6].dVelMaxEncIncrS, 0.0);
	n.param("drive8/VelPModeEncIncrS", m_Drives[6].dVelPModeEncIncrS, 0.0);
	n.param("drive8/AccIncrS2", m_Drives[6].dAccIncrS2, 0.0);
	n.param("drive8/DecIncrS2", m_Drives[6].dDecIncrS2, 0.0);
	n.param("drive8/Modulo", m_Drives[6].dModulo, 0.0);
    n.getParam("drive8/joint_name", m_Drives[6].sName);
	m_Drives[6].calcRadToIncr();
	//Drive9
    n.param("drive9/motor_active", m_Drives[7].bmotor_active, false);
    n.param("drive9/homing_active", m_Drives[7].bhoming_active, false);
	n.param("drive9/EncIncrPerRevMot", m_Drives[7].iEncIncrPerRevMot, 0);
	n.param("drive9/VelMeasFrqHz", m_Drives[7].dVelMeasFrqHz, 0.0);
	n.param("drive9/GearRatio", m_Drives[7].dGearRatio, 0.0);
	n.param("drive9/BeltRatio", m_Drives[7].dBeltRatio, 0.0);
    n.param("drive9/Sign", m_Drives[7].iSign, 0);
	n.param("drive9/VelMaxEncIncrS", m_Drives[7].dVelMaxEncIncrS, 0.0);
	n.param("drive9/VelPModeEncIncrS", m_Drives[7].dVelPModeEncIncrS, 0.0);
	n.param("drive9/AccIncrS2", m_Drives[7].dAccIncrS2, 0.0);
	n.param("drive9/DecIncrS2", m_Drives[7].dDecIncrS2, 0.0);
	n.param("drive9/Modulo", m_Drives[7].dModulo, 0.0);
    n.getParam("drive9/joint_name", m_Drives[7].sName);
	m_Drives[7].calcRadToIncr();
//----------------------------------------END GET PARAMS-------------------------------------------------------
//----------------------------------------OPEN COMPORT---------------------------------------------------------
	//Check which motors are active
    if(m_Drives[0].bmotor_active) {m_iactive_motors += 1;m_imotor_count++;}
    if(m_Drives[1].bmotor_active) {m_iactive_motors += 2;m_imotor_count++;}
    if(m_Drives[2].bmotor_active) {m_iactive_motors += 4;m_imotor_count++;}
    if(m_Drives[3].bmotor_active) {m_iactive_motors += 8;m_imotor_count++;}
    if(m_Drives[4].bmotor_active) {m_iactive_motors += 16;m_imotor_count++;}
    if(m_Drives[5].bmotor_active) {m_iactive_motors += 32;m_imotor_count++;}
    if(m_Drives[6].bmotor_active) {m_iactive_motors += 64;m_imotor_count++;}
    if(m_Drives[7].bmotor_active) {m_iactive_motors += 128;m_imotor_count++;}
	//Check if homing is active
    if(m_Drives[0].bhoming_active) m_ihoming_motors += 1;
    if(m_Drives[1].bhoming_active) m_ihoming_motors += 2;
    if(m_Drives[2].bhoming_active) m_ihoming_motors += 4;
    if(m_Drives[3].bhoming_active) m_ihoming_motors += 8;
    if(m_Drives[4].bhoming_active) m_ihoming_motors += 16;
    if(m_Drives[5].bhoming_active) m_ihoming_motors += 32;
    if(m_Drives[6].bhoming_active) m_ihoming_motors += 64;
    if(m_Drives[7].bhoming_active) m_ihoming_motors += 128;
	//Check external hardware
    if(m_bIOBoardActive) m_iext_hardware += 1;
    if(m_bUSBoardActive) m_iext_hardware += 2;
	ROS_INFO("Parameters loaded");
    ROS_INFO("Configuring RelayBoardV2");
	m_SerRelayBoard = new RelayBoardV2();
	int ret = m_SerRelayBoard->init(m_sComPort.c_str(),m_iactive_motors,m_ihoming_motors,m_iext_hardware,(long)m_Drives[0].dModulo,(long)m_Drives[1].dModulo,(long)m_Drives[2].dModulo,(long)m_Drives[3].dModulo,(long)m_Drives[4].dModulo,(long)m_Drives[5].dModulo,(long)m_Drives[6].dModulo,(long)m_Drives[7].dModulo);
    if(ret == m_SerRelayBoard->INIT_CONFIG_OK)
	{
        m_bRelayBoardV2Available = true;
		ROS_INFO("Opened RelayboardV2 at ComPort = %s", m_sComPort.c_str());
        m_iComState = COM_OK;
	}
	else	
    {
        ROS_ERROR("FAILED to open RelayboardV2 at ComPort = %s", m_sComPort.c_str());
        m_bRelayBoardV2Available = false;
        m_iComState = COM_CONFIG_FAILED;

        if(ret == m_SerRelayBoard->INIT_OPEN_PORT_FAILED)
        {
            ROS_ERROR("INIT_OPEN_PORT_FAILED");
        }
        else if(ret == m_SerRelayBoard->INIT_WRITE_FAILED)
        {
            ROS_ERROR("INIT_WRITE_FAILED");
        }
        else if(ret == m_SerRelayBoard->INIT_CONFIG_CHANGED)
        {
            ROS_ERROR("INIT_CONFIG_CHANGED");
        }
        else if(ret == m_SerRelayBoard->INIT_CONFIG_FAILED)
        {
            ROS_ERROR("INIT_CONFIG_FAILED");
        }
        else if(ret == m_SerRelayBoard->INIT_UNKNOWN_ERROR)
        {
            ROS_ERROR("INIT_UNKNOWN_ERROR");
        }
    }

    //Show Config
    for(int iMotorNr = 0; iMotorNr < 8; iMotorNr++)
    {
        m_Drives[iMotorNr].bmotor_avaliable = m_SerRelayBoard->getMotorAvailable(iMotorNr);
        m_Drives[iMotorNr].bmotor_homed = m_SerRelayBoard->getMotorHomed(iMotorNr);
        ROS_INFO("Drive %d: Active %s     Available %s",iMotorNr+2, m_Drives[iMotorNr].bmotor_active ? "true " : "false", m_Drives[iMotorNr].bmotor_avaliable ? "true" : "false");
        ROS_INFO("         Homing %s     Homed %s", m_Drives[iMotorNr].bhoming_active ? "true " : "false", m_Drives[iMotorNr].bmotor_homed ? "true" : "false");
    }
    ROS_INFO("IOBoard: Active %s     Available %s",m_bIOBoardActive ? "true" : "false", false ? "true" : "false");
    ROS_INFO("USBoard: Active %s     Available %s",m_bUSBoardActive ? "true" : "false", false ? "true" : "false");

//----------------------------------------END OPEN COMPORT-----------------------------------------------------
//----------------------------------------Init Publisher/Subscriber--------------------------------------------
	//topics and subscriber which will allways get published
    topicPub_isEmergencyStop = n.advertise<neo_msgs::EmergencyStopState>("emergency_stop_state", 1);
    topicPub_RelayBoardState = n.advertise<neo_msgs::RelayBoardV2>("state",1);
    topicPub_BatteryState = n.advertise<sensor_msgs::BatteryState>("battery_state",1);

    srv_SetRelay = n.advertiseService("set_relay", &neo_relayboardV2_node::serviceRelayBoardSetRelay, this);
    srv_StartCharging = n.advertiseService("start_charging", &neo_relayboardV2_node::serviceStartCharging, this);
    srv_StopCharging = n.advertiseService("stop_charging", &neo_relayboardV2_node::serviceStopCharging, this);
    srv_SetLCDMsg = n.advertiseService("set_LCD_msg", &neo_relayboardV2_node::serviceRelayBoardSetLCDMsg, this);


	if(m_iactive_motors != 0)
	{
        topicPub_drives = n.advertise<sensor_msgs::JointState>("/drives/joint_states",1);
        topicSub_drives = n.subscribe("/drives/joint_trajectory",1,&neo_relayboardV2_node::getNewVelocitiesFomTopic, this);
	}

    if(m_bIOBoardActive)
	{
        topicPub_IOBoard = n.advertise<neo_msgs::IOBoard>("/ioboard/data",1);
        srv_SetDigOut = n.advertiseService("/ioboard/set_digital_output", &neo_relayboardV2_node::serviceIOBoardSetDigOut, this);

	}
    if(m_bUSBoardActive)
	{
        topicPub_usBoard = n.advertise<neo_msgs::USBoard>("/usboard/measurements",1);

        topicPub_USRangeSensor1 = n.advertise<sensor_msgs::Range>("/usboard/sensor1",1);
        topicPub_USRangeSensor2 = n.advertise<sensor_msgs::Range>("/usboard/sensor2",1);
        topicPub_USRangeSensor3 = n.advertise<sensor_msgs::Range>("/usboard/sensor3",1);
        topicPub_USRangeSensor4 = n.advertise<sensor_msgs::Range>("/usboard/sensor4",1);
        topicPub_USRangeSensor5 = n.advertise<sensor_msgs::Range>("/usboard/sensor5",1);
        topicPub_USRangeSensor6 = n.advertise<sensor_msgs::Range>("/usboard/sensor6",1);
        topicPub_USRangeSensor7 = n.advertise<sensor_msgs::Range>("/usboard/sensor7",1);
        topicPub_USRangeSensor8 = n.advertise<sensor_msgs::Range>("/usboard/sensor8",1);
        topicPub_USRangeSensor9 = n.advertise<sensor_msgs::Range>("/usboard/sensor9",1);
        topicPub_USRangeSensor10 = n.advertise<sensor_msgs::Range>("/usboard/sensor10",1);
        topicPub_USRangeSensor11 = n.advertise<sensor_msgs::Range>("/usboard/sensor11",1);
        topicPub_USRangeSensor12 = n.advertise<sensor_msgs::Range>("/usboard/sensor12",1);
        topicPub_USRangeSensor13 = n.advertise<sensor_msgs::Range>("/usboard/sensor13",1);
        topicPub_USRangeSensor14 = n.advertise<sensor_msgs::Range>("/usboard/sensor14",1);
        topicPub_USRangeSensor15 = n.advertise<sensor_msgs::Range>("/usboard/sensor15",1);
        topicPub_USRangeSensor16 = n.advertise<sensor_msgs::Range>("/usboard/sensor16",1);
	}

	//logging
    if(m_bLog)
	{
        ROS_INFO("Log: enabled");
		m_SerRelayBoard->enable_logging();
	}
	else
	{
        ROS_INFO("Log: disabled");
		m_SerRelayBoard->disable_logging();
	}

//----------------------------------------END Init Publisher/Subscriber----------------------------------------
	return 0;
}

//--------------------------RelayBoardV2-----------------------------------------------------------------------
void neo_relayboardV2_node::HandleCommunication() 
{
    //if relayboard is not available return
    if(!m_bRelayBoardV2Available) return;

    //var to store return from sendDataToRelayBoard()
    int iTXReturn = 0;

    //mark return as RX_TIMEOUT
    int iRXReturn = m_SerRelayBoard->RX_TIMEOUT;

    //var to store the last return from evalRXBuffer()
    static int siLastRXReturn = 0;

    //resend data on timeout
    while(iRXReturn == m_SerRelayBoard->RX_TIMEOUT)
	{ 
        //send current data to relayboard
        iTXReturn = m_SerRelayBoard->sendDataToRelayBoard();

        //check if sending was ok
        if(iTXReturn == m_SerRelayBoard->TX_OK)
        {
            //try to receive current data from relayboard
            iRXReturn = m_SerRelayBoard->evalRxBuffer();

            //check if data was received
            if(iRXReturn == siLastRXReturn)
            {
                //Do not show message again
                if(iRXReturn == m_SerRelayBoard->RX_UPDATE_MSG) //ok
                {
                    m_time_last_message_received = ros::Time::now();
                }
            }
            else if(iRXReturn == m_SerRelayBoard->RX_UPDATE_MSG) //ok
            {
                ROS_INFO("communicating with RelayBoard");
                m_time_last_message_received = ros::Time::now();
            }
            else if(iRXReturn == m_SerRelayBoard->RX_TIMEOUT) //No Answer => resend
            {
                ROS_ERROR("no answer from RelayBoard (Timeout) ... ");
            }
            else if(iRXReturn == m_SerRelayBoard->RX_WRONG_CHECKSUM)
            {
                ROS_ERROR("wrong checksum");
            }
            else if(iRXReturn == m_SerRelayBoard->RX_NO_HEADER)
            {
                ROS_ERROR("no valid message header found");
            }
            else
            {
                //Unknown error
                ROS_ERROR("unknown error");
            }

            siLastRXReturn = iRXReturn;
        }
        else
        {
            //sending failed
            ROS_ERROR("failed to send data");
        }
		
	}

    //check if timeout
    ros::Duration time_since_last_msg = ros::Time::now() - m_time_last_message_received; // [s]
    if(time_since_last_msg.toSec() > m_dRelayBoard_timeout)
    {
        m_iComState = COM_LOST;
    }
    else
    {
        m_iComState = COM_OK;
    }
}

double neo_relayboardV2_node::getRequestRate()
{
	return m_dRequestRate;
}
//-------Publisher------
void neo_relayboardV2_node::PublishRelayBoardState()
{
    if(!m_bRelayBoardV2Available)
    {
        neo_msgs::RelayBoardV2 relayboardv2_msg;
        relayboardv2_msg.relayboardv2_state[0] = false;
        relayboardv2_msg.relayboardv2_state[1] = false;
        relayboardv2_msg.relayboardv2_state[2] = false;
        relayboardv2_msg.relayboardv2_state[3] = false;
        relayboardv2_msg.relayboardv2_state[4] = false;
        relayboardv2_msg.relayboardv2_state[5] = false;
        relayboardv2_msg.relayboardv2_state[6] = false;
        relayboardv2_msg.relayboardv2_state[7] = false;
        relayboardv2_msg.relayboardv2_state[8] = false;
        relayboardv2_msg.relayboardv2_state[9] = false;
        relayboardv2_msg.relayboardv2_state[10] = false;
        relayboardv2_msg.relayboardv2_state[11] = false;
        relayboardv2_msg.relayboardv2_state[12] = false;
        relayboardv2_msg.relayboardv2_state[13] = false;
        relayboardv2_msg.relayboardv2_state[14] = false;
        relayboardv2_msg.relayboardv2_state[15] = false;
        relayboardv2_msg.shutdown = false;

        //important part for COM_CONFIG_FAILED
        relayboardv2_msg.communication_state = m_iComState;

        relayboardv2_msg.charging_state = 0;
        relayboardv2_msg.temperature = 0.0;
        relayboardv2_msg.battery_voltage = 0.0;
        relayboardv2_msg.charging_current = 0.0;
        relayboardv2_msg.relay_states[0] = false;
        relayboardv2_msg.relay_states[1] = false;
        relayboardv2_msg.relay_states[2] = false;
        relayboardv2_msg.relay_states[3] = false;
        relayboardv2_msg.keypad[0] = false;
        relayboardv2_msg.keypad[1] = false;
        relayboardv2_msg.keypad[2] = false;
        relayboardv2_msg.keypad[3] = false;
        relayboardv2_msg.keypad[4] = false;
        relayboardv2_msg.keypad[5] = false;
        relayboardv2_msg.keypad[6] = false;
        relayboardv2_msg.keypad[7] = false;

        //publish neo_msgs::RelayBoardV2
        topicPub_RelayBoardState.publish(relayboardv2_msg);

    }
    else
    {
        //RelayBoardV2 is available
        neo_msgs::RelayBoardV2 relayboardv2_msg;

        int iState = 0;
        m_SerRelayBoard->getRelayBoardState(&iState);

        relayboardv2_msg.relayboardv2_state[0] = (iState & 1);      //no error
        relayboardv2_msg.relayboardv2_state[1] = (iState & 1024);   //charging relay error
        relayboardv2_msg.relayboardv2_state[2] = (iState & 128);    //release brakes button failed
        relayboardv2_msg.relayboardv2_state[3] = (iState & 8);      //motor error
        relayboardv2_msg.relayboardv2_state[4] = (iState & 16);     //safety relay error
        relayboardv2_msg.relayboardv2_state[5] = (iState & 32);     //Leistungsrelais error
        relayboardv2_msg.relayboardv2_state[6] = (iState & 64);     //EMStop system error
        relayboardv2_msg.relayboardv2_state[7] = false;
        relayboardv2_msg.relayboardv2_state[8] = false;
        relayboardv2_msg.relayboardv2_state[9] = false;
        relayboardv2_msg.relayboardv2_state[10] = false;
        relayboardv2_msg.relayboardv2_state[11] = false;
        relayboardv2_msg.relayboardv2_state[12] = false;
        relayboardv2_msg.relayboardv2_state[13] = false;
        relayboardv2_msg.relayboardv2_state[14] = false;
        relayboardv2_msg.relayboardv2_state[15] = false;

        relayboardv2_msg.shutdown = (iState & 0x400); //relayboard is powering of in < 30s

        relayboardv2_msg.communication_state = m_iComState;

        int16_t iChargingState = m_SerRelayBoard->getChargingState();
        relayboardv2_msg.charging_state = iChargingState;

        int16_t iTemperature = 0;
        m_SerRelayBoard->getTemperature(&iTemperature);
        relayboardv2_msg.temperature = iTemperature;

        u_int16_t iBatteryVoltage = 0;
        m_SerRelayBoard->getBattVoltage(&iBatteryVoltage);
        float_t fbattery_voltage = iBatteryVoltage/1000;   //[mV] => [v]
        relayboardv2_msg.battery_voltage = fbattery_voltage;

        int16_t iChargingCurrent = 0;
        m_SerRelayBoard->getChargingCurrent(&iChargingCurrent);
        float_t fcurrent = (float_t)iChargingCurrent/10;
        relayboardv2_msg.charging_current = fcurrent;       //[A]

        relayboardv2_msg.relay_states[0] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_CHARGE);
        relayboardv2_msg.relay_states[1] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_1);
        relayboardv2_msg.relay_states[2] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_2);
        relayboardv2_msg.relay_states[3] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_3);


        //keypad buttons have inverted logic
        relayboardv2_msg.keypad[0] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_INFO);
        relayboardv2_msg.keypad[1] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_HOME);
        relayboardv2_msg.keypad[2] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_START);
        relayboardv2_msg.keypad[3] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_STOP);
        relayboardv2_msg.keypad[4] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_RELEASE_BRAKE);
        relayboardv2_msg.keypad[5] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_1);
        relayboardv2_msg.keypad[6] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_2);
        relayboardv2_msg.keypad[7] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_3);

        //publish neo_msgs::RelayBoardV2
        topicPub_RelayBoardState.publish(relayboardv2_msg);

        //handle RelayBoardV2 shutdown
        //RelayBoardV2 will power off in < 30 sec
        if((iState & 0x400) != 0)
        {
            ROS_INFO("-----------SHUTDOWN Signal from RelayBoardV2----------");
            ros::shutdown();
            usleep(2000);
            system("dbus-send --system --print-reply --dest=org.freedesktop.login1 /org/freedesktop/login1 \"org.freedesktop.login1.Manager.PowerOff\" boolean:true");
        }
    }
}
void neo_relayboardV2_node::PublishBatteryState()
{
    if(!m_bRelayBoardV2Available) return;

    sensor_msgs::BatteryState bstate_msg;

    //get battery voltage from relayboardv2 msg
    u_int16_t ibattery_voltage = 0;
    m_SerRelayBoard->getBattVoltage(&ibattery_voltage);
    float_t fbattery_voltage = ibattery_voltage/1000;

    //get charging state from relayboardv2 msg
    u_int16_t iChargingState = m_SerRelayBoard->getChargingState();

    /*power_supply_status: (uint8   The charging status as reported.)
     * POWER_SUPPLY_STATUS_UNKNOWN
     * POWER_SUPPLY_STATUS_CHARGING
     * POWER_SUPPLY_STATUS_DISCHARGING
     * POWER_SUPPLY_STATUS_NOT_CHARGING
     * POWER_SUPPLY_STATUS_FULL
    */

    //power_supply_status only supports very few different states
    if(iChargingState == m_SerRelayBoard->CHS_CHARGING)
    {
        bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_CHARGING;
    }
    else
    {
        bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_NOT_CHARGING;
    }


    //get charging current from relayboardv2 msg
    int16_t icurrent = 0;
    m_SerRelayBoard->getChargingCurrent(&icurrent);
    float_t fcurrent = (float_t)icurrent/10;

    bstate_msg.header.stamp = ros::Time::now();
    bstate_msg.header.frame_id = "";

    bstate_msg.voltage = fbattery_voltage;              //float32 Voltage in Volts (Mandatory)
    bstate_msg.current = fcurrent;                           //float32 Negative when discharging (A)  (If unmeasured NaN)
    bstate_msg.charge = NAN;                //float32 Current charge in Ah  (If unmeasured NaN)
    bstate_msg.capacity = NAN;              //float32 Capacity in Ah (last full capacity)  (If unmeasured NaN)
    bstate_msg.design_capacity = m_fBatteryDesignCapacity;       //float32 Capacity in Ah (design capacity)  (If unmeasured NaN)
    bstate_msg.percentage = NAN;            //float32 Charge percentage on 0 to 1 range  (If unmeasured NaN)
    bstate_msg.power_supply_health = bstate_msg.POWER_SUPPLY_HEALTH_UNKNOWN;     //uint8   The battery health metric.
    bstate_msg.power_supply_technology = m_iBatteryChemistry; //uint8   The battery chemistry.
    bstate_msg.present = true;              //bool    True if the battery is present
    //bstate.cell_voltage[]             //float32[] An array of individual cell voltages for each cell in the pack
                                        //If individual voltages unknown but number of cells known set each to NaN
    bstate_msg.location = m_sBatteryLocation.c_str();          //string  The location into which the battery is inserted. (slot number or plug)
    bstate_msg.serial_number = m_sBatterySerialNumber.c_str();      //string  The best approximation of the battery serial number

    topicPub_BatteryState.publish(bstate_msg);

}

void neo_relayboardV2_node::PublishEmergencyStopStates()
{
    if(!m_bRelayBoardV2Available) return;
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	neo_msgs::EmergencyStopState EM_msg;

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
				if( duration_since_EM_confirmed.toSec() > m_duration_for_EM_free.toSec() )
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

bool neo_relayboardV2_node::serviceRelayBoardSetRelay(neo_srvs::RelayBoardSetRelay::Request &req, neo_srvs::RelayBoardSetRelay::Response &res)
{
    if(!m_bRelayBoardV2Available)
    {
        res.success = false;
        return false;
    }
    else
    {
        //check if Relay ID is valid
        if(req.id >= 0 && req.id < 4)
        {
            m_SerRelayBoard->setRelayBoardDigOut(req.id, req.state);
            res.success = true;
            return true;
        }
    }
    res.success = false;
    return false;
}

bool neo_relayboardV2_node::serviceStartCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if(m_bRelayBoardV2Available)
    {
        m_SerRelayBoard->startCharging();
        return true;
    }
    return false;
}

bool neo_relayboardV2_node::serviceStopCharging(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if(m_bRelayBoardV2Available)
    {
        m_SerRelayBoard->stopCharging();
        return true;
    }
    return false;
}

bool neo_relayboardV2_node::serviceRelayBoardSetLCDMsg(neo_srvs::RelayBoardSetLCDMsg::Request &req, neo_srvs::RelayBoardSetLCDMsg::Response &res)
{
    if(m_bRelayBoardV2Available)
    {
        m_SerRelayBoard->writeLCD(req.message.c_str());
        res.success = true;
        return true;
    }
    res.success = false;
    return false;
}

//#############################################
//  END  RelayBoardV2 Service Callbacks
//#############################################

//----------------------END RelayBoardV2-----------------------------------------------------------------------
//--------------------------Motor Ctrl-------------------------------------------------------------------------

//#############################################
//       RelayBoardV2 Motor Control
//#############################################
void neo_relayboardV2_node::PublishJointStates()
{
    if(!m_bRelayBoardV2Available) return;
	if(m_iactive_motors == 0) return;

	long lEnc[8] = {0,0,0,0,0,0,0,0};
	long lEncS[8] = {0,0,0,0,0,0,0,0};
	int iStatus[8] = {0,0,0,0,0,0,0,0};
	static float sfLastPos[8] = {0,0,0,0,0,0,0,0};
	
	sensor_msgs::JointState state;

	//Publish Data for all possible Motors
	state.name.resize(8);
	state.position.resize(8);
	state.velocity.resize(8);

	//TODO Joint Names einfügen
	//for(int i = 0; i<anz_drives; i++)  state.name[i] = joint_names[i];

	//Motor Data from MSG Handler for each Motor
	//Enc (4 Byte), EncS (4 Byte) and Status (2 Byte) for each Motor 
	for(int i = 0; i<8; i++)
	{
        state.name[i] = m_Drives[i].sName.c_str();
		m_SerRelayBoard->getMotorEnc(i,&lEnc[i]);
		m_SerRelayBoard->getMotorEncS(i,&lEncS[i]);
		m_SerRelayBoard->getMotorState(i,&iStatus[i]);
		//m_SerRelayBoard->setMotorDesiredEncS(i, 20000);
		state.position[i] = (float)lEnc[i] - sfLastPos[i];
		sfLastPos[i] = (float)lEnc[i];
		//ROS_INFO("Motor %d: Enc: %f",i,(float)lEnc[i]);
		state.velocity[i] = m_Drives[i].iSign * m_Drives[i].convIncrPerPeriodToRadS((float)lEncS[i]);	 
	}
	topicPub_drives.publish(state);
}

void neo_relayboardV2_node::getNewVelocitiesFomTopic(const trajectory_msgs::JointTrajectory jt)
{
    if(!m_bRelayBoardV2Available) return;
	if(m_iactive_motors == 0) return;
	double dvelocity = 0.0;
	trajectory_msgs::JointTrajectoryPoint point = jt.points[0];
    for(int i=0; i<m_imotor_count; i++)
    {
        //convert velocities [rad/s] -> [incr/period]
        //ROS_INFO("Motor: %d ; Vel: %d [rad]; Vel: %d [incr/period]",i,point.velocities[i],dvelocity);
        dvelocity = m_Drives[i].iSign * m_Drives[i].convRadSToIncrPerPeriod(point.velocities[i]);
        //check if velocity is too high -> limit velocity
        /*if(MathSup::limit((int)&dvelocity, (int)Drives[i].getVelMax()) != 0)
        {
            ROS_ERROR("Velocity for motor %d limited",i+2);
        }*/
        //send Data to MSG Handler
        m_SerRelayBoard->setMotorDesiredEncS(i, (long)dvelocity);
    }
}

//#############################################
//    END RelayBoardV2 Motor Control
//#############################################
//----------------------END Motor Ctrl-------------------------------------------------------------------------
//-----------------------------USBoard-------------------------------------------------------------------------
void neo_relayboardV2_node::PublishUSBoardData()
{
    if(!m_bRelayBoardV2Available || !m_bUSBoardActive) return;
	int usSensors1[8];
	int usSensors2[8];
	int usAnalog[4];
	neo_msgs::USBoard usBoard;
	m_SerRelayBoard->getUSBoardData1To8(usSensors1);
	for(int i=0; i<8; i++) usBoard.sensor[i] = usSensors1[i];
	m_SerRelayBoard->getUSBoardData9To16(usSensors2);
	for(int i=0; i<8; i++) usBoard.sensor[i+8] = usSensors2[i];
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);
	for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];	

    //Publish raw data in neo_msgs::USBoard format
	topicPub_usBoard.publish(usBoard);
	
    //Additionally publish data in ROS sensor_msgs::Range format
    //-------------------------------------------SENSOR1--------------------------------------------------------
    if(m_bUSBoardSensorActive[0])
    {
        std_msgs::Header USRange1Header;
        sensor_msgs::Range USRange1Msg;
        //create USRanger1Msg
        //fill in header
        USRange1Header.seq = 1; 				//uint32
        USRange1Header.stamp = ros::Time::now(); 		//time
        USRange1Header.frame_id = "usrangesensor1";		//string

        USRange1Msg.header = USRange1Header;
        USRange1Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange1Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange1Msg.min_range = 0.1; 				//float32 [m]
        USRange1Msg.max_range = 1.2; 				//float32 [m]
        USRange1Msg.range = ((float)usBoard.sensor[0]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor1.publish(USRange1Msg);
    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------------------------SENSOR2--------------------------------------------------------
    if(m_bUSBoardSensorActive[1])
    {
        std_msgs::Header USRange2Header;
        sensor_msgs::Range USRange2Msg;
        //create USRanger2Msg
        //fill in header
        USRange2Header.seq = 1; 				//uint32
        USRange2Header.stamp = ros::Time::now(); 		//time
        USRange2Header.frame_id = "usrangesensor2";		//string

        USRange2Msg.header = USRange2Header;
        USRange2Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange2Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange2Msg.min_range = 0.1; 				//float32 [m]
        USRange2Msg.max_range = 1.2; 				//float32 [m]
        USRange2Msg.range = ((float)usBoard.sensor[1]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor2.publish(USRange2Msg);

    }
    //----------------------------------------------------------------------------------------------------------
	//-------------------------------------------SENSOR3--------------------------------------------------------
    if(m_bUSBoardSensorActive[2])
    {
        std_msgs::Header USRange3Header;
        sensor_msgs::Range USRange3Msg;
        //create USRanger3Msg
        //fill in header
        USRange3Header.seq = 1; 				//uint32
        USRange3Header.stamp = ros::Time::now(); 		//time
        USRange3Header.frame_id = "usrangesensor3";		//string

        USRange3Msg.header = USRange3Header;
        USRange3Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange3Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange3Msg.min_range = 0.1; 				//float32 [m]
        USRange3Msg.max_range = 1.2; 				//float32 [m]
        USRange3Msg.range = ((float)usBoard.sensor[2]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor3.publish(USRange3Msg);
    }
    //----------------------------------------------------------------------------------------------------------
	//-------------------------------------------SENSOR4--------------------------------------------------------
    if(m_bUSBoardSensorActive[3])
    {
        std_msgs::Header USRange4Header;
        sensor_msgs::Range USRange4Msg;
        //create USRanger4Msg
        //fill in header
        USRange4Header.seq = 1; 				//uint32
        USRange4Header.stamp = ros::Time::now(); 		//time
        USRange4Header.frame_id = "usrangesensor4";		//string

        USRange4Msg.header = USRange4Header;
        USRange4Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange4Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange4Msg.min_range = 0.1; 				//float32 [m]
        USRange4Msg.max_range = 1.2; 				//float32 [m]
        USRange4Msg.range = ((float)usBoard.sensor[3]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor4.publish(USRange4Msg);
    }
    //----------------------------------------------------------------------------------------------------------
	//-------------------------------------------SENSOR5--------------------------------------------------------
    if(m_bUSBoardSensorActive[4])
    {
        std_msgs::Header USRange5Header;
        sensor_msgs::Range USRange5Msg;
        //create USRanger5Msg
        //fill in header
        USRange5Header.seq = 1; 				//uint32
        USRange5Header.stamp = ros::Time::now(); 		//time
        USRange5Header.frame_id = "usrangesensor5";		//string

        USRange5Msg.header = USRange5Header;
        USRange5Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange5Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange5Msg.min_range = 0.1; 				//float32 [m]
        USRange5Msg.max_range = 1.2; 				//float32 [m]
        USRange5Msg.range = ((float)usBoard.sensor[4]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor5.publish(USRange5Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR6--------------------------------------------------------
    if(m_bUSBoardSensorActive[5])
    {
        std_msgs::Header USRange6Header;
        sensor_msgs::Range USRange6Msg;
        //create USRanger6Msg
        //fill in header
        USRange6Header.seq = 1; 				//uint32
        USRange6Header.stamp = ros::Time::now(); 		//time
        USRange6Header.frame_id = "usrangesensor6";		//string

        USRange6Msg.header = USRange6Header;
        USRange6Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange6Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange6Msg.min_range = 0.1; 				//float32 [m]
        USRange6Msg.max_range = 1.2; 				//float32 [m]
        USRange6Msg.range = ((float)usBoard.sensor[5]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor6.publish(USRange6Msg);
    }
	//------------------------------------------------------------------------------------------------------------
	
	//-------------------------------------------SENSOR7--------------------------------------------------------
    if(m_bUSBoardSensorActive[6])
    {
        std_msgs::Header USRange7Header;
        sensor_msgs::Range USRange7Msg;
        //create USRanger7Msg
        //fill in header
        USRange7Header.seq = 1; 				//uint32
        USRange7Header.stamp = ros::Time::now(); 		//time
        USRange7Header.frame_id = "usrangesensor7";		//string

        USRange7Msg.header = USRange7Header;
        USRange7Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange7Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange7Msg.min_range = 0.1; 				//float32 [m]
        USRange7Msg.max_range = 1.2; 				//float32 [m]
        USRange7Msg.range = ((float)usBoard.sensor[6]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor7.publish(USRange7Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR8--------------------------------------------------------
    if(m_bUSBoardSensorActive[7])
    {
        std_msgs::Header USRange8Header;
        sensor_msgs::Range USRange8Msg;
        //create USRanger8Msg
        //fill in header
        USRange8Header.seq = 1; 				//uint32
        USRange8Header.stamp = ros::Time::now(); 		//time
        USRange8Header.frame_id = "usrangesensor8";		//string

        USRange8Msg.header = USRange8Header;
        USRange8Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange8Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange8Msg.min_range = 0.1; 				//float32 [m]
        USRange8Msg.max_range = 1.2; 				//float32 [m]
        USRange8Msg.range = ((float)usBoard.sensor[7]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor8.publish(USRange8Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR9--------------------------------------------------------
    if(m_bUSBoardSensorActive[8])
    {
        std_msgs::Header USRange9Header;
        sensor_msgs::Range USRange9Msg;
        //create USRanger4Msg
        //fill in header
        USRange9Header.seq = 1; 				//uint32
        USRange9Header.stamp = ros::Time::now(); 		//time
        USRange9Header.frame_id = "usrangesensor9";		//string

        USRange9Msg.header = USRange9Header;
        USRange9Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange9Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange9Msg.min_range = 0.1; 				//float32 [m]
        USRange9Msg.max_range = 1.2; 				//float32 [m]
        USRange9Msg.range = ((float)usBoard.sensor[8]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor9.publish(USRange9Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR10-------------------------------------------------------
    if(m_bUSBoardSensorActive[9])
    {
        std_msgs::Header USRange10Header;
        sensor_msgs::Range USRange10Msg;
        //create USRanger10Msg
        //fill in header
        USRange10Header.seq = 1; 				//uint32
        USRange10Header.stamp = ros::Time::now(); 		//time
        USRange10Header.frame_id = "usrangesensor10";		//string

        USRange10Msg.header = USRange10Header;
        USRange10Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange10Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange10Msg.min_range = 0.1; 				//float32 [m]
        USRange10Msg.max_range = 1.2; 				//float32 [m]
        USRange10Msg.range = ((float)usBoard.sensor[9]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor10.publish(USRange10Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR11-------------------------------------------------------
    if(m_bUSBoardSensorActive[10])
    {
        std_msgs::Header USRange11Header;
        sensor_msgs::Range USRange11Msg;
        //create USRanger11Msg
        //fill in header
        USRange11Header.seq = 1; 				//uint32
        USRange11Header.stamp = ros::Time::now(); 		//time
        USRange11Header.frame_id = "usrangesensor11";		//string

        USRange11Msg.header = USRange11Header;
        USRange11Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange11Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange11Msg.min_range = 0.1; 				//float32 [m]
        USRange11Msg.max_range = 1.2; 				//float32 [m]
        USRange11Msg.range = ((float)usBoard.sensor[10]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor11.publish(USRange11Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR12-------------------------------------------------------
    if(m_bUSBoardSensorActive[11])
    {
        std_msgs::Header USRange12Header;
        sensor_msgs::Range USRange12Msg;
        //create USRanger12Msg
        //fill in header
        USRange12Header.seq = 1; 				//uint32
        USRange12Header.stamp = ros::Time::now(); 		//time
        USRange12Header.frame_id = "usrangesensor12";		//string

        USRange12Msg.header = USRange12Header;
        USRange12Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange12Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange12Msg.min_range = 0.1; 				//float32 [m]
        USRange12Msg.max_range = 1.2; 				//float32 [m]
        USRange12Msg.range = ((float)usBoard.sensor[11]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor12.publish(USRange12Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR13-------------------------------------------------------
    if(m_bUSBoardSensorActive[12])
    {
        std_msgs::Header USRange13Header;
        sensor_msgs::Range USRange13Msg;
        //create USRanger11Msg
        //fill in header
        USRange13Header.seq = 1; 				//uint32
        USRange13Header.stamp = ros::Time::now(); 		//time
        USRange13Header.frame_id = "usrangesensor13";		//string

        USRange13Msg.header = USRange13Header;
        USRange13Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange13Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange13Msg.min_range = 0.1; 				//float32 [m]
        USRange13Msg.max_range = 1.2; 				//float32 [m]
        USRange13Msg.range = ((float)usBoard.sensor[12]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor13.publish(USRange13Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR14-------------------------------------------------------
    if(m_bUSBoardSensorActive[13])
    {
        std_msgs::Header USRange14Header;
        sensor_msgs::Range USRange14Msg;
        //create USRanger14Msg
        //fill in header
        USRange14Header.seq = 1; 				//uint32
        USRange14Header.stamp = ros::Time::now(); 		//time
        USRange14Header.frame_id = "usrangesensor14";		//string

        USRange14Msg.header = USRange14Header;
        USRange14Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange14Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange14Msg.min_range = 0.1; 				//float32 [m]
        USRange14Msg.max_range = 1.2; 				//float32 [m]
        USRange14Msg.range = ((float)usBoard.sensor[13]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor14.publish(USRange14Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR15-------------------------------------------------------
    if(m_bUSBoardSensorActive[14])
    {
        std_msgs::Header USRange15Header;
        sensor_msgs::Range USRange15Msg;
        //create USRanger15Msg
        //fill in header
        USRange15Header.seq = 1; 				//uint32
        USRange15Header.stamp = ros::Time::now(); 		//time
        USRange15Header.frame_id = "usrangesensor15";		//string

        USRange15Msg.header = USRange15Header;
        USRange15Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange15Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange15Msg.min_range = 0.1; 				//float32 [m]
        USRange15Msg.max_range = 1.2; 				//float32 [m]
        USRange15Msg.range = ((float)usBoard.sensor[14]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor15.publish(USRange15Msg);
    }
	//------------------------------------------------------------------------------------------------------------

	//-------------------------------------------SENSOR16-------------------------------------------------------
    if(m_bUSBoardSensorActive[15])
    {
        std_msgs::Header USRange16Header;
        sensor_msgs::Range USRange16Msg;
        //create USRanger16Msg
        //fill in header
        USRange16Header.seq = 1; 				//uint32
        USRange16Header.stamp = ros::Time::now(); 		//time
        USRange16Header.frame_id = "usrangesensor16";		//string

        USRange16Msg.header = USRange16Header;
        USRange16Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange16Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange16Msg.min_range = 0.1; 				//float32 [m]
        USRange16Msg.max_range = 1.2; 				//float32 [m]
        USRange16Msg.range = ((float)usBoard.sensor[15]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor16.publish(USRange16Msg);
    }
	//------------------------------------------------------------------------------------------------------------
}
/*void neo_relayboardV2_node::startUSBoard(const std_msgs::Int16& configuration)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->startUSBoard(configuration.data);
}

void neo_relayboardV2_node::stopUSBoard(const std_msgs::Empty& empty)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->stopUSBoard();
}*/
//-----------------------------END USBoard---------------------------------------------------------------------
//---------------------------------IOBoard---------------------------------------------------------------------

void neo_relayboardV2_node::PublishIOBoard()
{
    if(!m_bRelayBoardV2Available || !m_bIOBoardActive) return;

    neo_msgs::IOBoard msg_IOBoard;
    //bool[16] digital_inputs			# state for all digital inputs
    //bool[16] digital_outputs          # state for all digital outputs
    //uint8[4] analog_inputs			# analog input values

    int iDigInData = 0;
    m_SerRelayBoard->getIOBoardDigIn(&iDigInData);

    int iDigOutData = 0;
    m_SerRelayBoard->getIOBoardDigOut(&iDigOutData);

    for(uint16_t iIOCnt = 0; iIOCnt < 16; iIOCnt++)
    {
        int iMask = 1 << iIOCnt;

        if((iDigInData & iMask) > 0)
        {
            msg_IOBoard.digital_inputs[iIOCnt] = true;
        }
        else
        {
            msg_IOBoard.digital_inputs[iIOCnt] = false;
        }

        if((iDigOutData & iMask) > 0)
        {
            msg_IOBoard.digital_outputs[iIOCnt] = true;
        }
        else
        {
            msg_IOBoard.digital_outputs[iIOCnt] = false;
        }
    }

    int *pointer = 0;
    int analogin[8];
    pointer = analogin;
    m_SerRelayBoard->getIOBoardAnalogIn(pointer);
    for(int i=0;i <8; i++) msg_IOBoard.analog_inputs[i] = pointer[i];

    topicPub_IOBoard.publish(msg_IOBoard);

}

bool neo_relayboardV2_node::serviceIOBoardSetDigOut(neo_srvs::IOBoardSetDigOut::Request &req, neo_srvs::IOBoardSetDigOut::Response &res)
{
    if(!m_bRelayBoardV2Available || !m_bIOBoardActive)
    {
        res.success = false;
        return false;
    }
    else
    {
        //check if DigOut ID is valid
        if(req.id >= 0 && req.id < 16)
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
