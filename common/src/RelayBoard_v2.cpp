/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Neobotix GmbH
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


#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include "../include/RelayBoard_v2.h"


//-----------------------------------------------



#define RS422_BAUDRATE 420000

#define RS422_TIMEOUT 0.025


RelayBoardV2::RelayBoardV2()
{
	m_iNumBytesSend = 0;
	m_iNumBytesRec = 0;

    m_bRelayData = false;
    m_bLCDData= false;
    m_bIOBoardData= false;
    m_bUSBoardData= false;
    m_bSpeakerData= false;
    m_bChargeData= false;

	m_REC_MSG.iRelayBoard_Status = 0;
	m_REC_MSG.iCharging_Current = 0;
	m_REC_MSG.iCharging_State = 0;
	m_REC_MSG.iBattery_Voltage = 0;
	m_REC_MSG.iKey_Pad = 0;
	m_REC_MSG.iTemp_Sensor = 0;
	m_REC_MSG.iIODig_In = 0;
	m_REC_MSG.iIOBoard_State = 0;
	m_REC_MSG.iUSBoard_State = 0;

	for(int a = 0; a<8;a++)
	{
        m_Motor[a].bActive = false;
        m_Motor[a].bAvailable = false;
        m_Motor[a].bHoming = false;
        m_Motor[a].bHomed = false;
		m_Motor[a].lEnc = 0.0;
		m_Motor[a].lEncS = 0.0;
		m_Motor[a].iStatus = 0;
		m_Motor[a].ldesiredEncS = 0.0;
	}

    m_IOBoard.bActive = false;
    m_IOBoard.bAvailable = false;

    m_USBoard.bActive = false;
    m_USBoard.bAvailable = false;

	m_S_MSG.iSoftEM = 0;
	m_S_MSG.iCmdRelayBoard = 0;
	m_S_MSG.IOBoard_Cmd = 0;
	m_S_MSG.USBoard_Cmd = 0;
	m_S_MSG.Speaker = 0;
	m_S_MSG.SpeakerLoud = 0;

	m_blogging = false;

    //ROS_INFO("Starting RelayBoardV2 Node\n");

}
//-----------------------------------------------
RelayBoardV2::~RelayBoardV2()
{
	m_SerIO.closeIO();
}
//----------------------------------------------
int RelayBoardV2::evalRxBuffer()
{
	int errorFlag = NO_ERROR;

	bool found_header = false;
	int waiting_cnt = 0;
	int waiting_cnt2 = 0;
	int msg_type = 100;
	int error_cnt = 0;
	int no_data_cnt = 0;
	int BytesToRead = 0;
	int received_checksum = 0;
	int my_checksum = 0;
	const int c_iSizeBuffer = 130;//4096;
	int iNrBytesRead = 0;
	unsigned char cDat[c_iSizeBuffer];
	unsigned char cHeader[4] = {0x00, 0x00, 0x00, 0x00};
	//ROS_INFO("EVAL RX");
	while(found_header == false)
	{
		if(m_SerIO.getSizeRXQueue() >= 1)
		{
			waiting_cnt = 0;
			//Read Header
			cHeader[3] = cHeader[2];
			cHeader[2] = cHeader[1];
			cHeader[1] = cHeader[0];
			iNrBytesRead = m_SerIO.readBlocking((char*)&cHeader[0], 1);

			if((cHeader[3] == 0x08) && (cHeader[2] == 0xFE) && (cHeader[1] == 0xEF) && (cHeader[0] == 0x08))
			{
				//Update Msg
                //ROS_INFO("Update Msg");
				msg_type = 1;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0x80) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Config Msg
                //ROS_INFO("Config Msg");
				msg_type = 2;
				found_header = true;
			}
			else if((cHeader[3] == 0x02) && (cHeader[2] == 0xFF) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
			{
				//Error Msg
                //ROS_INFO("Error Msg");
				msg_type = 3;
				found_header = true;
			}
			if(++error_cnt > 20)
			{
				//No Header in first 20 Bytes -> Error
				//flush input
                return RX_NO_HEADER;
			}

		}
		else
		{
			waiting_cnt++;
			if(waiting_cnt > 60000)
			{
				waiting_cnt = 0;
				waiting_cnt2++;
				if(waiting_cnt2 > 10)
				{
					//resend
                    return RX_TIMEOUT;
				} 
			}
		}
    }//end while(found_header == false)

    //look up number of bytes to read
	switch(msg_type)
	{
		case 1: BytesToRead = m_iNumBytesRec - 4;
				break;
		case 2: BytesToRead = 6;
				break;
		case 3: BytesToRead = 3;
				break;
        default: return RX_UNKNOWN_MSG;
	}

    //wait for data in RXQueue
	error_cnt = 0;
	while(m_SerIO.getSizeRXQueue() < BytesToRead)
	{
		usleep(2000);
	}

    //read data from RXQueue
	iNrBytesRead = m_SerIO.readBlocking((char*)&cDat[0], BytesToRead);

    //log data to file
	if(m_blogging == true)
	{
		log_to_file(2, cDat); //direction 1 = transmitted; 2 = recived
	}

    //calc Checksum
	my_checksum += cHeader[3];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[2];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[1];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[0];
	for(int e = 0; e < iNrBytesRead - 2; e++)
	{
		my_checksum %= 0xFF00;
		my_checksum += cDat[e];
	}

    //read checksum from msg
	received_checksum = (cDat[BytesToRead - 1] << 8);
	received_checksum += cDat[BytesToRead - 2];
	//ROS_INFO("Fast Fertig Eval RX");

    //check if checksum is equal to received checksum
	if(received_checksum != my_checksum)
	{
		//Wrong Checksum
        return RX_WRONG_CHECKSUM;
	}

    //handle msg (depends on msg type)
	if(msg_type == 1)
	{
        //Update Msg
		convRecMsgToData(&cDat[0]);
        return RX_UPDATE_MSG;
	}
	else if(msg_type == 2)
	{
        //Config Msg
		m_iFoundMotors = cDat[0];
		m_iHomedMotors = cDat[1];
		m_iFoundExtHardware = cDat[2];
		m_iConfigured = cDat[3];

        //Set motors available
        if((m_iFoundMotors & 0x80) == 0x80) m_Motor[7].bAvailable = true;
        else m_Motor[7].bAvailable = false;
        if((m_iFoundMotors & 0x40) == 0x40) m_Motor[6].bAvailable = true;
        else m_Motor[6].bAvailable = false;
        if((m_iFoundMotors & 0x20) == 0x20) m_Motor[5].bAvailable = true;
        else m_Motor[5].bAvailable = false;
        if((m_iFoundMotors & 0x10) == 0x10) m_Motor[4].bAvailable = true;
        else m_Motor[4].bAvailable = false;
        if((m_iFoundMotors & 0x8) == 0x8) m_Motor[3].bAvailable = true;
        else m_Motor[3].bAvailable = false;
        if((m_iFoundMotors & 0x4) == 0x4) m_Motor[2].bAvailable = true;
        else m_Motor[2].bAvailable = false;
        if((m_iFoundMotors & 0x2) == 0x2) m_Motor[1].bAvailable = true;
        else m_Motor[1].bAvailable = false;
        if((m_iFoundMotors & 0x1) == 0x1) m_Motor[0].bAvailable = true;
        else m_Motor[0].bAvailable = false;

        //Set motors homed
        if((m_iHomedMotors & 0x80) == 0x80) m_Motor[7].bHomed = true;
        else m_Motor[7].bHomed = false;
        if((m_iHomedMotors & 0x40) == 0x40) m_Motor[6].bHomed = true;
        else m_Motor[6].bHomed = false;
        if((m_iHomedMotors & 0x20) == 0x20) m_Motor[5].bHomed = true;
        else m_Motor[5].bHomed = false;
        if((m_iHomedMotors & 0x10) == 0x10) m_Motor[4].bHomed = true;
        else m_Motor[4].bHomed = false;
        if((m_iHomedMotors & 0x8) == 0x8) m_Motor[3].bHomed = true;
        else m_Motor[3].bHomed = false;
        if((m_iHomedMotors & 0x4) == 0x4) m_Motor[2].bHomed = true;
        else m_Motor[2].bHomed = false;
        if((m_iHomedMotors & 0x2) == 0x2) m_Motor[1].bHomed = true;
        else m_Motor[1].bHomed = false;
        if((m_iHomedMotors & 0x1) == 0x1) m_Motor[0].bHomed = true;
        else m_Motor[0].bHomed = false;

        //Set ext hardware available
        if((m_iFoundExtHardware & 0x2) == 0x2) m_USBoard.bAvailable = true;
        else m_USBoard.bAvailable = false;
        if((m_iFoundExtHardware & 0x1) == 0x1) m_IOBoard.bAvailable = true;
        else m_IOBoard.bAvailable = false;

		//ROS_INFO("Found Motors: %d",cDat[0]);
		//ROS_INFO("Homed Motors: %d",cDat[1]);
        //ROS_INFO("Ext Hardware: %d",cDat[2]);
		//ROS_INFO("Configuriert: %d",cDat[3]);
        return RX_CONFIG_MSG;
	}
	else if(msg_type == 3)
	{
        //Error Msg
        return RX_ERROR_MSG;
	}
	else
	{
        //unknown Msg
        return RX_UNKNOWN_MSG;
	}

    return RX_UNKNOWN_ERROR;
}
//----------------------------------Configuration--------------------------------------
int RelayBoardV2::init(const char* cdevice_name, int iactive_motors, int ihoming_motors, int iext_hardware, long lModulo0, long lModulo1, long lModulo2, long lModulo3, long lModulo4, long lModulo5, long lModulo6, long lModulo7)
{
    int openPortReturn = 99;
	m_SerIO.setDeviceName(cdevice_name);
	m_SerIO.setBaudRate(RS422_BAUDRATE);
    openPortReturn = m_SerIO.openIO();

    if(openPortReturn != 0)
    {
        //failed to open IOPort
        //ROS_ERROR("Failed to open Seriel Port - return %d",openPortReturn);
        return INIT_OPEN_PORT_FAILED;
    }

	unsigned char cConfig_Data[33]; //4 Byte Header 3 Config Bytes 24 Byte Modulo 2 Byte Checksum
	int iChkSum = 0;
	int byteswritten = 0;
    int evalRXreturn = 0;
	int iNumDrives = 0;
	//Header
	cConfig_Data[0] = 0x02;
	cConfig_Data[1] = 0x80;
	cConfig_Data[2] = 0xD6;
	cConfig_Data[3] = 0x02;
	//-------------configuration Bytes-----------------------------------
	cConfig_Data[4] = iactive_motors;
	cConfig_Data[5] = ihoming_motors;
	cConfig_Data[6] = iext_hardware;

	//-------------set motors active-------------------------------------
    if((iactive_motors & 0x80) == 0x80) m_Motor[7].bActive = true;
    else m_Motor[7].bActive = false;
    if((iactive_motors & 0x40) == 0x40) m_Motor[6].bActive = true;
    else m_Motor[6].bActive = false;
    if((iactive_motors & 0x20) == 0x20) m_Motor[5].bActive = true;
    else m_Motor[5].bActive = false;
    if((iactive_motors & 0x10) == 0x10) m_Motor[4].bActive = true;
    else m_Motor[4].bActive = false;
    if((iactive_motors & 0x8) == 0x8) m_Motor[3].bActive = true;
    else m_Motor[3].bActive = false;
    if((iactive_motors & 0x4) == 0x4) m_Motor[2].bActive = true;
    else m_Motor[2].bActive = false;
    if((iactive_motors & 0x2) == 0x2) m_Motor[1].bActive = true;
    else m_Motor[1].bActive = false;
    if((iactive_motors & 0x1) == 0x1) m_Motor[0].bActive = true;
    else m_Motor[0].bActive = false;

    //-------------set homeing active-------------------------------------
    if((ihoming_motors & 0x80) == 0x80) m_Motor[7].bHoming = true;
    else m_Motor[7].bHoming = false;
    if((ihoming_motors & 0x40) == 0x40) m_Motor[6].bHoming = true;
    else m_Motor[6].bHoming = false;
    if((ihoming_motors & 0x20) == 0x20) m_Motor[5].bHoming = true;
    else m_Motor[5].bHoming = false;
    if((ihoming_motors & 0x10) == 0x10) m_Motor[4].bHoming = true;
    else m_Motor[4].bHoming = false;
    if((ihoming_motors & 0x8) == 0x8) m_Motor[3].bHoming = true;
    else m_Motor[3].bHoming = false;
    if((ihoming_motors & 0x4) == 0x4) m_Motor[2].bHoming = true;
    else m_Motor[2].bHoming = false;
    if((ihoming_motors & 0x2) == 0x2) m_Motor[1].bHoming = true;
    else m_Motor[1].bHoming = false;
    if((ihoming_motors & 0x1) == 0x1) m_Motor[0].bHoming = true;
    else m_Motor[0].bHoming = false;
	//---------------Modulo for all Motors-------------------------------
	//Motor1
	cConfig_Data[7] = lModulo0 >> 16;
	cConfig_Data[8] = lModulo0 >> 8;
	cConfig_Data[9] = lModulo0;
	//Motor2
	cConfig_Data[10] = lModulo1 >> 16;
	cConfig_Data[11] = lModulo1 >> 8;
	cConfig_Data[12] = lModulo1;
	//Motor3
	cConfig_Data[13] = lModulo2 >> 16;
	cConfig_Data[14] = lModulo2 >> 8;
	cConfig_Data[15] = lModulo2;
	//Motor4
	cConfig_Data[16] = lModulo3 >> 16;
	cConfig_Data[17] = lModulo3 >> 8;
	cConfig_Data[18] = lModulo3;
	//Motor5
	cConfig_Data[19] = lModulo4 >> 16;
	cConfig_Data[20] = lModulo4 >> 8;
	cConfig_Data[21] = lModulo4;
	//Motor6
	cConfig_Data[22] = lModulo5 >> 16;
	cConfig_Data[23] = lModulo5 >> 8;
	cConfig_Data[24] = lModulo5;
	//Motor7
	cConfig_Data[25] = lModulo6 >> 16;
	cConfig_Data[26] = lModulo6 >> 8;
	cConfig_Data[27] = lModulo6;
	//Motor8
	cConfig_Data[28] = lModulo7 >> 16;
	cConfig_Data[29] = lModulo7 >> 8;
	cConfig_Data[30] = lModulo7;

	//-----------------Calc m_iNumBytesRec-----------------------------
	m_iNumBytesRec = 4; //4Byte Header
	m_iNumBytesRec += 13; //11Byte ActRelayboardConfig, State, Charging Current, Charging State, Batt Voltage, Keypad, Temp
	for(int g = 0; g < 8;g++)
	{
        if(m_Motor[g].bActive)
		{
			iNumDrives++;
		}
	}
	//ROS_INFO("Number of Drives %i",iNumDrives);
	m_iNumBytesRec += (iNumDrives*10); //10 Byte pro active Motor

	if((iext_hardware & 1) == 1) //IOBoard
	{
        //ROS_INFO("Ext. Hardware: IOBoard");
		m_iNumBytesRec += 20;
	}
	if((iext_hardware & 0x2) == 0x2) //USBoard
	{
        //ROS_INFO("Ext. Hardware: USBoard");
		m_iNumBytesRec += 26;
	}

	m_iNumBytesRec += 2; //2Byte Checksum
	//ROS_INFO("iNumBytesRec: %d",m_iNumBytesRec);
	//----------------Calc Checksum-------------------------------------
	for(int i=4;i<=30;i++)    //i=4 => Header not added to Checksum
	{
		iChkSum %= 0xFF00;
		iChkSum += cConfig_Data[i];
	}
	//----------------END Calc Checksum---------------------------------

	//----------------Add Checksum to Data------------------------------
	cConfig_Data[31] = iChkSum >> 8;
	cConfig_Data[32] = iChkSum;
	//------------END Add Checksum to Data------------------------------
	if(m_blogging == true)
	{
		log_to_file(3, cConfig_Data); //direction 1 = transmitted; 2 = recived ; 3 = config
	}
	byteswritten = m_SerIO.writeIO((char*)cConfig_Data,33);
	if(byteswritten != 33)
	{
		//ROS_ERROR("FAILED: Could not write data to serial port!");
        return INIT_WRITE_FAILED;
	}
	else
	{
		//ROS_INFO("Waiting for RelayBoard....");
	}

    evalRXreturn = evalRxBuffer();

    if(evalRXreturn == RX_CONFIG_MSG)
	{
        //check config

		//Check received Data
		if(m_iConfigured == 1)
		{
            if(!checkConfig())
            {
                return INIT_CONFIG_CHANGED;
            }
            //ROS_INFO("Configured");
            return INIT_CONFIG_OK;
		}
		else
		{
            //ROS_ERROR("FAILED: Invalid configuration");
            return INIT_CONFIG_FAILED;
		}
	}
	
    return INIT_UNKNOWN_ERROR;
	

}
//-----------------------------------------------
void RelayBoardV2::getConfig(int* ActiveMotors, int* HomedMotors, int* ExtHardware, int* Configured)
{
    m_Mutex.lock();

    *ActiveMotors = m_iFoundMotors;
    *HomedMotors = m_iHomedMotors;
    *ExtHardware = m_iFoundExtHardware;
    *Configured = m_iConfigured;

    m_Mutex.unlock();
}
//-----------------------------------------------
bool RelayBoardV2::checkConfig()
{
    //returns true is config is ok
    //false otherwise

    bool config_ok = true;

    //Motor1
    if(m_Motor[0].bActive)
    {
        if(!m_Motor[0].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor1 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[0].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor1 is NOT active but available!");
        }
    }
    //Motor2
    if(m_Motor[1].bActive)
    {
        if(!m_Motor[1].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor2 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[1].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor2 is NOT active but available!");
        }
    }
    //Motor3
    if(m_Motor[2].bActive)
    {
        if(!m_Motor[2].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor3 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[2].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor3 is NOT active but available!");
        }
    }
    //Motor4
    if(m_Motor[3].bActive)
    {
        if(!m_Motor[3].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor4 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[3].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor4 is NOT active but available!");
        }
    }
    //Motor5
    if(m_Motor[4].bActive)
    {
        if(!m_Motor[4].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor5 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[4].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor5 is NOT active but available!");
        }
    }
    //Motor6
    if(m_Motor[5].bActive)
    {
        if(!m_Motor[5].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor6 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[5].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor6 is NOT active but available!");
        }
    }
    //Motor7
    if(m_Motor[6].bActive)
    {
        if(!m_Motor[6].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor7 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[6].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor7 is NOT active but available!");
        }
    }
    //Motor8
    if(m_Motor[7].bActive)
    {
        if(!m_Motor[7].bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("Motor8 is active but NOT available!");
        }
    }
    else
    {
        if(m_Motor[7].bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("Motor8 is NOT active but available!");
        }
    }
    //IOBoard
    if(m_IOBoard.bActive)
    {
        if(!m_IOBoard.bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("IOBoard is active but NOT available!");
        }
    }
    else
    {
        if(m_IOBoard.bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("IOBoard is NOT active but available!");
        }
    }
    //USBoard
    if(m_USBoard.bActive)
    {
        if(!m_USBoard.bAvailable)
        {
            config_ok = false;
            //ROS_ERROR("USBoard is active but NOT available!");
        }
    }
    else
    {
        if(m_USBoard.bAvailable)
        {
            //config_ok = false;
            //ROS_WARN("USBoard is NOT active but available!");
        }
    }

    return config_ok;
}
//-----------------------------------------------
bool RelayBoardV2::shutdownPltf()
{
	m_SerIO.closeIO();
	return true;
}
//-----------------------------------------------
bool RelayBoardV2::isEMStop()
{
	if( (m_REC_MSG.iRelayBoard_Status & 0x0001) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
//-----------------------------------------------
void RelayBoardV2::setEMStop()
{		
	m_S_MSG.iSoftEM |= 0x01;
	ROS_ERROR("Software Emergency Stop AKTIVE");
}

//-----------------------------------------------
void RelayBoardV2::resetEMStop()
{	
	m_S_MSG.iSoftEM |= 0x02;
	ROS_ERROR("Software Emergency Stop INAKTIVE");
}
//-----------------------------------------------
bool RelayBoardV2::isScannerStop()
{
	if( (m_REC_MSG.iRelayBoard_Status & 0x0002) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
//-------------------------------------------------
int RelayBoardV2::sendDataToRelayBoard()
{

    int errorFlag = TX_OK;
	int iNrBytesWritten;

	unsigned char cMsg[80];

	m_Mutex.lock();
	//ROS_INFO("Converting...");
	m_iNumBytesSend = convDataToSendMsg(cMsg);
	//ROS_INFO("Fin");	
	m_SerIO.purgeTx();
	iNrBytesWritten = m_SerIO.writeIO((char*)cMsg,m_iNumBytesSend);

	if(iNrBytesWritten < m_iNumBytesSend) {
        errorFlag = TX_WRITE_FAILED;
		//ROS_ERROR("ZU wenig gesendet!");
	}
	//log
	if(m_blogging == true)
	{
		log_to_file(1, cMsg); //direction 1 = transmitted; 2 = recived
	}
	//m_LastTime = ros::Time::now();
	m_Mutex.unlock();

	return errorFlag;

};
//---------------------------------------------------------------------------------------------------------------------
// MotorCtrl
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::setMotorDesiredEncS(int imotor_nr, long lEncS)
{		
	m_Mutex.lock();
	
	m_Motor[imotor_nr].ldesiredEncS = lEncS;

	m_Mutex.unlock();
}
//-------------------------------------------------------------
void RelayBoardV2::getMotorEnc(int imotor_nr,long* lmotorEnc)
{
	m_Mutex.lock();
	
	*lmotorEnc = m_Motor[imotor_nr].lEnc;
	
	m_Mutex.unlock();

}
//-------------------------------------------------------------
void RelayBoardV2::getMotorEncS(int imotor_nr,long* lmotorEncS)
{
	m_Mutex.lock();
	
	*lmotorEncS = m_Motor[imotor_nr].lEncS;

	m_Mutex.unlock();
}
//-------------------------------------------------------------
void RelayBoardV2::getMotorState(int imotor_nr,int* imotorStatus)
{
	m_Mutex.lock();
	
	*imotorStatus = m_Motor[imotor_nr].iStatus;

	m_Mutex.unlock();
}
//-------------------------------------------------------------
bool RelayBoardV2::getMotorAvailable(int imotor_nr)
{
    return m_Motor[imotor_nr].bAvailable;
}

//-------------------------------------------------------------
bool RelayBoardV2::getMotorHomed(int imotor_nr)
{
    return m_Motor[imotor_nr].bHomed;
}

//-------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
// RelayBoard
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::getRelayBoardState(int* State)
{
	m_Mutex.lock();
	*State = m_REC_MSG.iRelayBoard_Status;
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getRelayBoardDigOut(int* DigOut)
{
	m_Mutex.lock();
	*DigOut = m_S_MSG.iCmdRelayBoard;
	m_Mutex.unlock();
}
//-------------------------------------------------------
bool RelayBoardV2::getRelayBoardDigOutState(int ID)
{

    /* Relay states are not evaluated from relayboardv2
     * -> All states are the same as was has been set
     */

    /* RELAY STATES BIT CODED:
     * LSB : Info Taste gedrueckt
     * Bit 2: Home Taste gedrueckt
     * Bit 3: Start Taste gedrueckt
     * Bit 4: Stop Taste gedrueckt
     * Bit 5: Bremsen loesen Taste gedrueckt
     * Bit 6: Reserviert
     * Bit 7: Reserviert
     * Bit 8: Reserviert
     * Bit 9: Reserviert
     * Bit 10: Reserviert
     * Bit 11: Reserviert
     * Bit 12: Reserviert
     * Bit 13: Reserviert
     * Bit 14: Reserviert
     * Bit 15: Reserviert
     * MSB: Reserviert
     */
    int iRelayStates = 0;
    getRelayBoardDigOut(&iRelayStates);

    //Charge Relay
    if(ID == RELAY_CHARGE)
    {
        if((iRelayStates & 1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //On Demand Relay 1
    else if(ID == RELAY_ON_DEMAND_1)
    {
        if((iRelayStates & 2))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //On Demand Relay 2
    else if(ID == RELAY_ON_DEMAND_2)
    {
        if((iRelayStates & 4))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //On Demand Relay 3
    else if(ID == RELAY_ON_DEMAND_3)
    {
        if((iRelayStates & 8))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

//-------------------------------------------------------
void RelayBoardV2::setRelayBoardDigOut(int iChannel, bool bOn)
{

  /*Bit 4: Relais 1 schalten
	Bit 5: Relais 2 schalten
	Bit 6: Relais 3 schalten
	Bit 7: Relais 4 schalten*/
    m_bRelayData = true;
	switch( iChannel)
	{
	case 1:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 8; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 8; }
		
		break;

	case 2:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 16; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 16; }

		break;

	case 3:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 32; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 32; }

		break;

	case 4:

		if(bOn) { m_S_MSG.iCmdRelayBoard |= 64; }
		else { m_S_MSG.iCmdRelayBoard &= ~ 64; }

		break;

	default:

		m_S_MSG.iCmdRelayBoard = 0;
	}
}
//-------------------------------------------------------
void RelayBoardV2::getTemperature(int16_t* temp)
{
	m_Mutex.lock();

	*temp = m_REC_MSG.iTemp_Sensor;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getBattVoltage(u_int16_t* battvolt)
{
	m_Mutex.lock();

	*battvolt = m_REC_MSG.iBattery_Voltage;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getChargingCurrent(int16_t* current)
{
	m_Mutex.lock();

	*current = m_REC_MSG.iCharging_Current;

	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getChargingStateByte(u_int16_t* state)
{
	m_Mutex.lock();

	*state = m_REC_MSG.iCharging_State;

	m_Mutex.unlock();
}
//-------------------------------------------------------
u_int16_t RelayBoardV2::getChargingState()
{

    u_int16_t iChargingState = 0;
    getChargingStateByte(&iChargingState);

    return iChargingState;
}
//-----------------------------------------------
void RelayBoardV2::writeLCD(const std::string& sText)
{
	int iSize;
    m_bLCDData = true;
	m_Mutex.lock();

	iSize = sText.size();
	
	for(int i = 0; i < 20; i++)
	{
		if(i < iSize)
		{
			m_S_MSG.LCD_Txt[i] = sText[i];
		}
		else
		{
			m_S_MSG.LCD_Txt[i] = ' ';
		}
	}
	m_Mutex.unlock();
}
//-------------------------------------------------------
void RelayBoardV2::getKeyPad(int* keypad)
{
	m_Mutex.lock();
	*keypad = m_REC_MSG.iKey_Pad;
	m_Mutex.unlock();
}
bool RelayBoardV2::getKeyState(int ID)
{

    /* KEY PAD BIT CODED:
     * LSB : Info Taste gedrueckt
     * Bit 2: Home Taste gedrueckt
     * Bit 3: Start Taste gedrueckt
     * Bit 4: Stop Taste gedrueckt
     * Bit 5: Bremsen loesen Taste gedrueckt
     * Bit 6: On Demand Key 1
     * Bit 7: On Demand Key 2
     * Bit 8: On Demand Key 3
     * Bit 9: Reserviert
     * Bit 10: Reserviert
     * Bit 11: Reserviert
     * Bit 12: Reserviert
     * Bit 13: Reserviert
     * Bit 14: Reserviert
     * Bit 15: Reserviert
     * MSB: Reserviert
     */

    int iButtons = 0;
    getKeyPad(&iButtons);

    //Buttons have inverted logic!

    //Info Button
    if(ID == KEY_INFO)
    {
        if(!(iButtons & 1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //Home Button
    else if(ID == KEY_HOME)
    {
        if(!(iButtons & 2))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //Start Button
    else if(ID == KEY_START)
    {
        if(!(iButtons & 4))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //Stop Button
    else if(ID == KEY_STOP)
    {
        if(!(iButtons & 8))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //release brake Button
    else if(ID == KEY_RELEASE_BRAKE)
    {
        if(!(iButtons & 16))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //on demand digital input
    else if(ID == KEY_ON_DEMAND_1)
    {
        if(!(iButtons & 32))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //on demand digital input
    else if(ID == KEY_ON_DEMAND_2)
    {
        if(!(iButtons & 64))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //on demand digital input
    else if(ID == KEY_ON_DEMAND_3)
    {
        if(!(iButtons & 128))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

//-------------------------------------------------------
void RelayBoardV2::startCharging()
{
	m_Mutex.lock();
    m_bRelayData = true;
	m_S_MSG.iCmdRelayBoard |= 1;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::stopCharging()
{
	m_Mutex.lock();
    m_bRelayData = true;
	m_S_MSG.iCmdRelayBoard &= ~ 1;
	m_Mutex.unlock();
}
//-----------------------------------------------
// IOBoard
//-----------------------------------------------
void RelayBoardV2::getIOBoardDigIn(int* DigIn)
{
	m_Mutex.lock();
	*DigIn = m_REC_MSG.iIODig_In;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getIOBoardDigOut(int* DigOut)
{
	m_Mutex.lock();
	*DigOut = m_S_MSG.IOBoard_Cmd;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::setIOBoardDigOut(int iChannel, bool bVal)
{
    m_bIOBoardData = true;
	int iMask;

	iMask = (1 << iChannel);
	
	if(bVal)
	{
		m_S_MSG.IOBoard_Cmd |= iMask;
	}
	else
	{
		m_S_MSG.IOBoard_Cmd &= ~iMask;
	}	
}
//-----------------------------------------------
void RelayBoardV2::getIOBoardAnalogIn(int* iAnalogIn)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 8; i++)
	{
		iAnalogIn[i] = m_REC_MSG.iIOAnalog_In[i];
	}

	m_Mutex.unlock();
}
//---------------------------------------------------------------------------------------------------------------------
// USBoard
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::startUSBoard(int iChannelActive)
{
	m_Mutex.lock();
    m_bUSBoardData = true;
	m_S_MSG.USBoard_Cmd = iChannelActive;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::stopUSBoard()
{
	m_Mutex.lock();
    m_bUSBoardData = true;
	m_S_MSG.USBoard_Cmd = 0x00;
	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardData1To8(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();
	
	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = m_REC_MSG.iUSSensor_Dist[i];
	}

	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardData9To16(int* piUSDistMM)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 8; i++)
	{
		piUSDistMM[i] = m_REC_MSG.iUSSensor_Dist[i + 8];
	}

	m_Mutex.unlock();
}
//-----------------------------------------------
void RelayBoardV2::getUSBoardAnalogIn(int* piAnalogIn)
{
	int i;

	m_Mutex.lock();

	for(i = 0; i < 4; i++)
	{
		piAnalogIn[i] = m_REC_MSG.iUSAnalog_In[i];
	}

	m_Mutex.unlock();
}
//---------------------------------------------------------------------------------------------------------------------
//Logging
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::enable_logging()
{
	m_blogging = true;
}
//-----------------------------------------------
void RelayBoardV2::disable_logging()
{
	m_blogging = false;
}
//-----------------------------------------------
void RelayBoardV2::log_to_file(int direction, unsigned char cMsg[])
{
	FILE * pFile;
	//Open Logfile
	pFile = fopen ("neo_relayboard_RX_TX_log.log","a");
	//Write Data to Logfile
	if(direction == 1)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<m_iNumBytesSend; i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	if(direction == 2)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<(m_iNumBytesRec + 2); i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	if(direction == 3)
	{
		fprintf (pFile, "\n\n Direction: %i", direction);
		for(int i=0; i<(33); i++)
			fprintf(pFile," %.2x", cMsg[i]);
		fprintf(pFile,"\n");
	}
	//Close Logfile
	fclose (pFile);
}
//---------------------------------------------------------------------------------------------------------------------
//Data Converter
//---------------------------------------------------------------------------------------------------------------------
void RelayBoardV2::convRecMsgToData(unsigned char cMsg[])
{
	//ROS_INFO("Convert Rec to Data");
	int data_in_message = 0;

	m_Mutex.lock();

	// convert data
	int iCnt = 0;

	//Has Data
	data_in_message = cMsg[iCnt];
	iCnt++;
	//Relayboard Status
	m_REC_MSG.iRelayBoard_Status = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Charging Current
	m_REC_MSG.iCharging_Current = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Charging State
	m_REC_MSG.iCharging_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Battery Voltage
	m_REC_MSG.iBattery_Voltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Keypad
	m_REC_MSG.iKey_Pad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;
	//Temp Sensor
	m_REC_MSG.iTemp_Sensor = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// IOBoard
	if((m_iFoundExtHardware & 0x1) == 0x1)
	{
		m_REC_MSG.iIODig_In = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
		for(int i = 0; i < 8; i++)
		{
			m_REC_MSG.iIOAnalog_In[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}

		m_REC_MSG.iIOBoard_State =  (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}
	// USBoard
	if((m_iFoundExtHardware & 0x2) == 0x2)
	{
		for(int i = 0; i < 16; i++)
		{
			m_REC_MSG.iUSSensor_Dist[i] = (cMsg[iCnt++]);
		}
		for(int i = 0; i < 4; i++)
		{
			m_REC_MSG.iUSAnalog_In[i] = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}

		m_REC_MSG.iUSBoard_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}
	//Motor Data
    if(m_Motor[0].bActive)
	{
		//Enc Data
		m_Motor[0].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[0].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[0].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[1].bActive)
	{
		//Enc Data
		m_Motor[1].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[1].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[1].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[2].bActive)
	{
		//Enc Data
        m_Motor[2].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[2].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[2].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[3].bActive)
	{
		//Enc Data
		m_Motor[3].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[3].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[3].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[4].bActive)
	{
		//Enc Data
		m_Motor[4].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[4].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[4].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[5].bActive)
	{
		//Enc Data
		m_Motor[5].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[5].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[5].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[6].bActive)
	{
		//Enc Data
		m_Motor[6].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[6].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[6].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}
    if(m_Motor[7].bActive)
	{
		//Enc Data
		m_Motor[7].lEnc = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//EncS Data
		m_Motor[7].lEncS = (cMsg[iCnt+3] << 24) + (cMsg[iCnt+2] << 16) + (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 4;
		//Motor Status
		m_Motor[7].iStatus = (cMsg[iCnt+1] << 8) + cMsg[iCnt];
		iCnt += 2;
	}

	m_Mutex.unlock();
}

//------------------------------------------------------------
int RelayBoardV2::convDataToSendMsg(unsigned char cMsg[])
{
	//m_Mutex.lock();
	int iCnt = 0;
	int iChkSum = 0;

    int iDataAvailable = 0;
    int bMotoData8 = false;
    int bMotoData4 = false;

    if((m_Motor[0].bActive) || (m_Motor[1].bActive) || (m_Motor[2].bActive) || (m_Motor[3].bActive))
	{
        bMotoData4 = true;
        if((m_Motor[4].bActive) || (m_Motor[5].bActive) || (m_Motor[6].bActive) || (m_Motor[7].bActive))
		{
            bMotoData8 = true;
		}
	}
	//First Bit not in use yet
    iDataAvailable = (bMotoData8 << 6) + (bMotoData4 << 5) + (m_bRelayData << 4) + (m_bLCDData << 3) + (m_bIOBoardData << 2) +(m_bUSBoardData << 1) + (m_bSpeakerData);
	//Data in Message:
	//Header
	cMsg[iCnt++] = 0x02;
	cMsg[iCnt++] = 0xD6;
	cMsg[iCnt++] = 0x80;
	cMsg[iCnt++] = 0x02;
	//has_data
	//soft_em
	//Motor9-6
	//Motor5-2
	//Relaystates
	//LCD Data
	//IO Data
	//US Data
	//Speaker Data
	//Checksum
    cMsg[iCnt++] = iDataAvailable;
	cMsg[iCnt++] = m_S_MSG.iSoftEM; //SoftEM

    if(bMotoData8)
	{
		//Motor 9
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[7].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[7].ldesiredEncS & 0xFF);
		//Motor 8
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[6].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[6].ldesiredEncS & 0xFF);
		//Motor 7
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[5].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[5].ldesiredEncS & 0xFF);
		//Motor 6
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[4].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[4].ldesiredEncS & 0xFF);
	}
    if(bMotoData4)
	{
		//Motor 5
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[3].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[3].ldesiredEncS & 0xFF);
		//Motor 4
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[2].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[2].ldesiredEncS & 0xFF);
		//Motor 3
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[1].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[1].ldesiredEncS & 0xFF);
		//Motor 2
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF000000) >> 24);
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF0000) >> 16);
		cMsg[iCnt++] = ((m_Motor[0].ldesiredEncS & 0xFF00) >> 8);
		cMsg[iCnt++] =  (m_Motor[0].ldesiredEncS & 0xFF);
	}
	//Relaystates
    if(m_bRelayData)
	{
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard >> 8;
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard;
	}
	//LCD Data
    if(m_bLCDData)
	{
		//ROS_INFO("Display: %s",m_S_MSG.LCD_Txt);
		for(int u = 0; u < 20; u++)
		{
			cMsg[iCnt++] = m_S_MSG.LCD_Txt[u];
		}
	}
	//IO Data
    if(m_bIOBoardData)
	{
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd;
	}
	//US Data
    if(m_bUSBoardData)
	{
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd;
	}
	//Speaker Data
    if(m_bSpeakerData)
	{
		cMsg[iCnt++] = m_S_MSG.Speaker >> 8;
		cMsg[iCnt++] = m_S_MSG.SpeakerLoud;
	}
	m_iNumBytesSend = iCnt;
	// calc checksum
	for(int i = 4; i < m_iNumBytesSend; i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}
	cMsg[m_iNumBytesSend] = iChkSum >> 8;
	cMsg[m_iNumBytesSend + 1] = iChkSum;
	
	//resett data indicators
    m_bRelayData = false;
    m_bLCDData = false;
    m_bIOBoardData = false;
    m_bUSBoardData = false;
    m_bSpeakerData = false;

	//m_Mutex.unlock();

	return m_iNumBytesSend + 2;
}
