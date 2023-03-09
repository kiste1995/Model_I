#include "zeta_mdrobot_motor_control/mdrobot_motor_control.hpp"

mdrobot_motor_control::mdrobot_motor_control()
{
	b_serialOpenOK = false;
	PacketNumber = 0;
	b_recvOK = false;
	b_SerialThreadRun = false;
	b_PacketOK = false;
	b_MDMCRunOK = false;
}

mdrobot_motor_control::~mdrobot_motor_control()
{
	PNTMainDataBC(OFF);

	b_SerialThreadRun = false;

	Stop();

	if(b_serialOpenOK)
		mc_serial.close();
}

void mdrobot_motor_control::recvThread_Run(void)
{
	b_SerialThreadRun = true;

	boost::thread* recv_thread = new boost::thread(boost::bind(&mdrobot_motor_control::RecvDatafromMC, this));
		
}

void mdrobot_motor_control::RecvDatafromMC(void)
{
	BYTE rbuff[100];
	BYTE rnum = 0;
	BYTE ret;

	ros::Rate rec_dealy(1000);

	step = 0;

	int srcnt = 0;
	
	while(b_SerialThreadRun)
	{
		if(b_serialOpenOK) {
			rnum = mc_serial.available();
			
			if(rnum > 0)
			{
				SetZeroMem(rbuff, 100);
				
				mc_serial.read(rbuff, rnum);

				ret = RecvDatatoBuff(rbuff, rnum);
				
			}
		}

		srcnt++;
		if(srcnt>=1000)
		{
			if(srcnt == 1000)
				checkMDMCRunning();

			if(srcnt == 1001)
			{
				srcnt = 0;

#ifdef _DEBUG_BASIC				
				if(isMDMCRun() != ON)
				{
					ROS_INFO_STREAM("ERROR : disconneted MD MC!!!");
				} else {
					ROS_INFO_STREAM("Connecting MD MC....");
				}
#endif				
			}
		}

		rec_dealy.sleep();
	}
}

RetByte mdrobot_motor_control::ShorttoByte(short sVal)
{
    RetByte RetVal;

    RetVal.LowByte = sVal & 0xff;
    RetVal.HighByte = (sVal >> 8) & 0xff;

    return RetVal;
}

int mdrobot_motor_control::BytetoShort(BYTE LowByte, BYTE HighByte)
{
    return (LowByte | (int)(HighByte << 8));
}

int mdrobot_motor_control::BytetoLInt(BYTE Data1Byte, BYTE Data2Byte, BYTE Data3Byte, BYTE Data4Byte)
{
    return ((int)Data1Byte | (int)(Data2Byte << 8) | (int)(Data3Byte << 16) | (int)(Data4Byte << 24));
}

int mdrobot_motor_control::InitSerial(void)
{
    // nBaudrate = 19200;
	nBaudrate = 115200;

    try
    {
        mc_serial.setPort("/dev/ttyUSB-MC");
        mc_serial.setBaudrate(nBaudrate);
		// 556 when baud is 19200, 1.8ms
		//1667 when baud is 57600, 0.6ms
		//2857 when baud is 115200, 0.35ms
        serial::Timeout to = serial::Timeout::simpleTimeout(2857); 
		//serial::Timeout to = serial::Timeout::simpleTimeout(556); 
        mc_serial.setTimeout(to);
        mc_serial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(mc_serial.isOpen())
	{
		b_serialOpenOK = true;
		
        ROS_INFO_STREAM("MD BLDC Motor Controller Serial Port initialized");

		recvThread_Run();

		ROS_INFO_STREAM("Run serial receive thread...");

	}
    else
        return -1;
}

void mdrobot_motor_control::ClearRecvBuff(void)
{
	BYTE rbuff[100];
	BYTE rnum = 0;

	if(mc_serial.available() > 0)
	{
		mc_serial.read(rbuff, rnum);
		ROS_INFO_STREAM("Receive data of initial serial buffer...");
	}

	SetZeroMem(recvBuffer, MAX_RECVBUFF_SIZE);

}

int mdrobot_motor_control::SetVelocity(short lVel, short rVel)
{
	RetByte velByte;
	BYTE byPidDataSize = 13;
	BYTE byTempDataSum = 0;
    int i;

	if(lVel > MAX_SPEED)
		lVel = MAX_SPEED;
	else if(lVel < MIN_SPEED)
		lVel = MIN_SPEED;

	if(rVel > MAX_SPEED)
		rVel = MAX_SPEED;
	else if(rVel < MIN_SPEED)
		rVel = MIN_SPEED;
	
	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_PNT_VEL_CMD;
	sendBuf[4] = 7;
	sendBuf[5] = ENABLE;
	
	velByte = ShorttoByte(lVel);	
	sendBuf[6] = velByte.LowByte;
	sendBuf[7] = velByte.HighByte;
	
	sendBuf[8] = ENABLE;
	
	velByte = ShorttoByte(rVel);	
	sendBuf[9] = velByte.LowByte;
	sendBuf[10] = velByte.HighByte;
	
	//sendBuf[11] = 0;
	sendBuf[11] = REQUEST_PNT_MAIN_DATA;
	
	for(i = 0; i < (byPidDataSize-1); i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

#ifdef _DEBUG_BASIC
	ROS_INFO_STREAM("send vel : ("  << lVel << "," << rVel << ") ");

	for(i = 0; i < 12; i++)
		ROS_INFO(" %d", sendBuf[i]);
	ROS_INFO("\n");
	//ROS_INFO_STREAM(" [%d %d]  [%d %d]"  << sendBuf[6] << " " << sendBuf[7] << " " << sendBuf[9] << " " << sendBuf[10]);
#endif	

	// ROS_INFO_STREAM("send vel : ("  << lVel << "," << rVel << ") ");

	mc_serial.write(sendBuf, byPidDataSize);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::SetZeroMem(BYTE buff[], BYTE num)
{
	int i;
	for(i = 0; i < num; i++)
		buff[i] = 0;
}

BYTE mdrobot_motor_control::RecvDatatoBuff(BYTE buff[], BYTE num)
{
	std::stringstream ss;
	int i, j;
	
	if(num > 0)
	{

#ifdef DEBUG_BASIC 
		ROS_INFO("[1]receive num: %d", num);

		//ss << "num:"  << num << " recvNum:" << recvNumber;
		//ROS_INFO_STREAM(ss.str());

		//ROS_INFO("num: %d   PacketNum:%d\n", num, PacketNumber);

		ROS_INFO("[1]Receive Data ==> ");
		for(i = 0; i < num; i++)
			ROS_INFO("(%d) : %d ", i, buff[i]);
#endif

		for(i = 0; i < num; i++)		
		{
			switch(step)
			{
				case 0:
					// check reading machine id
					if((buff[i] == MDUI) && (buff[i+1] == MID_BLDC_CTR))
					{

						SetZeroMem(recvBuffer, MAX_PACKET_SIZE);

						b_PacketOK = false;
						PacketNumber = 0;
						ChkCommErr = 0;
						
						recvBuffer[PacketNumber++] = buff[i];
						CheckSum = buff[i];
						i++;
						recvBuffer[PacketNumber++] = buff[i];
						CheckSum += buff[i];

						step++;						
						
					} else {
						PacketNumber = 0;
						TotalRecvNum = 0;
						ChkCommErr++;
						
					}
					break;

				case 1:
					// check controller id		
					if((buff[i] == CTRL_ID) || (buff[i] == ID_ALL))
					{
						recvBuffer[PacketNumber++] = buff[i];
						CheckSum += buff[i];
						ChkCommErr = 0;
						
						step++;

					} else {
						step = 0;					
						PacketNumber = 0;
						ChkCommErr++;
						
					}

					break;

				case 2:
					// put the PID number		
					recvBuffer[PacketNumber++] = buff[i];
					CheckSum += buff[i];
					
					step++;

					break;

				case 3:
					// put the data num
					recvBuffer[PacketNumber++] = buff[i];
					CheckSum += buff[i];
					MaxDataNum = buff[i];
					DataNum = 0;
					step++;

					break;

				case 4:
					// put the data into the receive buffer
					recvBuffer[PacketNumber++] = buff[i];
					CheckSum += buff[i];

					if(++DataNum >= MAX_DATA_SIZE)
					{
						step = 0;
						TotalRecvNum = 0;
						break;
					}

					if(DataNum >= MaxDataNum)
					{
						step++;
					}

					break;

				case 5:
					// put the check sum after checking it
					recvBuffer[PacketNumber++] = buff[i];
					CheckSum += buff[i];

					if(CheckSum == 0)
					{
#ifdef _DEBUG_BASIC
						ROS_INFO("[6]i: %d   PacketNum:%d", i, PacketNumber);

						ROS_INFO("[6]Packet Data ==> ");
						for(j = 0; j< PacketNumber; j++)
							ROS_INFO("(%d) : %d ", j, recvBuffer[j]);
#endif							
						b_PacketOK = true;
						DataNum = 0;
						MaxDataNum = 0;
					}

					step = 0;
					TotalRecvNum = 0;

					break;

				default:
					step = 0;

					break;

			}

			if(b_PacketOK == true)
			{
				b_PacketOK = false;
				PacketNumber = 0;

				ParseReceiveData();
			}

			if(ChkCommErr == 30)
			{
				ChkCommErr = 0;
				step = 0;
				CheckSum = 0;
				MaxDataNum = 0;
				DataNum = 0;
				
				i = num;
			}

		}
		
	}	
	
	return SUCCESS;	
}

void mdrobot_motor_control::SendReqPIDData(BYTE PID)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_REQ_PID_DATA;
	sendBuf[4] = 1;
	sendBuf[5] = PID;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::PNTMainDataBC(BYTE type)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_COMMAND;
	sendBuf[4] = 1;
	if(type == ON)
		sendBuf[5] = CMD_PNT_MAIN_DATA_BC_ON;
	else 
		sendBuf[5] = CMD_PNT_MAIN_DATA_BC_OFF;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);
	memset(sendBuf,0,MAX_PACKET_SIZE);

#ifdef _DEBUG_BASIC  
	ROS_INFO("PID_MainDataBC Send Data ==> ");
	for(int i = 0; i < 7; i++)
		ROS_INFO("(%d) : %d ", i, sendBuf[i]);
#endif		
}

void mdrobot_motor_control::TorqueOff(BYTE type)
{
	BYTE byTempDataSum = 0;
	int i;
	
	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_PNT_TQ_OFF;
	sendBuf[4] = 3;
	if(type == ON)
	{
		sendBuf[5] = ENABLE;
		sendBuf[6] = ENABLE;
	}
	else 
	{
		sendBuf[5] = DISABLE;
		sendBuf[6] = DISABLE;		
	}

	sendBuf[7] = 2;
		
	for(i = 0; i < 8; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[8] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 9);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::BreakOff(BYTE type)
{
	BYTE byTempDataSum = 0;
	int i;
	
	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_PNT_BRAKE;
	sendBuf[4] = 3;
	if(type == ON)
	{
		sendBuf[5] = ENABLE;
		sendBuf[6] = ENABLE;
	}
	else 
	{
		sendBuf[5] = DISABLE;
		sendBuf[6] = DISABLE;		
	}

	sendBuf[7] = DISABLE;
		
	for(i = 0; i < 8; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[8] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 9);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::ResetAlarm(void)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_ALARM_RESET;
	sendBuf[4] = 1;
	sendBuf[5] = 0;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::ResetPosition(void)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
	sendBuf[3] = PID_COMMAND;
    //sendBuf[3] = PID_POSI_RESET;
	sendBuf[4] = 1;
	sendBuf[5] = CMD_POSI_RESET;
	//sendBuf[5] = 0;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::checkMDMCRunning(void)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_REQ_PID_DATA;
	sendBuf[4] = 1;
	sendBuf[5] = PID_VER;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);

#ifdef _DEBUG_BASIC  
	ROS_INFO("checkMDMCRunning Send Data ==> ");
	for(int i = 0; i < 7; i++)
		ROS_INFO("(%d) : %d ", i, sendBuf[i]);
#endif		
	memset(sendBuf,0,MAX_PACKET_SIZE);

}

void mdrobot_motor_control::setLimitSW(BYTE val)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_USE_LIMIT_SW;
	sendBuf[4] = 1;
	sendBuf[5] = val;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);

#ifdef _DEBUG_BASIC  
	ROS_INFO("setLimitSW Send Data ==> ");
	for(int i = 0; i < 7; i++)
		ROS_INFO("(%d) : %d ", i, sendBuf[i]);
#endif		
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::setStopStatus(BYTE val)
{
	BYTE byTempDataSum = 0;
	int i;

	for(i = 0; i <MAX_PACKET_SIZE; i++) 
		sendBuf[i] = 0;
	
	sendBuf[0] = MID_BLDC_CTR;
    sendBuf[1] = MDUI;
    sendBuf[2] = CTRL_ID;
    sendBuf[3] = PID_STOP_STATUS;
	sendBuf[4] = 1;
	sendBuf[5] = val;
		
	for(i = 0; i < 6; i++) 
		byTempDataSum += sendBuf[i];
        
	sendBuf[6] = ~(byTempDataSum) + 1; //check sum

	mc_serial.write(sendBuf, 7);

#ifdef _DEBUG_BASIC  
	ROS_INFO("Stop Status Send Data ==> ");
	for(int i = 0; i < 7; i++)
		ROS_INFO("(%d) : %d ", i, sendBuf[i]);
#endif		
	memset(sendBuf,0,MAX_PACKET_SIZE);
}

void mdrobot_motor_control::Stop(void)
{
	SetVelocity(0, 0);
}

void mdrobot_motor_control::GetMotorPosition(long *lpos, long *rpos)
{
	*lpos =  MotorPosition[MOTOR_LEFT];
	*rpos =  MotorPosition[MOTOR_RIGHT];
}

void mdrobot_motor_control::GetVelocityRMP(short *lvel, short *rvel)
{
	*lvel =  sMotorRPM[MOTOR_LEFT];
	*rvel =  sMotorRPM[MOTOR_RIGHT];
}

int mdrobot_motor_control::ParseReceiveData(void)
{
#ifdef _DEBUG_BASIC	
	ROS_INFO("recvPID:%d\n", recvBuffer[3]);
	ROS_INFO_STREAM("recv num:"  << recvNumber << " recvPID:" <<  recvBuffer[3]);
#endif

	switch(recvBuffer[3])
	{

		case PID_VER:
#ifdef _DEBUG_BASIC
			ROS_INFO("PID_VER Receive Data ==> ");
			for(int i = 0; i < 7; i++)
				ROS_INFO("(%d) : %d ", i, recvBuffer[i]);
#endif
			if(recvBuffer[5] > 0)
				b_MDMCRunOK = true;

			break;

		case PID_PNT_MONITOR:                                                              
			byStatus[MOTOR_LEFT]   = recvBuffer[5];
            fgAlarm[MOTOR_LEFT]    = recvBuffer[5] & BIT0;
            fgCtrlFail[MOTOR_LEFT] = recvBuffer[5] & BIT1;
            fgOverVolt[MOTOR_LEFT] = recvBuffer[5] & BIT2;
            fgOverTemp[MOTOR_LEFT] = recvBuffer[5] & BIT3;
            fgOverLoad[MOTOR_LEFT] = recvBuffer[5] & BIT4;
            fgHallFail[MOTOR_LEFT] = recvBuffer[5] & BIT5;
            fgInvVel[MOTOR_LEFT]   = recvBuffer[5] & BIT6;
            fgStall[MOTOR_LEFT]    = recvBuffer[5] & BIT7;
			sMotorRPM[MOTOR_LEFT]  = BytetoShort(recvBuffer[6], recvBuffer[7]);

            byStatus[MOTOR_RIGHT]   = recvBuffer[8];
            fgAlarm[MOTOR_RIGHT]    = recvBuffer[8] & BIT0;
            fgCtrlFail[MOTOR_RIGHT] = recvBuffer[8] & BIT1;
            fgOverVolt[MOTOR_RIGHT] = recvBuffer[8] & BIT2;
            fgOverTemp[MOTOR_RIGHT] = recvBuffer[8] & BIT3;
            fgOverLoad[MOTOR_RIGHT] = recvBuffer[8] & BIT4;
            fgHallFail[MOTOR_RIGHT] = recvBuffer[8] & BIT5;
            fgInvVel[MOTOR_RIGHT]   = recvBuffer[8] & BIT6;
            fgStall[MOTOR_RIGHT]    = recvBuffer[8] & BIT7;
			sMotorRPM[MOTOR_RIGHT]  = BytetoShort(recvBuffer[9], recvBuffer[10]);
            ROS_INFO("Receive Data ==> ");
			for(int i = 0; i < 12; i++)
				ROS_INFO("(%d) : %d ", i, recvBuffer[i]);
			ROS_INFO("recvData => %ld  %ld", (long)sMotorRPM[MOTOR_LEFT], (long)sMotorRPM[MOTOR_RIGHT]);
#ifdef _DEBUG_BASIC
			ROS_INFO("Receive Data ==> ");
			for(int i = 0; i < 12; i++)
				ROS_INFO("(%d) : %d ", i, recvBuffer[i]);
			ROS_INFO("recvData => %ld  %ld", (long)sMotorRPM[MOTOR_LEFT], (long)sMotorRPM[MOTOR_RIGHT]);
#endif
			b_recvOK = true;

			break;

		case PID_PNT_MAIN_DATA:
			sMotorRPM[MOTOR_LEFT]  = BytetoShort(recvBuffer[5], recvBuffer[6]);
            sCurrent[MOTOR_LEFT]   = BytetoShort(recvBuffer[7], recvBuffer[8]);
            byStatus[MOTOR_LEFT]   = recvBuffer[9];
            fgAlarm[MOTOR_LEFT]    = recvBuffer[9] & BIT0;
            fgCtrlFail[MOTOR_LEFT] = recvBuffer[9] & BIT1;
            fgOverVolt[MOTOR_LEFT] = recvBuffer[9] & BIT2;
            fgOverTemp[MOTOR_LEFT] = recvBuffer[9] & BIT3;
            fgOverLoad[MOTOR_LEFT] = recvBuffer[9] & BIT4;
            fgHallFail[MOTOR_LEFT] = recvBuffer[9] & BIT5;
            fgInvVel[MOTOR_LEFT]   = recvBuffer[9] & BIT6;
            fgStall[MOTOR_LEFT]    = recvBuffer[9] & BIT7;

            MotorPosition[MOTOR_LEFT] = BytetoLInt(recvBuffer[10], recvBuffer[11],
                recvBuffer[12], recvBuffer[13]);

            sMotorRPM[MOTOR_RIGHT]  = BytetoShort(recvBuffer[14], recvBuffer[15]);
            sCurrent[MOTOR_RIGHT]   = BytetoShort(recvBuffer[16], recvBuffer[17]);
            byStatus[MOTOR_RIGHT]   = recvBuffer[18];
            fgAlarm[MOTOR_RIGHT]    = recvBuffer[18] & BIT0;
            fgCtrlFail[MOTOR_RIGHT] = recvBuffer[18] & BIT1;
            fgOverVolt[MOTOR_RIGHT] = recvBuffer[18] & BIT2;
            fgOverTemp[MOTOR_RIGHT] = recvBuffer[18] & BIT3;
            fgOverLoad[MOTOR_RIGHT] = recvBuffer[18] & BIT4;
            fgHallFail[MOTOR_RIGHT] = recvBuffer[18] & BIT5;
            fgInvVel[MOTOR_RIGHT]   = recvBuffer[18] & BIT6;
            fgStall[MOTOR_RIGHT]    = recvBuffer[18] & BIT7;

            MotorPosition[MOTOR_RIGHT] = BytetoLInt(recvBuffer[19], recvBuffer[20],
                    recvBuffer[21], recvBuffer[22]);

			b_recvOK = true;

			// ROS_INFO("recvData => %d  %d", sCurrent[MOTOR_LEFT], sCurrent[MOTOR_RIGHT]);
            // ROS_INFO("[ %d, %ld, %ld, %d, %ld, %ld ]", byStatus[MOTOR_LEFT], (long)sMotorRPM[MOTOR_LEFT], MotorPosition[MOTOR_LEFT],
            //          byStatus[MOTOR_RIGHT], (long)sMotorRPM[MOTOR_RIGHT], MotorPosition[MOTOR_RIGHT]);

#ifdef _DEBUG_BASIC
            ROS_INFO("recvData => %d  %d", sCurrent[MOTOR_LEFT], sCurrent[MOTOR_RIGHT]);
            ROS_INFO("[ %d, %ld, %ld, %d, %ld, %ld ]", byStatus[MOTOR_LEFT], (long)sMotorRPM[MOTOR_LEFT], MotorPosition[MOTOR_LEFT],
                     byStatus[MOTOR_RIGHT], (long)sMotorRPM[MOTOR_RIGHT], MotorPosition[MOTOR_RIGHT]);
#endif

			break;
		
	}
}	
