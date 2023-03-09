
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <serial/serial.h>
#include <boost/thread/thread.hpp>

#define ID_BLDC_CTRL        1
#define ID_MDUI             2
#define ID_ALL              0xfe

#define PID_VER				1
#define PID_REQ_PID_DATA    4
#define PID_TQ_OFF 			5

#define PID_COMMAND         10
#define PID_ALARM_RESET 	12
#define PID_POSI_RESET      13
#define PID_USE_LIMIT_SW 	17
#define PID_HALL_TYPE		21
#define PID_STOP_STATUS		24
#define PID_CTRL_STATUS 	34
#define PID_CTRL_STATUS2 	39
#define PID_HALL2_TYPE		65
#define PID_HALL1_TYPE		68
#define PID_STOP_STATUS1	73
#define PID_MAX_RPM1		121
#define PID_MAX_RPM2		122
#define PID_VEL_CMD 		130
#define PID_VEL_CMD2 		131
#define PID_ID 				133

#define PID_BAUDRATE        135
#define PID_VOLT_IN         143
#define PID_SLOW_START      153
#define PID_SLOW_DOWN       154

#define PID_ENC_PPR 		156

#define PID_PV_GAIN			167
#define PID_P_GAIN			168
#define PID_I_GAIN			169
#define PID_TQ_P_GAIN		170

#define PID_PNT_TQ_OFF      174
#define PID_PNT_BRAKE       175

#define PID_MAIN_DATA 		193
#define PID_MONITOR 		196
#define PID_POSI_DATA 		197
#define PID_MAIN_DATA2 		200
#define PID_MONITOR2 		201
#define PID_GAIN 			203

#define PID_PNT_VEL_CMD     207
#define PID_PNT_MAIN_DATA   210

#define PID_PNT_MONITOR		216
#define PID_POSI_SET		217
#define PID_POSI_SET2 		218
#define PID_MAX_RPM 		221
#define PID_MIN_RPM 		223

#define PID_GAIN1			252
#define PID_GAIN2			253

#define CMD_ALARMRESET		8
#define CMD_POSI_RESET		10
#define CMD_PNT_MAIN_DATA_BC_ON		61
#define CMD_PNT_MAIN_DATA_BC_OFF	62

#define ON					1
#define OFF					0

#define ENABLE            	1
#define DISABLE           	0
#define FAIL              	0	
#define SUCCESS           	1

#define MOTOR_LEFT			0
#define MOTOR_RIGHT			1

#define MAX_PACKET_SIZE     26
#define MAX_DATA_SIZE       21
#define MAX_RECVBUFF_SIZE	250

#define MID_BLDC_CTR		183
#define MDUI				184
#define MDT					183
#define CTRL_ID				1

#define REQUEST_PNT_MONITOR_DATA 1
#define REQUEST_PNT_MAIN_DATA 	 2

#define ENABLE				1
#define DISABLE				0

typedef unsigned char 		BYTE;
typedef unsigned short 		WORD;
typedef unsigned int 		DWORD;

#define BIT0				0x01
#define BIT1				0x02
#define BIT2				0x04
#define BIT3				0x08
#define BIT4				0x10
#define BIT5				0x20
#define BIT6				0x40
#define BIT7				0x80

#define MAX_SPEED			4500
#define MIN_SPEED			-4500

typedef struct {
		BYTE LowByte;
		BYTE HighByte;
} RetByte;

class mdrobot_motor_control
{
	
public:	
	mdrobot_motor_control();
	~mdrobot_motor_control();
	
	RetByte ShorttoByte(short sVal);
	int BytetoShort(BYTE LowByte, BYTE HighByte);
	int BytetoLInt(BYTE Data1Byte, BYTE Data2Byte, BYTE Data3Byte, BYTE Data4Byte);
	int InitSerial(void);
	int SetVelocity(short lVel, short rVel);
	void recvThread_Run(void);
	int ParseReceiveData(void);
	void ClearRecvBuff(void);
	void SetZeroMem(BYTE buff[], BYTE num);
	void RecvDatafromMC(void);
	BYTE RecvDatatoBuff(BYTE buff[], BYTE num);
	void PNTMainDataBC(BYTE type);
	void GetMotorPosition(long *lpos, long *rpos);
	void GetVelocityRMP(short *lvel, short *rvel);
	void TorqueOff(BYTE type);
	void BreakOff(BYTE type);
	void Stop(void);
	void ResetAlarm(void);
	void ResetPosition(void);
	void SendReqPIDData(BYTE PID);
	BYTE isMDMCRun(void) { if(b_MDMCRunOK) return ON; else return OFF;};
	void checkMDMCRunning(void);
	void resetMDMCRunFlag(void) { b_MDMCRunOK = OFF; };
	void ReconnectSerial(void);
	
	serial::Serial mc_serial;
	
	int nBaudrate;
	BYTE sendBuf[MAX_PACKET_SIZE];
	BYTE recvBuffer[MAX_RECVBUFF_SIZE];
	BYTE rbuff[MAX_RECVBUFF_SIZE];
	BYTE PacketNumber;
	bool b_serialOpenOK;
	bool b_recvOK;
	bool b_PacketOK;
	BYTE TotalRecvNum;
	BYTE MaxDataNum;
	BYTE DataNum;
	BYTE step;
	BYTE SerialErrCnt;
	
	short sMotorRPM[2];
    long MotorPosition[2];

    WORD sCurrent[2];

    BYTE byStatus[2];
    BYTE fgAlarm[2];
	BYTE fgCtrlFail[2];
	BYTE fgOverVolt[2];
	BYTE fgOverTemp[2];
    BYTE fgOverLoad[2];
	BYTE fgHallFail[2];
	BYTE fgInvVel[2];
	BYTE fgStall[2];
	
	BYTE ChkCommErr;
	BYTE CheckSum;

	bool b_SerialThreadRun;

	bool b_MDMCRunOK;

};





