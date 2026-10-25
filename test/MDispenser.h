/*
============================================================================
 Name : 		MDispenser.h
 Author  :		Kim
 Version :		1.0
============================================================================
============================================================================
 Project general functions prototypes
============================================================================
*/
void MainInit();
void MachineSequences();
void MainClose();
void MachineSequencesInit();
void EnableMachineSequencesTimer(int TimerCycle);
void BackgroundProcesses();
void MachineSequencesClose();
void MachineSequencesTimer(int iSig);
void ReadAllInputData();
void WriteAllOutputData();
long GetModbusLong(unsigned short iOffset) ;
void InsertLongVarToModbusShortArr(short* spArr, long lVal) ;
int  OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) ;
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) ;
void ModbusWrite_Received() ;
void TerminateApplication(int iSigNum);
int  CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
void MultiAxesParameterInit();
void SingleAxisParameterInit();
void StartVisionOCInProcess(int iAxis);
void StopVisionOCInProcess(int iAxis);
void StartValveOCInProcess(int iAxis,int iGroup);
void StopValveOCInProcess(int iAxis);
void ValveOCInInit(int iAxis,int iGroup);
/*
============================================================================
 States functions
============================================================================
*/
void StateFunction_PTP(int iAxis);
void StateFunction_GroupLinearAbsolute(int iGroup);
void StateFunction_GroupCircularAbsolute(int iGroup);
void StateFunction_Stop(int iAxis);
void StateFunction_Group_Stop(int iGroup);
void StateFunction_Dispensing_Stop(int iGroup);
void StateFunction_ACKError(int iAxis);
void StateFunction_Group_ACKError(int iGroup);
void StateFunction_Home (int iAxis) ;
void StateFunction_PowerOn(int iAxis);
void StateFunction_PowerOff(int iAxis);
void StateFunction_DispensingProcess(int iGroup);
void StateFunction_VisionOC_Is_Downloading(int iAxis);
void StateFunction_VisionOC_Enable(int iAxis);
void StateFunction_VisionOC_Disable(int iAxis);
void MMCPP_InitConnection() ;
unsigned long int PmasTimeUpdate();


unsigned short Input_CalcCRC_IML(int usLength, int usOffSet);
unsigned short OutPut_CalcCRC_IML(int usLength, int usOffSet);
unsigned short CalcCRC_IML(void);
unsigned short COMM_CrcValueCalc(const char* data, int length);

#define 	SINGLE_VALVE
// #define	DOUBLE_VALVE
//#define	SINGLE_VALVE_R
//#define	SINGLE_VALVE_3D
//#define	SINGLE_VALVE_3DR
//#define	SINGLE_VALVE_3D2R

/*
============================================================================
 General constants
============================================================================
*/

#ifdef	SINGLE_VALVE
#define 	MAX_AXES				3		// number of Physical axes in the system.
#define		MAX_VECTORS				1
#endif

#ifdef	SINGLE_VALVE_3D
#define 	MAX_AXES				3		// number of Physical axes in the system.
#define		MAX_VECTORS				1
#endif

#ifdef	SINGLE_VALVE_3DR
#define 	MAX_AXES				4		// number of Physical axes in the system.
#define		MAX_VECTORS				1
#endif

#ifdef	SINGLE_VALVE_3D2R
#define 	MAX_AXES				5		// number of Physical axes in the system.
#define		MAX_VECTORS				1
#endif


#ifdef	SINGLE_VALVE_R
#define 	MAX_AXES				5		// number of Physical axes in the system.
#define		MAX_VECTORS				1
#endif

#ifdef	DOUBLE_VALVE
#define 	MAX_AXES				6		// number of Physical axes in the system.
#define		MAX_VECTORS				3
#endif

#define 	FALSE					0
#define 	TRUE					1
#define 	SYNC_MULTIPLIER			1
#define 	MAX_CODE_NUM			100000
#define		MAX_OC_POINTS			50
//
// Modbus memory map offsets must be updated accordingly.

#define		MODBUS_W_COMMAND_FRAMAE					0;
#define 	MODBUS_W_COMMAND_SINGLE_PARAMETER		10;
#define 	MODBUS_W_COMMAND_GROUP_PARAMETER		50;
#define		MODBUS_W_COMMAND_OC_DATA				10;
#define		MODBUS_W_VALVE_PARAMETER_ACK			50;
#define		MODBUS_W_COMMAND_X_DIGITAL_OUTPUT		98;
#define 	MODBUS_W_COMMAND_CRC					99;
#define		MODBUS_R_VALVE_PARAMETER_EVENT			100;
#define		MODBUS_R_PROCESS_CODE					103;
#define 	MODBUS_R_UNIT_CODE						104;
#define 	MODBUS_R_LINE_CODE						105;
#define		MODBUS_R_AXIS_STATUS					110;
#define		MODBUS_R_AXIS_POSITION					120;
#define 	MODBUS_R_ERROR_AXISCODE					140;
#define 	MODBUS_R_ERROR_GMASCODE					160;
#define		MODBUS_R_DISPENSING_COUNTER				167;
#define 	MODBUS_R_GROUP_STATUS					170;
#define 	MODBUS_R_ERROR_GROUPCODE				180;
#define 	MODBUS_R_DIGITAL_IN						190;

/*
============================================================================
 Project constants
============================================================================
*/
#define		SLEEP_TIME				1000	// Sleep time of the backround idle loop, in micro seconds
#define		TIMER_CYCLE				20		// Cycle time of the main sequences timer, in ms
/*
============================================================================
 States Machines constants
============================================================================
*/
enum eMainStateMachines
{
	eIDLE	= 	0		,
	ePTP				,			// Main state machine #1 - PTP
	eGroupLinear		,			// Main state machine #2 - Group Linear
	eGroupCircular		,			// Main state machine #3 - Group Circular
	eHome				,			// Main state machine #4 - Home
	eStop				,			// Main state machine #5 - Stop
	ePowerOn			,			// Main state machine #6 - Power On
	ePowerOff			,			// Main state machine #7 - Power Off
	eAckError			,			// Main state machine #8 - ACK Error
	eDispensingStart	,			// Main state machina #9 - Dispensing Start
	eDispensingStop		,			// Main state machine #10- Dispensing Stop
	eOCStart			,			// Main state machine #12- OC Start
	eOCStop			,				// Main state machine #13- OC Stop
};

enum eErrorStateMachine
{
	eGmasNoError = 0,				// No Error
	eGMASErr,						// GMAS Error
	eEMGCYErr,						// Emergency Received. Need to handle.
};
//
enum eSubStateMachine_ErrorACK
{
	eSubState_SMReset  =1 	 ,		// Reset PLCopen
	eSubState_SMReset_WAIT	 ,		// Wait till no Error mask.
	eSubState_SMReset_DONE	 ,		// Reset Vars
};

enum eSubStateMachine_GrouptErrorACK
{
	eSubState_Group_SMReset  =1 	 ,		// Reset PLCopen
	eSubState_Group_SMReset_WAIT	 ,		// Wait till no Error mask.
	eSubState_Group_SMReset_DONE	 ,		// Reset Vars
};


//
enum eSubStateMachine_PTP
{
	eSubState_PTP_ChOPMode =1   ,		// Change Operation mode to PP
	eSubState_PTP_WChOPMode	 	,		// Wait till Operation mode == PP
	eSubState_PTP_PowerOn 	 	,		// Power on
	eSubState_PTP_WPowerOn 		,		// Wait until Power on
	eSubState_PTP_Move 	 		,		// Perform first motion
	eSubState_PTP_WMove 	 	,		// Wait until wnd of motion
};
//
enum eSubStateMachine_Home
{
	eSubState_Direct_Home_PowerOn = 1	,
	eSubState_Direct_Home_WPowerOn 		,
	eSubState_Direct_Home_ChOPModeHome 	,	// Change Operation mode to Homing
	eSubState_Direct_Home_WChOPModeHome	,	// Wait till Operation mode == Homing
	eSubState_Direct_Home_DOHOME 	 	,	// Perform Homing
	eSubState_Direct_Home_WDOHOME 	 	,	// Wait until Homing ended
	eSubState_Direct_Home_WCHOPTOPP  	,	// Change OP mode to PP
	eSubState_GO_Home_ChOPMode			,
	eSubState_GO_Home_WChOPMode			,
	eSubState_GO_Home_Move				,	// Perform AbsMove
	eSubState_GO_Home_WMove				,   // Wait until end of the motion
	eSubState_GO_Home_ChCSPMode			,
	eSubState_GO_Home_WChCSPMode		,
};
//
enum eSubStateMachine_Stop
{
	eSubState_Stop_Stop  = 1	 ,		// Stop
	eSubState_Stop_WStop	 	 ,		// Wait until all came to a stop
};
enum eSubStateMachine_GroupStop
{
	eSubState_GroupStop_Stop  = 1	 ,		// Stop
	eSubState_GroupStop_WStop	 	 ,		// Wait until all came to a stop
};
enum eSubStateMachine_DispensingStop
{
	eSubState_DispensingStop_Stop  = 1	 ,		// Stop
	eSubState_DispensingStop_WStop	 	 ,		// Wait until all came to a stop
};
enum eSubStateMachine_GroupLinear
{
	eSubState_GroupLinear_Check	=1		,
	eSubState_GroupLinear_Init			,
	eSubState_GroupLinear_Run			,
	eSubState_GroupLinear_WRun			,
};

enum eSubStateMachine_GroupCircular
{
	eSubState_GroupCircular_Check	=1		,
	eSubState_GroupCircular_Init			,
	eSubState_GroupCircular_Run				,
	eSubState_GroupCircular_WRun			,
};

enum eSubStateMachine_DispensingProcess
{
	eSubState_Dispensing_FileOpen	=1		,
	eSubState_Dispensing_FileAccess			,
	eSubState_Dispensing_Init				,
	eSubState_Dispensing_WInit				,
	eSubState_Dispensing_Start				,
	eSubState_Dispensing_QueueWait			,
	eSubState_Dispensing_ZMove				,
	eSubState_Dispensing_ZWait				,
	eSubState_Dispensing_StartVisionOC		,
	eSubState_Dispensing_WStartVisionOC		,
	eSubState_Dispensing_StopVisionOC		,
	eSubState_Dispensing_WStopVisionOC		,
	eSubState_Dispensing_StartValveOC		,
	eSubState_Dispensing_WStartValveOC		,
	eSubState_Dispensing_StopValveOC		,
	eSubState_Dispensing_WStopValveOC		,
	eSubState_Dispensing_Z1Move				,
	eSubState_Dispensing_Z1Wait				,
	eSubState_Dispensing_ValveChg			,
	eSubState_Dispensing_ValveAck			,
	eSubState_Dispensing_N1Move				,
	eSubState_Dispensing_N1Wait				,
	eSubState_Dispensing_N2Move				,
	eSubState_Dispensing_N2Wait				,
	eSubState_Dispensing_UMove				,
	eSubState_Dispensing_UWait				,
	eSubState_Dispensing_FunDelay			,
	eSubState_Dispensing_WFunDelay			,
	eSubState_Dispensing_WVPause			,
	eSubState_Dispensing_WZPause			,
	eSubState_Dispensing_ZToSafe			,
	eSubState_Dispensing_WZToSafe			,
	eSubState_Dispensing_PauseIsDone		,
	eSubState_Dispensing_ZBackToSafe		,
	eSubState_Dispensing_WZBackToSafe		,
	eSubState_Dispensing_VBackToPause		,
	eSubState_Dispensing_WVBackToPause		,
	eSubState_Dispensing_ZBackToPause		,
	eSubState_Dispensing_WZBackToPause		,
	eSubState_Dispensing_Restart			,
	eSubState_Dispensing_OCInError			,
	eSubState_Dispensing_DIWait				,
	eSubState_Dispensing_End				,

};

enum eSubStateMachine_PowerOn
{
	eSubState_PowerOn = 1 	,
	eSubState_WPowerOn 	,
};

enum eSubStateMachine_PowerOff
{
	eSubState_PowerOff = 1 	,
	eSubState_WPowerOff 	,
};

enum eAxisType1
{
	eX	= 0 ,
	eY		,
	eZ		,
};
enum eAxisType2
{
	eX1	= 3 ,
	eY1		,
	eZ1		,
};
enum eAxisType3
{
	eN1	= 3	,
	eN2		,
};

enum eAxisType4
{
	eU	= 3	,
};

enum eGroupType
{
	eV0 = 0 ,
	eV1	 	,
	eV2		,
};

enum eCommandTest
{
	CmdTest	= 9 ,

};

/*
============================================================================
 Application global variables
============================================================================
*/
int 	giTerminate;		// Flag to request program termination
int		giReentrance;		// Used to detect reentrancy to the main timer function
int 	giModbusDataTrue;
int		giModbusCmdReset;
short	gsInitFinished;
short	gsHomeReady[MAX_AXES];
short	gsHomeMoveFlag[MAX_AXES];
short   gsCircleMode;

// 		Data read from the GMAS core regarding the drives
int 	giStatus[MAX_AXES] ;// Holds PLCopen statuses for all axes
int		giGroupStatus[MAX_VECTORS];
short 	sSingleStatus[MAX_AXES] ;
short	sGroupStatus[MAX_VECTORS];
int 	giPos[MAX_AXES] ;	// Holds position for all axes.
int		giTotal[MAX_VECTORS];
int 	iIndex;
int 	iPointer;
int 	iCounter;
short	gsDispensingFinished[MAX_VECTORS];
short	gsDispensingInProcess[MAX_VECTORS];
short 	gsGroupInPosition[MAX_VECTORS];
short 	gsGroupInMotion[MAX_VECTORS];
short	gsGroupInValveChg[MAX_VECTORS];
short	gsOCIsInProcess;

short	sStartVisionOCStep[MAX_AXES];
short	sStopVisionOCStep[MAX_AXES];

short	sStartValveOCStep[MAX_AXES];
short	sStopValveOCStep[MAX_AXES];
short	sValveOCInitStep[MAX_AXES];

short	sDispensingCounter[MAX_VECTORS];

unsigned short int usiWriteValue[MAX_AXES];
unsigned short int usiReadValue[MAX_AXES];

int		iDIValue[MAX_AXES];
unsigned long int		iDIStartTime[MAX_VECTORS];
unsigned long int		iDIEndTime[MAX_VECTORS];

unsigned long int		iHomeWStartTime[MAX_AXES];
unsigned long int		iHomeWEndTime[MAX_AXES];

int iCommandCode;
int iLineCode;
int iProcessCode;
int iUnitCode;

int giGlobalPause;
int giFirstPush;


//int 4Byte short 2Byte long 8Byte

typedef struct _gsSingleAxisCommands
{
	long 	lCommand ;
	long	lOldCommand;
	long 	lPosition ;
	long	lMaxPosition ;
	long 	lVelocity ;
	long	lMaxVelocity ;
	long 	lAcceleration ;
	long 	lDeceleration ;
	long 	lJerk ;
	float	fOverride;
	long 	lHomePosition ;
	long	lHomeVelocity;
	short 	sHomeMode ;
	short	sState ;
	short 	sSubState ;
	short	sHomeState ;
	short   sMotionState ;
	short	sOCState ;
	short 	sLastGMASError ;
	short	sDIErrState;
	long	lLastDriveMotorFault ;
	char	cInErrorDriveOrGmas ;
	MC_BUFFERED_MODE_ENUM eBufferMode;
	MC_DIRECTION_ENUM eDirection;

}gsSingleAxisCommands;
typedef struct _gsGroupParameters
{
	long 	lCommand ;
	long	lOldCommand ;
	long 	lAuxPosition[16] ;
	long 	lEndPosition[16] ;
	long 	lVelocity ;
	long	lMaxVelocity;
	long 	lAcceleration ;
	long 	lDeceleration ;
	long 	lJerk ;
	float	fOverride;
	short	sState ;
	short 	sSubState ;
	short 	sLastGMASError ;
	char	cGmasInErr ;
	short 	sValveChgCode ;
	short	sValveAckCode ;
	MC_COORD_SYSTEM_ENUM eCoordSystem;
	NC_TRANSITION_MODE_ENUM eTranstion;
	MC_BUFFERED_MODE_ENUM eBufferMode;
}gsGroupParameters;
typedef struct _gsCommandFrame
{
	long	lCommand;
	long	lAcceleration;
	long	lJerk;
	short	sAxisIndex;
	short	sGroupIndex;
	short	sOverride;
	short 	sHomeMode ;
}gsCommandFrame;

typedef struct _gsCodeFrame
{

	int Index;				//�������
	int FunCode;			//�����룬00: �㶨λ����ʱ�㽺��01:ֱ�߲岹�������㽺��02:Բ���岹�������㽺��03:Z�ᶨλ��04:��������05:�ȴ���ʱ
							//06:����������07:����ֹͣ��08:���������ģ�09:Z1�ᶨλ��10: N1�ᶨλ��11: N2�ᶨλ
	int Velocity;			//�˶��ٶ�
	int Acceleration;		//���ٶ�
	int Deceleration;		//���ٶ�
	int Jerk;				//�Ӽ��ٶ�
	int XPosition;			//X����
	int YPosition;			//Y����
	int ZPosition;			//Z����
	int XAUXPosition;		//X���꣬Բ���岹:X�м������
	int	YAUXPosition;		//Y���꣬Բ���岹:Y�м������
	int	ZAUXPosition;		//Z���꣬Բ���岹:Z�м������
	int X1Position;			//X1����
	int Y1Position;			//Y1����
	int Z1Position;			//Z1����
	int N1Position;			//N1����
	int N2Position;			//N2����
	int UPosition;			//U����
	int Para_1;				//Բ���岹���Ƕȣ��㶨λ��ʱ��,�Ƚ��������ʼλ��
	int	Para_2;				//�㽺���1�����λ���ƣ� ��չ��ܿ��أ� 1:��;0:��;,�Ƚ�������������
	int	Para_3;				//�ζι���ģʽ��0��BufferMode;100:Blending Mode,�Ƚ�������������
	int Para_4;				//5�����߹���ģʽ������Բ���뾶
	int Para_5;				//�㽺���2
	int Points;				//�Ƚ����λ�õ���
	int PulseDuration;		//�Ƚ�����������
	int Direction;			//�Ƚ�����˶�����
	int OCPosition[MAX_OC_POINTS];//OCλ������
	int	StartPosition;		//�Ƚ������ʼλ��
	int	DeltaPosition;		//�Ƚ�������
	int ProcessID;			//���պ�
	int UnitID;				//��Ԫ��

}gsCodeFrame;

typedef struct _gsOCPara
{
	int Points;
	int	PulseDuration;
	int VisionPulseWidth;
	int	ValvePulseWidth;
	int	Direction;
	int	Position[MAX_OC_POINTS];
	int	StartPosition;
	int	DeltaPosition;
	int ValveOCInitFlag;
}gsOCPara;

typedef struct _gsPausePara
{
	int XPosition;			//X����
	int YPosition;			//Y����
	int ZPosition;			//Z����
	int X1Position;			//X1����
	int Y1Position;			//Y1����
	int Z1Position;			//Z1����
	int N1Position;			//N1����
	int N2Position;			//N2����
	int UPosition;			//U����
}gsPausePara;

// Modbus Command masks:
#define MOTOR_ENABLE				0x1		// Cmd 1
#define MOTOR_DISABLE				0x2		// Cmd 2
#define MOTION_S_HOME_START			0x4		// Cmd 4
#define MOTION_S_JOG				0x8		// Cmd 8
#define	MOTION_S_START				0x10	// Cmd 16
#define	MOTION_S_STOP				0x20    // Cmd 32
#define MOTION_V_LINEAR_START		0x40	// Cmd 64
#define	MOTION_V_CIRCULAR_START		0x80	// Cmd 128
#define MOTION_V_DISPENSING_START	0x100	// Cmd 256
#define MOTION_V_STOP				0x200	// Cmd 512
#define MOTION_V_DISPENSING_STOP	0x400	// Cmd 1024
#define MOTION_X_DIGITAL_OUTPUT		0x800	// Cmd 2048
#define MOTION_PARAMETER_SETTING	0x1000	// Cmd 4096
#define MOTION_JOG_MODE				0x4000	// Cmd 16384
#define ERROR_ACK					0x8000	// Cmd 32768

#define MOTION_OC_START				0x20000 // OC Start
#define MOTION_OC_STOP				0x40000 // OC Stop
#define MOTION_VALVE_ACK			0x80000	// Valve ack
#define MOTION_V_DISPENSING_PAUSE	0x100000// Dispensing paused
#define MOTION_V_DISPENSING_RESTART	0x200000// Dispensing restart
#define MOTION_SYSTEM_RESTART		0x400000// Reset System

//
// Modbus Status mask:
#define NOTHING				0x0
#define MOTOR_ENABLED		0x1
#define MOTOR_INHOMING		0x2
#define MOTOR_HOMEDONE		0x4
#define MOTOR_INMOTION		0x8
#define MOTOR_INPOSITION	0x10
#define MOTOR_INERROR		0x80
#define MOTOR_OC_DATA_DOWNLOADING	0x100
#define MOTOR_OC_DATA_DOWNLOADED	0x200
#define MOTOR_OC_IS_DISABLED		0x400
#define MOTOR_OC_IS_ENABLED			0x800
#define MOTOR_OC_ERROR				0x1000
#define MOTOR_DI_ERROR				0x4000

#define OC_IN_ERROR					0x1234

// Group status mask
#define GROUP_ENABLED				0x1
#define GROUP_INMOTION				0x8
#define GROUP_INPOSITION			0x10
#define GROUP_DISPENSING_IN_PROCESS	0x20
#define	GROUP_DISPENSING_FINISHED	0x40
#define GROUP_INERROR				0x80
#define GROUP_IN_VALVE_CHG_EVENT	0x100;

gsSingleAxisCommands 	sSingleCommands[MAX_AXES];
gsGroupParameters		sGroup[MAX_VECTORS];
gsCommandFrame			gsCmdFrame;
CMMCPIVar cPIReadVar[MAX_AXES];
CMMCPIVar cPIDIReadVar[MAX_AXES];
CMMCPIVar cPIWriteVar[MAX_AXES];
gsOCPara	gsOC;
gsPausePara	gsPause;
unsigned char ucOCIndex;
long lSDOValue;
int iTable[100];
char buff[250];
char *p[MAX_VECTORS];
char *lp[MAX_VECTORS];
fstream fp[MAX_VECTORS];
/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL 	gConnHndl ;						// Connection Handle
CMMCConnection 		cConn ;							// Connection class
CMMCSingleAxis 		cAxes[MAX_AXES] ;				// Single axis class
CMMCGroupAxis		cVector[MAX_VECTORS];
CMMCHostComm		cHost ;							// Modbus host class
gsCodeFrame			gsCode[MAX_CODE_NUM];

//
MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT 	mbus_read_out;
MMC_MODBUSREADHOLDINGREGISTERSTABLE_IN 		mbus_read_in;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN 	mbus_write_in;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT 	mbus_write_out;
MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data
MMC_MOTIONPARAMS_GROUP		stMultiDefault;		// Vector default data
MMC_SETKINTRANSFORM_IN 		stSetKinTransform[MAX_VECTORS];
MMC_HOMEDS402_IN			stDS402Home ;		// standard DS402 input structure

int rc[MAX_VECTORS];
MMC_CLEARFBLIST_IN	stMMC_CLEARFBLIST_IN[MAX_VECTORS];
MMC_CLEARFBLIST_OUT	stMMC_CLEARFBLIST_OUT[MAX_VECTORS];

MMC_WRITEGROUPOFPARAMETERSMEMBER GroupOfPrmWrt[GROUP_OF_PARAMETERS_MAXIMUM_SIZE];

MMC_WAITUNTILCONDITIONFB_IN stInput;
struct timeval tv;
unsigned long int iTimeStart;
unsigned long int iTimeEnd;
unsigned long int iDTime;
unsigned long int iCycleStart;
unsigned long int iCycleEnd;

MMC_RESETSYSTEM_IN ResetIn;
MMC_RESETSYSTEM_OUT ResetOut;

int iResetrc;

