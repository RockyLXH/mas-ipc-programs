/**
 *  Description:	Use mbus to control the motion,
 * 	HoldingRegister[0] -> =1: terminate the program.
 * 	HoldingRegister[1] -> set target velocity, unit: RPM.
 * 	HoldingRegister[2] -> velocity loop KP.
 * 	HoldingRegister[3] -> velocity loop KI.
 */

#include <fstream>
#include <chrono>
#include <syslog.h> // for system log
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "SIL_Sample.h" // Application header file.
#include "tools/tools.h"
#include "fmt/format.h"

using namespace utility;

static cleaner::Cleaner cleaner{MainClose};

Instance instance = {DoSilTest,
					 MMC_UCUSER607A_SRC,
					 OPM402_CYCLIC_SYNC_POSITION_MODE};

/*
 ============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				29/07/2022
 Modifications:			N/A

 The main function of this sample project.
 ============================================================================
 */

int main(int argc, char *argv[])
{
	try
	{
		/*
		 * open system log file to write.
		 *
		 *  LOG_CONS
		 *    Write directly to system console if there is an error while sending to system logger.
		 *
		 *  LOG_NDELAY
		 *     Open the connection immediately (normally, the connection is opened when the first message is logged).
		 *
		 *  LOG_NOWAIT
		 *     Do not wait for child processes that may have been created while logging the message.  (The GNU C library does not create a
		 *     child process, so this option has no effect on Linux.)
		 *
		 *  LOG_ODELAY
		 *     The converse of LOG_NDELAY; opening of the connection is delayed until syslog() is called.  (This is the  default,  and  need
		 *     not be specified.)
		 *
		 *  LOG_PERROR
		 *     (Not in SUSv3.) Print to stderr as well.
		 *
		 *  LOG_PID
		 *     Include PID with each message.
		 */
		openlog("SIL Program", LOG_CONS | LOG_PID, 0);

		// System log file location: /var/log/syslog
		syslog(LOG_DEBUG, "the program <%s> is working\n", argv[0]);

		// Initialize system, axes and all needed initializations
		MainInit();

		SILInit();

		MainLoop();

		// MainClose();

		return 0;
	}
	catch (CMMCException &e)
	{
		// fprintf(stderr,
		// 		"Exception in function %s, axis ref=%s, err=%d, status=%d, bye\n",
		// 		e.what(), e.axisName(), e.error(), e.status());
		fmt::println(stderr, "Exception in function {}, axis ref={}, err={}, status={}, bye",
					 e.what(), e.axisName(), e.error(), e.status());
		// MainClose();
		exit(0);
	}
	catch (...)
	{
		// std::cerr << "Unknown exception caught\n";
		fmt::println(stderr, "Unknown excpetion caught");
		// MainClose();
		exit(0);
	}
}

/**
 * HoldingRegister[0] -> terminate the programm.
 * HoldingRegister[1] -> set target velocity, unit: rpm.
 * HoldingRegister[2] -> velocity loop kp.
 * HoldingRegister[3] -> velocity loop ki.
 */
void ReadMbusInput(void)
{
	mbus.MbusReadHoldingRegisterTable(0, 4, mbus_read_out);

	is_terminate = mbus_read_out.regArr[0];
	target_vel = static_cast<double>(mbus_read_out.regArr[1]);
	vel_kp = static_cast<double>(mbus_read_out.regArr[2] / 10000.0);
	vel_ki = static_cast<double>(mbus_read_out.regArr[3] / 1000.0);

	return;
}

void UpdatePID(void)
{
	mbus_write_in.startRef = 2;
	mbus_write_in.refCnt = 2;
	mbus_write_in.regArr[0] = static_cast<short>(vel_kp * 10000.0);
	mbus_write_in.regArr[1] = static_cast<short>(vel_ki * 1000.0);

	mbus.MbusWriteHoldingRegisterTable(mbus_write_in);

	return;
}

void MainLoop(void)
{

	// struct sigaction SigAction;

	// SigAction.sa_handler = TerminateApplication;

	// sigaction(SIGINT, &SigAction, NULL);

	while (!is_terminate)
	{

		ReadMbusInput();

		if (is_test_done)
		{
			std::cout << "SIL Test is done" << std::endl;
			break;
		}

		if ((pid_vel.ki() != vel_ki) || (pid_vel.kp() != vel_kp))
		{
			pid_vel.set_ki(vel_ki);
			pid_vel.set_kp(vel_kp);
		}

		UpdatePID();
		usleep(500000);
	}

	return;
}

void SILInit(void)
{
	auto [func, src, mode] = instance;

	MMC_DestroySYNCTimer(conn_handler);

	for (int i = 0; i < MAX_AXES; ++i)
	{
		axis_rt[i].SetBoolParameter(0 /* 0 - NC profiler, 2 - User */, src, 0);
		axis_rt[i].SetOpMode(mode);
		while (axis_rt[i].GetOpMode() != mode)
			;
		axis_rt[i].SetBoolParameter(2, src, 0);
		axis_rt[i].SetUser6071(0.0f);

		usleep(1000); // wait some time for ensure the mode changes done.

		axis_rt[i].PowerOn();

		while (!(axis_rt[i].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
			usleep(10000);

		pos[i] = axis_rt[i].GetActualPosition();
	}

	// set sync timer for running the callback function
	MMC_CreateSYNCTimer(conn_handler, func, 1 /* sync timer 1X */);

	// set user call-back function to highest priority -> 1.
	// it must be set after CreateSyncTimer func, not before.
	MMC_SetRTUserCallback(conn_handler, 1);

	return;
}
/*
 ============================================================================
 Function:				MainInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the system, including axes, communication, etc.
 ============================================================================
 */
void MainInit(void)
{
	// ensure there is at least 1 axes to be configured.
	static_assert(MAX_AXES >= 1, "configure at least 1 axis by setting 'MAX_AXES' macro!");

	// InitializeCommunication to the GMAS:
	conn_handler = conn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK)CallbackFunc);

	mbus.MbusStartServer(conn_handler, 1);

	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true, false);

	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);

	// Register the callback function for Emergency:
	conn.RegisterEventCallback(MMCPP_EMCY, (void *)Emergency_Received);

	// Initialize default parameters. This is not a must. Each parameter may be initialized individually.
	axis_default_para.fEndVelocity = 0;
	axis_default_para.dbDistance = 100000;
	axis_default_para.dbPosition = 0;
	axis_default_para.fVelocity = 100000;
	axis_default_para.fAcceleration = 1000000;
	axis_default_para.fDeceleration = 1000000;
	axis_default_para.fJerk = 20000000;
	axis_default_para.eDirection = MC_POSITIVE_DIRECTION;
	axis_default_para.eBufferMode = MC_BUFFERED_MODE;
	axis_default_para.ucExecute = 1;
	//
	// TODO: Update number of necessary axes:
	//

	char sAxisName[20];

	for (int i = 0; i < MAX_AXES; ++i)
	{
		// sprintf(sAxisName, "a%02d", i + 1);
		fmt::format_to(sAxisName, "{:02d}", i + 1);
		axis_rt[i].InitAxisData(sAxisName, conn_handler);
		axis_rt[i].SetDefaultParams(axis_default_para);

		if (axis_rt[i].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
			axis_rt[i].Reset();
	}

	return;
}
/*
 ============================================================================
 Function:				MainClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed before the application progra, is
 terminated.
 ============================================================================
 */
void MainClose(void)
{
	//
	//	Here will come code for all closing processes
	//

	fmt::println(stdout, "the program is closing ...");
	MMC_DestroySYNCTimer(conn_handler);

	for (int i = 0; i < MAX_AXES; ++i)
	{
		axis_rt[i].SetUser6071(0.0f);
		axis_rt[i].SetBoolParameter(0, MMC_UCUSER6071_SRC, 0);

		axis_rt[i].SetUser607A(0.0f);
		axis_rt[i].SetBoolParameter(0, MMC_UCUSER607A_SRC, 0);
		axis_rt[i].SetUser60FF(0.0f);
		axis_rt[i].SetBoolParameter(0, MMC_UCUSER60FF_SRC, 0);

		axis_rt[i].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		while (axis_rt[i].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE)
			;

		axis_rt[i].PowerOff();

		while (!(axis_rt[i].ReadStatus() & NC_AXIS_DISABLED_MASK))
			usleep(10000);
	}

	if (is_test_done)
	{
		ofstream file;
		file.open("record.txt", ios::out);
		if (!file.is_open())
			std::cerr << "can not open the file\n";

		for (int i = 0; i < TEST_COUNT; ++i)
		{
			file << arr_rec[i] << endl;
		}
		file.flush();
		file.close();
		std::cout << "record finished\n";
	}

	mbus.MbusStopServer();
	MMC_CloseConnection(conn_handler);

	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interupt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CallbackFunc(unsigned char *recvBuffer, short recvBufferSize, void *lpsock)
{
	// Which function ID was received ...
	switch (recvBuffer[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n");
		break;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n");
		break;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n");
		break;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n");
		break;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n");
		break;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n");
		break;
		/* This is commented as a specific event was written for this function. Once it occurs
		 * the mbusWrite_Received will be called
		 case mbus_WRITE_EVT:
		 // TODO Update additional data to be read such as function parameters.
		 // TODO Remove return 0 if you want to handle as part of callback.
		 return 0;
		 printf("mbus Write Event received - Updating Outputs\r\n") ;

		 break ;
		 */
	}

	return 1;
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case a Run Time Error was received.
//	Input			:	const char - *msg, unsigned int - uiConnHndl, unsigned short - usAxisRef, short - sErrorID, unsigned short - usStatus.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
				   unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf(
		"MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n",
		msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop mbus, engines, and power off engines.
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
// void TerminateApplication(int iSigNum)
// {
// 	//
// 	printf("\nTerminating Application ...\n");
// 	giTerminate = true;
// 	sigignore(SIGALRM);
// 	//
// 	switch (iSigNum)
// 	{
// 	// Handle ctrl+c.
// 	case SIGINT:
// 		// TODO Close what needs to be closed before program termination.
// 		break;
// 	default:
// 		break;
// 	}
// 	return;
// }

///////////////////////////////////////////////////////////////////////
//	Function name	:	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case an Emergency event was received.
//	Input			:	unsigned short - usAxisRef, short - sEmcyCode.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef,
		   sEmcyCode);
}

int DoSinGenForPosLoop(void)
{
	double rtb_SineWave;
	static double SineWave_AccFreqNorm = 0.0;
	double SineWave_Frequency = 1.0;
	int rtb_DataTypeConversion;

	// S-Function (sdspsine2): '<Root>/Sine Wave'
	rtb_SineWave = 5000.0 * sin(SineWave_AccFreqNorm);

	// Update accumulated normalized freq value
	// for next sample.  Keep in range [0 2*pi)
	SineWave_AccFreqNorm += SineWave_Frequency * 0.0062831853071795866;
	if (SineWave_AccFreqNorm >= 6.2831853071795862)
	{
		SineWave_AccFreqNorm -= 6.2831853071795862;
	}
	else
	{
		if (SineWave_AccFreqNorm < 0.0)
		{
			SineWave_AccFreqNorm += 6.2831853071795862;
		}
	}

	// End of S-Function (sdspsine2): '<Root>/Sine Wave'

	// DataTypeConversion: '<Root>/Data Type Conversion'
	rtb_SineWave = floor(rtb_SineWave);
	//	  if (rtIsNaN(rtb_SineWave) || rtIsInf(rtb_SineWave)) {
	//	    rtb_SineWave = 0.0;
	//	  } else {
	//	    rtb_SineWave = fmod(rtb_SineWave, 4.294967296E+9);
	//	  }

	// DataTypeConversion: '<Root>/Data Type Conversion'
	rtb_DataTypeConversion =
		rtb_SineWave < 0.0 ? -static_cast<int>(static_cast<unsigned int>(-rtb_SineWave)) : static_cast<int>(static_cast<unsigned int>(rtb_SineWave));

	// S-Function (E607AWrite): '<Root>/E607AWrite'
	//	  Elmo_Write_607A(SineForPos_P.E607AWrite_p1, rtb_DataTypeConversion);

	axis_rt[0].SetUser607A(rtb_DataTypeConversion);

	return 0;
}

int DoSilTest(void)
{
	static bool b = false;
	static int cnt = 0;

	if (cnt < TEST_COUNT)
	{
		if (b)
		{
			//		axis_rt[0].EthercatWritePIVar(4, 0);
			axis_rt[0].SetUser607A(0);
			b = false;
			arr_rec[cnt++] = axis_rt[0].GetUser607A();
		}
		else
		{
			//		axis_rt[0].EthercatWritePIVar(4, 65536);
			axis_rt[0].SetUser607A(10000);
			b = true;
			arr_rec[cnt++] = axis_rt[0].GetUser607A();
		}
	}
	else
	{
		is_test_done = true;
	}

	return 0;
}

int DoAnalogCmdForVelLoop(void)
{
	short value = 0;
	axis_rt[2].EthercatReadPIVar(6, 0, value);

	if ((value >= -1000) && (value <= 1000))
		value = 0;

	double kp = 50000.0;
	double output = kp * value / 1000.0;

	axis_rt[0].SetUser60FF(output);
	axis_rt[1].SetUser60FF(output);
	//	axis_rt[2].SetUser60FF(dOutput1);

	return 0;
}

int DoVelLoopPidCtrl(void)
{
	/*
	 *  KP = 0.08, KI = 1, TS = 1ms for velocity close loop gains in rad/s units
	 *  KP = 0.01, KI = 1, TS = 1ms for velocity close loop gains in rpm units
	 */

	double actual_vel = axis_rt[0].GetActualVelocity(); // * 60 / 10000.0f;
	double targetCurrent = pid_vel(target_vel - actual_vel);
	axis_rt[0].SetUser6071(targetCurrent);

	return 0;
}

int DoRatchetEffect(void)
{
	static double distance = 10000 / 10; // A/B: A -> resolution of feedback, B -> equal parts
	static PIDController pidPos{0.0008, 0.0, 0.0, 10000.0, 2.0, 0.00025};
	double actPos = axis_rt[0].GetActualPosition();
	double targetPos = round(actPos / distance) * distance;
	double targetCurrent = pidPos(targetPos - actPos);

	axis_rt[0].SetUser6071(targetCurrent);

	return 0;
}

int DoEdgeEffect(void)
{
	double actual_pos = axis_rt[0].GetActualPosition();
	static PIDController pos_pid{0.01, 0.0, 0.0, 10000.0, 2.0, 0.00025};
	double L = pos[0] - 2500, R = pos[0] + 2500;

	//	if (act_pos < R && act_pos > L)
	//		axis_rt[0].SetUser6071(0);
	//	else
	//	{
	//		if (act_pos < L)
	//			axis_rt[0].SetUser6071(pid_pos(L - act_pos));
	//		if (act_pos > R)
	//			axis_rt[0].SetUser6071(pid_pos(R - act_pos));
	//	}

	double output = (actual_pos < L) ? (L - actual_pos) : (actual_pos > R) ? (R - actual_pos)
																		   : 0.0;

	axis_rt[0].SetUser6071(pos_pid(output));

	return 0;
}

int DoDampEffect(void)
{
	double kp = 0.0001;

	axis_rt[0].SetUser6071(-kp * axis_rt[0].GetActualVelocity());

	return 0;
}

int DoSmoothEffect(void)
{
	static PIDController pid{0.0001, 0, 0, 1000, 0.05, 0.00025};

	axis_rt[0].SetUser6071(pid(axis_rt[0].GetActualVelocity()));

	return 0;
}
