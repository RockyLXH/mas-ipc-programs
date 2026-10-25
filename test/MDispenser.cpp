/*
============================================================================
 Name : 	MDispenser.cpp
 Author :	Kim
 Version :	1.00
 Description : The following program supports the following functionalities:

	 - Modbus callback registration.
	 - Emergency callback registration.
	 - Modbus reading and updates of axis status and positions.
	 - Point to Point
	 - Homing
	 - Stop All axes

 The program works with MAX_AXES in DS402 Profile Position and DS402 Homing motion modes. These are configured automatically by
 the called functions.
 For the above functions, the following modbus 'codes' are to be sent to address 40001.

The following information is updated to the Modbus at address MODBUS_W_POSITION_INDEX:
	- All positions of axes, depending on MAX_AXES.
	- All status of axes,depending on MAX_AXES.
The following information is received from MODBUS master at address MODBUS_R_POSITION_INDEX:
	- All commands of axes, depending on MAX_AXES.
	- All parameters of axes,depending on MAX_AXES.
============================================================================
*/

#include "mmc_definitions.h"
#include "mmcpplib.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <sys/time.h> // For time structure
#include <signal.h>	  // For Timer mechanism
#include <math.h>
#include <MDispenser.h> // Application header file.
#include <time.h>
/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 The main function of this  project.
============================================================================
*/

int main()
{
	//
	//	Initialize system, axes and all needed initializations
	//
	// sleep(10);
	MainInit();
	//
	//	Execute the state machine to handle the system sequences and control
	//
	// MachineSequences();
	//
	//	Close what needs to be closed before program termination
	//
	MainClose();
	//
	return 1; // Terminate the application program back to the Operating System
}
/*
============================================================================
 Function:				MainInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Initilaize the system, including axes, communication, etc.
============================================================================
*/
void MainInit()
{
	//
	//	Here will come initialization code for all system, axes, communication etc.
	//
	// 	InitializeCommunication to the GMAS:
	//
	int i;
	char sName[20];
	long lStatus;
	char str1[10];
	char str2[10];
	char str3[10];
	//
	// Perform IPC connection and enable ALL
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK)CallbackFunc);

	CMMCPPGlobal::Instance()->SetThrowFlag(true, false);
	//	CMMCPPGlobal::Instance()->SetThrowWarningFlag(false) ;
	//
	// Start the Modbus Server on device 1.
	cHost.MbusStartServer(gConnHndl, 1);

	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);

	//
	// Register the callback function for Modbus and Emergency:
	cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE, (void *)ModbusWrite_Received);
	cConn.RegisterEventCallback(MMCPP_EMCY, (void *)Emergency_Received);
	//
	// Set default motion parameters
	stSingleDefault.fEndVelocity = 0;
	stSingleDefault.dbDistance = 0;
	stSingleDefault.dbPosition = 0;
	stSingleDefault.fVelocity = 10000;
	stSingleDefault.fAcceleration = 1E+07;
	stSingleDefault.fDeceleration = 1E+07;
	stSingleDefault.fJerk = 2E+08;
	stSingleDefault.eDirection = MC_POSITIVE_DIRECTION;
	stSingleDefault.eBufferMode = MC_BUFFERED_MODE;
	stSingleDefault.ucExecute = 1;

	// Initialize axis names and default parameters.

#ifdef SINGLE_VALVE
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
#endif

#ifdef SINGLE_VALVE_3D
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
#endif

#ifdef SINGLE_VALVE_3DR
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a04");
	cAxes[eN1].InitAxisData(sName, gConnHndl);
#endif

#ifdef SINGLE_VALVE_3D2R
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a04");
	cAxes[eN1].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a05");
	cAxes[eN2].InitAxisData(sName, gConnHndl);
#endif

#ifdef SINGLE_VALVE_R
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a04");
	cAxes[eN1].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a05");
	cAxes[eN2].InitAxisData(sName, gConnHndl);
#endif

#ifdef DOUBLE_VALVE
	sprintf(sName, "a01");
	cAxes[eX].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a02");
	cAxes[eY].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a03");
	cAxes[eZ].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a04");
	cAxes[eX1].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a05");
	cAxes[eY1].InitAxisData(sName, gConnHndl);
	sprintf(sName, "a06");
	cAxes[eZ1].InitAxisData(sName, gConnHndl);
#endif

	for (i = 0; i < MAX_AXES; i++)
	{
		sSingleCommands[i].cInErrorDriveOrGmas = eGmasNoError;
		cAxes[i].SetDefaultParams(stSingleDefault);
		lStatus = cAxes[i].ReadStatus();
		if (lStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cAxes[i].Reset();
		}
	}
	// Wait a bit and recheck if in Error state:
	sleep(1);
	for (i = 0; i < MAX_AXES; i++)
	{
		lStatus = cAxes[i].ReadStatus();
		if (lStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "One of axes is in error. Exiting.";
		}
	}

	stMultiDefault.fVelocity = 10000;
	stMultiDefault.fAcceleration = 1E+07;
	stMultiDefault.fDeceleration = 1E+07;
	stMultiDefault.fJerk = 2E+08;
	stMultiDefault.eBufferMode = MC_BUFFERED_MODE;
	stMultiDefault.eTransitionMode = MC_TM_NONE_MODE;
	memset(stMultiDefault.fTransitionParameter, 0, sizeof(double) * NC_MAX_NUM_AXES_IN_NODE);
	stMultiDefault.eCoordSystem = MC_MCS_COORD;
	stMultiDefault.ucExecute = 1;

	for (i = 0; i < MAX_VECTORS; i++)
	{
		sGroup[i].cGmasInErr = eGmasNoError;
		sprintf(sName, "v%02d", i + 1);
		cVector[i].InitAxisData(sName, gConnHndl);

		cVector[i].SetDefaultParams(stMultiDefault);
		if (cVector[i].GroupReadStatus() & NC_GROUP_ERROR_STOP_MASK)
		{
			cVector[i].GroupReset();
		}
	}
	sleep(1);
	for (i = 0; i < MAX_VECTORS; i++)
	{
		if (cVector[i].GroupReadStatus() & NC_GROUP_ERROR_STOP_MASK)
		{
			cout << "One of groups is in error. Exiting.";
		}
	}

try
{

	sprintf(str1, "I0x603F.0");
	for (i = 0; i < MAX_AXES; i++)
	{
		try
		{
			cPIReadVar[i].Init(gConnHndl, cAxes[i].GetRef(), str1);
		}
		catch (CMMCException &ex)
		{
			// printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[i].GetRef(), ex.what(), (short)ex.error(), ex.status());
			std::cout << "in catch" << std::endl;
			throw ;
		}
	}
}
catch (CMMCException &ex)
{
	std::cerr << "in outter catch\n";
}

	sprintf(str1, "I0x683F.0");
	for (i = 0; i < MAX_AXES; i++)
	{
		try
		{
			cPIReadVar[i].Init(gConnHndl, cAxes[i].GetRef(), str1);
		}
		catch (CMMCException &ex)
		{
			printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[i].GetRef(), ex.what(), (short)ex.error(), ex.status());
		}
	}

	sprintf(str2, "O0x6040.0");
	for (i = 0; i < MAX_AXES; i++)
	{
		try
		{
			cPIWriteVar[i].Init(gConnHndl, cAxes[i].GetRef(), str2);
		}
		catch (CMMCException &ex)
		{
			printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[i].GetRef(), ex.what(), (short)ex.error(), ex.status());
		}
	}

	sprintf(str3, "I0x60FD.0");
	for (i = 0; i < MAX_AXES; i++)
	{
		try
		{
			cPIDIReadVar[i].Init(gConnHndl, cAxes[i].GetRef(), str3);
		}
		catch (CMMCException &ex)
		{
			printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[i].GetRef(), ex.what(), (short)ex.error(), ex.status());
		}
	}

	MultiAxesParameterInit();
	SingleAxisParameterInit();

	memset(mbus_write_in.regArr, 0x0, 250);

	return;
}
/*
============================================================================
 Function:				MainClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed before the application progra, is
 terminated.
============================================================================
*/
void MainClose()
{
	//
	//	Here will come code for all closing processes
	//
	cHost.MbusStopServer();
	MMC_CloseConnection(gConnHndl);
	return;
}
/*
============================================================================
 Function:				MachineSequences()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Starts the Main Timer function that will execute the states machines
 to control the system. Also performs a slow background loop for
 less time-critical background processes and monitoring of requests
 to terminate the application.
============================================================================
*/
void MachineSequences()
{
	//
	//	Init all variables of the states machines
	//
	MachineSequencesInit();
	//
	//	Enable MachineSequencesTimer() every TIMER_CYCLE ms
	//
	EnableMachineSequencesTimer(TIMER_CYCLE);
	//
	//	Background loop. Handles termination request and other less time-critical background proceses
	//
	while (!giTerminate)
	{
		//		iTimeStart = PmasTimeUpdate();
		//		printf("Timer is:  %ld\n", (unsigned long int)iTimeStart);
		MachineSequencesTimer(1);
		//		Execute background process if required
		//
		// BackgroundProcesses();
		//
		//		Sleep for ~SLEEP_TIME micro-seconds to reduce CPU load
		//
		//		usleep(SLEEP_TIME);
	}
	//
	//	Termination requested. Close what needs to be cloased at the states machines
	//
	MachineSequencesClose();

	return; // Back to the main() for program termination
}
/*
============================================================================
 Function:				MachineSequencesInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Initilaize the states machines variables
============================================================================
*/
void MachineSequencesInit()
{
	//
	//	Initializing all variables for the states machines
	//
	giTerminate = FALSE;
	giReentrance = FALSE;
	for (int m = 0; m < MAX_AXES; m++)
	{
		sSingleCommands[m].sState = ePowerOn;
		sSingleCommands[m].sSubState = eSubState_PowerOn;
	}

	return;
}
/*
============================================================================
 Function:				MachineSequencesClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void MachineSequencesClose()
{
	//
	//	Here will come code for all closing processes
	//
	return;
}
/*
============================================================================
 Function:				BackgroundProcesses()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Performs background processes that are not managed by the States Machines.
 This can be saving logs to the FLASH, performing background calculations etc.
 Generally speaking, this function, although colored in red in the Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void BackgroundProcesses()
{
	//
	//	Here will come code for all closing processes
	//
	return;
}
/*
============================================================================
 Function:				EnableMachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Enables the main machine sequences timer function, to be executed each
 TIMER_CYCLE ms.
============================================================================
*/
void EnableMachineSequencesTimer(int TimerCycle)
{
	struct sigaction stSigAction;

	// Whenever a signal is caught, call TerminateApplication function
	stSigAction.sa_handler = TerminateApplication;

	sigaction(SIGINT, &stSigAction, NULL);
	sigaction(SIGTERM, &stSigAction, NULL);
	sigaction(SIGABRT, &stSigAction, NULL);
	sigaction(SIGQUIT, &stSigAction, NULL);

	// Start syncronized timer in system - only with firmware supporting this.

	//	MMC_DestroySYNCTimer(gConnHndl) ;
	//	MMC_CreateSYNCTimer(gConnHndl,(IN MMC_SYNC_TIMER_CB_FUNC)MachineSequencesTimer,1);
	return;
}
/*
============================================================================
 Function:				MachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 A timer function that is called by the OS every TIMER_CYCLE ms.
 It executes the machine sequences states machines and actully controls
 the sequences and behavior of the machine.
============================================================================
*/
void MachineSequencesTimer(int iSig)
{
	//	In case the application is waiting for termination, do nothing.
	//	This can happen if giTerminate has been set, but the background loop
	//	didn't handle it yet (it has a long sleep every loop)
	//
	if (giTerminate == TRUE)
		return;
	//
	//	Avoid reentrance of this time function
	//
	//	Reentrance can theoretically happen if the execution of this timer function
	//	is wrongly longer than TIMER_CYCLE. In such case, reentrance should be avoided
	//	to prevent unexpected behavior (the code is not designed for reentrance).
	//
	//	In addition, some error handling should be taken by the user.
	//
	if (giReentrance)
	{
		//
		//		Print an error message and return. Actual code should take application related error handling
		//
		printf("Reentrancy!\n");

		return;
	}

	giReentrance = TRUE; // to enable detection of reentrancy. The flag is cleared at teh end of this function

	//
	//	Read all input data.
	//
	//	Here, every TIMER_CYCLE ms, the user should read all input data that may be
	//	required for the states machine code and copy them into "mirror" variables.
	//
	//	The states machines code, below, should use only the mirror variables, to ensure
	//	that all input data is synchronized to the timer event.
	//
	//	Input data can be from the Host (MODBUS) or from the drives or I/Os units
	//	(readingfrom the GMAS core firmware using one of the Function Blocks library
	//	functions) or from any other source.
	//
	ReadAllInputData();
	/*
	============================================================================

		States Machines code starts here!

	============================================================================
	*/

	//	The value of the State variable is used to make decisions of the main states machine and to call,
	//	as necessary, the relevant function that handles to process itslef in a sub-state machine.
	//

	for (int i = 0; i < MAX_AXES; i++)
	{
		if ((giStatus[i] & NC_AXIS_ERROR_STOP_MASK) || (!(sSingleCommands[i].cInErrorDriveOrGmas == eGmasNoError)))
		{
			if (sSingleCommands[i].lCommand & ERROR_ACK)
			{
				sSingleCommands[i].sState = eAckError;
				sSingleCommands[i].sSubState = eSubState_SMReset;
			}
		}

		if ((giStatus[i] & NC_AXIS_DISABLED_MASK) && ((sSingleCommands[i].lCommand & MOTOR_ENABLE) && (!(sSingleCommands[i].lOldCommand & MOTOR_ENABLE))))
		{
			sSingleCommands[i].sState = ePowerOn;
			sSingleCommands[i].sSubState = eSubState_PowerOn;
		}
		else if ((!(giStatus[i] & NC_AXIS_DISABLED_MASK)) && ((sSingleCommands[i].lCommand & MOTOR_DISABLE) && (!(sSingleCommands[i].lOldCommand & MOTOR_DISABLE))))
		{
			sSingleCommands[i].sState = ePowerOff;
			sSingleCommands[i].sSubState = eSubState_PowerOff;
		}
		else if ((giStatus[i] & NC_AXIS_STAND_STILL_MASK) && (sSingleCommands[i].lCommand & MOTION_S_HOME_START))
		{
			if (sSingleCommands[i].sState != eHome)
			{
				sSingleCommands[i].sState = eHome;
				sSingleCommands[i].sHomeState = MOTOR_INHOMING;
				sSingleCommands[i].sSubState = eSubState_GO_Home_ChOPMode;
			}
		}
		else if ((giStatus[i] & NC_AXIS_STAND_STILL_MASK) &&
				 ((sSingleCommands[i].lCommand & MOTION_S_START) || ((sSingleCommands[i].lCommand & MOTION_S_JOG) && (!(sSingleCommands[i].lOldCommand & MOTION_S_JOG)))))
		{
			sSingleCommands[i].sState = ePTP;
			sSingleCommands[i].sSubState = eSubState_PTP_ChOPMode;
		}
		else if ((giStatus[i] & NC_AXIS_DISCRETE_MOTION_MASK) &&
				 ((sSingleCommands[i].lCommand & MOTION_S_STOP) || ((sSingleCommands[i].lOldCommand & MOTION_S_JOG) && (!(sSingleCommands[i].lCommand & MOTION_S_JOG)))))
		{
			sSingleCommands[i].sState = eStop;
			sSingleCommands[i].sSubState = eSubState_Stop_Stop;
		}
		else if (sSingleCommands[i].lCommand & MOTION_OC_START)
		{
			sSingleCommands[i].sState = eOCStart;
			sStartVisionOCStep[i] = 1;
		}
		else if (sSingleCommands[i].lCommand & MOTION_OC_STOP)
		{
			sSingleCommands[i].sState = eOCStop;
			sStopVisionOCStep[i] = 1;
		}

		sSingleCommands[i].lOldCommand = sSingleCommands[i].lCommand;
		switch (sSingleCommands[i].sState)
		{
		case eIDLE:
		{
			break;
		}
		case eHome:
		{
			StateFunction_Home(i);
			break;
		}
		case ePTP:
		{
			StateFunction_PTP(i);
			break;
		}
		case eStop:
		{
			StateFunction_Stop(i);
			break;
		}
		case eAckError:
		{
			StateFunction_ACKError(i);
			break;
		}
		case ePowerOn:
		{
			StateFunction_PowerOn(i);
			break;
		}
		case ePowerOff:
		{
			StateFunction_PowerOff(i);
			break;
		}

		case eOCStart:
		{
			StartVisionOCInProcess(i);
			break;
		}
		case eOCStop:
		{
			StopVisionOCInProcess(i);
			break;
		}

		default:
		{
			break;
		}
		}
		sSingleCommands[i].lCommand = sSingleCommands[i].lCommand & MOTION_S_JOG;
	}

	for (int k = 0; k < MAX_VECTORS; k++)
	{
		if ((giGroupStatus[k] & NC_GROUP_ERROR_STOP_MASK) || (!(sGroup[k].cGmasInErr == eGmasNoError)))
		{
			if (gsDispensingInProcess[k] == 1)
			{
				gsDispensingInProcess[k] = 0;
				try
				{
					GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[0].iParameterIndex = 0;
					GroupOfPrmWrt[0].usPadding1 = 0;
					GroupOfPrmWrt[0].usPadding2 = 0;
					GroupOfPrmWrt[0].usPadding3 = 0;

					GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

					GroupOfPrmWrt[0].dbValue = 0x0;

					GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[1].iParameterIndex = 0;
					GroupOfPrmWrt[1].usPadding1 = 0;
					GroupOfPrmWrt[1].usPadding2 = 0;
					GroupOfPrmWrt[1].usPadding3 = 0;

					GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

					GroupOfPrmWrt[1].dbValue = 0x0;

					cVector[k].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 2);

					gsOC.ValveOCInitFlag = 0;
					cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
					cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0

					cAxes[eY].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
					cAxes[eY].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0
				}
				catch (CMMCException &ex)
				{
					printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cVector[k].GetRef(), ex.what(), (short)ex.error(), ex.status());
				}
			}
			if (sGroup[k].lCommand & ERROR_ACK)
			{
				sGroup[k].sState = eAckError;
				sGroup[k].sSubState = eSubState_Group_SMReset;
			}
		}
		if ((sGroup[k].lCommand & MOTION_V_LINEAR_START) && (giGroupStatus[k] & NC_GROUP_DISABLED_MASK))
		{
			sGroup[k].sState = eGroupLinear;
			sGroup[k].sSubState = eSubState_GroupLinear_Check;
		}
		else if ((sGroup[k].lCommand & MOTION_V_CIRCULAR_START) && (giGroupStatus[k] & NC_GROUP_DISABLED_MASK))
		{
			sGroup[k].sState = eGroupCircular;
			sGroup[k].sSubState = eSubState_GroupCircular_Check;
		}
		else if (sGroup[k].lCommand & MOTION_V_DISPENSING_START)
		{
			sGroup[k].sState = eDispensingStart;
			sGroup[k].sSubState = eSubState_Dispensing_FileOpen;
		}
		else if (sGroup[k].lCommand & MOTION_V_DISPENSING_STOP)
		{
			sGroup[k].sState = eDispensingStop;
			sGroup[k].sSubState = eSubState_DispensingStop_Stop;
		}
		else if (sGroup[k].lCommand & MOTION_V_DISPENSING_RESTART)
		{
			if (giGlobalPause == 1)
			{
				sGroup[k].sState = eDispensingStart;
				sGroup[k].sSubState = eSubState_Dispensing_ZBackToSafe;
			}
		}
		else if ((sGroup[k].lCommand & MOTION_V_STOP) && (giGroupStatus[k] & NC_GROUP_MOVING_MASK))
		{
			sGroup[k].sState = eStop;
			sGroup[k].sSubState = eSubState_GroupStop_Stop;
		}

		switch (sGroup[k].sState)
		{
		case eIDLE:
			break;
		case eGroupLinear:
			StateFunction_GroupLinearAbsolute(k);
			break;
		case eGroupCircular:
			StateFunction_GroupCircularAbsolute(k);
			break;
		case eStop:
			StateFunction_Group_Stop(k);
			break;
		case eAckError:
			StateFunction_Group_ACKError(k);
			break;
		case eDispensingStart:
			// StateFunction_DispensingProcess(k);
			break;
		case eDispensingStop:
			StateFunction_Dispensing_Stop(k);
			break;
		default:
			break;
		}
		sGroup[k].lCommand = 0;
	}
	//
	//	Write all output data
	//
	//	Here, every TIMER_CYCLE ms, after the execution of all states machines
	//	(based on all the Input Data as read from all sources at teh top of this function)
	//	the user should write all output data (written into mirror variables within the
	//	states machines code) to the "external world" (MODBUS, GMAS FW core, ...).
	//
	//	After alll states machines were executed and generated their outputs, the outputs
	//	are writen to the "external world" to actually execute the states machines "decisions"
	//
	WriteAllOutputData();
	//
	//	Clear the reentrancy flag. Now next execution of this function is allowed
	//
	giReentrance = FALSE;
	//
	return; // End of the sequences timer function. The function will be triggered again upon the next timer event.
}
/*
============================================================================
 Function:				ReadAllInputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 The data used during the processing of the states machines must be synchronized with the
 timer event and must be "frozen" during the processing of all states machines code, so that
 all decisions and calculations will use the same data.

 This function is called at the beginning of the timer function, each TIMER_CYCLE ms,
 to collect all required input data (as relevant to each application) and to copy
 it into "mirror" variables, that will be used by the state machines code.
 ============================================================================
*/
void ReadAllInputData()
{
	int i, k;
	short sCurrentOffset;
	giModbusDataTrue = 0;
	giModbusCmdReset = 0;
	cHost.MbusReadHoldingRegisterTable(0, 100, mbus_read_out);
	sCurrentOffset = MODBUS_W_COMMAND_CRC;
	if (Input_CalcCRC_IML(sCurrentOffset, 0) == (unsigned short)mbus_read_out.regArr[sCurrentOffset])
	{
		giModbusDataTrue = 1;
	}

	/*****************************************************************/
	/*
	if(( gsDispensingFinished[0]==1 )&&(sGroup[0].cGmasInErr == eGmasNoError))
	{
		gsDispensingFinished[0] = 0;
		giModbusDataTrue = 1;
	}
	 */
	/*****************************************************************/

	if ((giModbusDataTrue == 1) || (mbus_read_out.regArr[CmdTest] == 1))
	{
		mbus_write_in.startRef = CmdTest;
		mbus_write_in.regArr[0] = 0;
		mbus_write_in.refCnt = 1;
		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

		mbus_write_in.startRef = MODBUS_W_COMMAND_CRC;
		mbus_write_in.regArr[0] = 0;
		mbus_write_in.refCnt = 1;
		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

		sCurrentOffset = MODBUS_W_COMMAND_FRAMAE;

		gsCmdFrame.lCommand = GetModbusLong(sCurrentOffset);
		sCurrentOffset += 2;
		/*****************************************************************/
		//		gsCmdFrame.lCommand			= 256;
		/*****************************************************************/
		gsCmdFrame.sAxisIndex = mbus_read_out.regArr[sCurrentOffset];
		sCurrentOffset += 1;
		gsCmdFrame.sGroupIndex = mbus_read_out.regArr[sCurrentOffset];
		sCurrentOffset += 2;

		gsCmdFrame.sHomeMode = mbus_read_out.regArr[sCurrentOffset];
		gsCmdFrame.lJerk = GetModbusLong(sCurrentOffset);
		sCurrentOffset += 2;
		gsCmdFrame.lAcceleration = GetModbusLong(sCurrentOffset);

		sCurrentOffset = MODBUS_W_COMMAND_SINGLE_PARAMETER;
		for (i = 0; i < MAX_AXES; i++)
		{
			sSingleCommands[i].lVelocity = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sSingleCommands[i].lPosition = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
		}

		for (i = 0; i < MAX_AXES; i++)
		{
			if ((gsCmdFrame.sAxisIndex >> i) & 0x0001)
			{
				if ((gsCmdFrame.lCommand & MOTION_S_START) || (gsCmdFrame.lCommand & MOTION_S_STOP) || (gsCmdFrame.lCommand & ERROR_ACK) || (gsCmdFrame.lCommand & MOTOR_ENABLE) || (gsCmdFrame.lCommand & MOTOR_DISABLE))

				{
					sSingleCommands[i].lCommand = gsCmdFrame.lCommand;
				}
				else if (gsCmdFrame.lCommand & MOTION_S_HOME_START)
				{
					sSingleCommands[i].lCommand = gsCmdFrame.lCommand;
					sSingleCommands[i].lHomePosition = sSingleCommands[i].lPosition;
					sSingleCommands[i].lHomeVelocity = sSingleCommands[i].lVelocity;
					sSingleCommands[i].sHomeMode = gsCmdFrame.sHomeMode;
				}
				else if (gsCmdFrame.lCommand & MOTION_JOG_MODE)
				{
					gsCmdFrame.lCommand = gsCmdFrame.lCommand & (MOTION_S_JOG | MOTION_JOG_MODE);
					sSingleCommands[i].lCommand = gsCmdFrame.lCommand;
				}
				else if (gsCmdFrame.lCommand & MOTION_OC_START)
				{
					sCurrentOffset = MODBUS_W_COMMAND_OC_DATA;
					gsOC.Points = mbus_read_out.regArr[sCurrentOffset];
					sCurrentOffset += 1;
					gsOC.PulseDuration = mbus_read_out.regArr[sCurrentOffset];
					sCurrentOffset += 1;
					gsOC.Direction = mbus_read_out.regArr[sCurrentOffset];
					sCurrentOffset += 3;
					for (int n = 0; n < MAX_OC_POINTS; n++)
					{
						gsOC.Position[n] = 2 * GetModbusLong(sCurrentOffset);
						sCurrentOffset += 2;
					}
					sSingleCommands[i].lCommand = gsCmdFrame.lCommand;
				}
				else if (gsCmdFrame.lCommand & MOTION_OC_STOP)
				{
					sSingleCommands[i].lCommand = gsCmdFrame.lCommand;
				}
				else if (gsCmdFrame.lCommand & MOTION_X_DIGITAL_OUTPUT)
				{
					sCurrentOffset = MODBUS_W_COMMAND_X_DIGITAL_OUTPUT;
					try
					{
						GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[0].iParameterIndex = 0;
						GroupOfPrmWrt[0].usPadding1 = 0;
						GroupOfPrmWrt[0].usPadding2 = 0;
						GroupOfPrmWrt[0].usPadding3 = 0;

						GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[0].dbValue = (double)((long)mbus_read_out.regArr[sCurrentOffset] << 16);

						GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[1].iParameterIndex = 0;
						GroupOfPrmWrt[1].usPadding1 = 0;
						GroupOfPrmWrt[1].usPadding2 = 0;
						GroupOfPrmWrt[1].usPadding3 = 0;

						GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

						GroupOfPrmWrt[1].dbValue = (double)((long)mbus_read_out.regArr[sCurrentOffset] << 8);

						cAxes[eX].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 2);
					}
					catch (CMMCException &ex)
					{
						printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[eX].GetRef(), ex.what(), (short)ex.error(), ex.status());
					}
				}
			}
		}

		sCurrentOffset = MODBUS_W_COMMAND_GROUP_PARAMETER;

		for (k = 0; k < MAX_VECTORS; k++)
		{
			sGroup[k].lVelocity = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;

#ifdef SINGLE_VALVE_3D
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif

#ifdef SINGLE_VALVE_3DR
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif

#ifdef SINGLE_VALVE_3D2R
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Z_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif

#ifdef SINGLE_VALVE
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif

#ifdef SINGLE_VALVE_R
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif

#ifdef DOUBLE_VALVE
			sGroup[k].lEndPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lEndPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_X_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
			sGroup[k].lAuxPosition[NC_Y_AXIS_TYPE] = GetModbusLong(sCurrentOffset);
			sCurrentOffset += 2;
#endif
		}

		for (k = 0; k < MAX_VECTORS; k++)
		{
			if ((gsCmdFrame.sGroupIndex >> k) & 0x0001)
			{
				if ((gsCmdFrame.lCommand & MOTION_V_LINEAR_START) || (gsCmdFrame.lCommand & MOTION_V_CIRCULAR_START) || (gsCmdFrame.lCommand & MOTION_V_STOP) || (gsCmdFrame.lCommand & ERROR_ACK) || (gsCmdFrame.lCommand & MOTION_V_DISPENSING_START) || (gsCmdFrame.lCommand & MOTION_V_DISPENSING_STOP) || (gsCmdFrame.lCommand & MOTION_V_DISPENSING_RESTART))
				{
					sGroup[k].lCommand = gsCmdFrame.lCommand;
				}
				if (gsCmdFrame.lCommand & MOTION_VALVE_ACK)
				{
					sCurrentOffset = MODBUS_W_VALVE_PARAMETER_ACK;
					sGroup[k].sValveAckCode = mbus_read_out.regArr[sCurrentOffset + k];
				}

				if (gsCmdFrame.lCommand & MOTION_V_DISPENSING_PAUSE)
				{
					if (gsDispensingInProcess[k] == 1)
						giGlobalPause = 1;
				}
			}
		}
		if (gsCmdFrame.lCommand & MOTION_PARAMETER_SETTING)
		{
			for (k = 0; k < MAX_VECTORS; k++)
			{
				sGroup[k].lAcceleration = gsCmdFrame.lAcceleration;
				sGroup[k].lDeceleration = gsCmdFrame.lAcceleration;
				sGroup[k].lJerk = gsCmdFrame.lJerk;
			}
			for (i = 0; i < MAX_AXES; i++)
			{
				sSingleCommands[i].lAcceleration = gsCmdFrame.lAcceleration;
				sSingleCommands[i].lDeceleration = gsCmdFrame.lAcceleration;
				sSingleCommands[i].lJerk = gsCmdFrame.lJerk;
			}
		}
		if (gsCmdFrame.lCommand & MOTION_SYSTEM_RESTART)
		{
			try
			{
				// iResetrc = MMC_ResetSystem(gConnHndl,&ResetIn,&ResetOut);
				system("reboot");
			}
			catch (CMMCException &ex)
			{
				printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
			}
		}

		if (!(gsCmdFrame.lCommand & MOTION_JOG_MODE))
			gsCmdFrame.lCommand = 0;

		giModbusCmdReset = 1;
	}
	for (i = 0; i < MAX_AXES; i++)
	{
		giStatus[i] = (int)cAxes[i].ReadStatus();
		giPos[i] = (int)cAxes[i].GetActualPosition();
	}
	for (k = 0; k < MAX_VECTORS; k++)
	{
		giGroupStatus[k] = cVector[k].GroupReadStatus();
	}
	return;
}
/*
============================================================================
 Function:				WriteAllOutputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 Write all the data, generated by the states machines code, to the "external
 world".

 This is done here (and not inside the states machines code) to ensure that
 all data is updated simultaneously and in a synchronous way.

 The states machines code write the data into "mirror" variables, that are here
 copied or sent to the "external world" (Host via MODBUS, GMAS core firmware, etc.)
  ============================================================================
*/
void WriteAllOutputData()
{
	int i;

	// Build the statuses - General and individual:
	for (i = 0; i < MAX_AXES; i++)
	{
		sSingleStatus[i] = 0;
		// Buid the status for the Modbus message ;
		if (!(giStatus[i] & NC_AXIS_DISABLED_MASK))
		{
			sSingleStatus[i] |= MOTOR_ENABLED;
		}
		if (!(giStatus[i] & NC_AXIS_STAND_STILL_MASK) && (sSingleStatus[i] & MOTOR_ENABLED))
		{
			sSingleStatus[i] |= MOTOR_INMOTION;
		}
		else
		{
			// Only if motor enabled then in position:
			if (sSingleStatus[i] & MOTOR_ENABLED)
			{
				sSingleStatus[i] |= MOTOR_INPOSITION;
			}
		}
		if ((giStatus[i] & NC_AXIS_ERROR_STOP_MASK) || (sSingleCommands[i].cInErrorDriveOrGmas))
		{
			sSingleStatus[i] |= MOTOR_INERROR;
		}
		if (sSingleCommands[i].sHomeState == MOTOR_INHOMING)
		{
			sSingleStatus[i] |= MOTOR_INHOMING;
		}
		if (sSingleCommands[i].sHomeState == MOTOR_HOMEDONE)
		{
			sSingleStatus[i] |= MOTOR_HOMEDONE;
		}
		if (sSingleCommands[i].sOCState == MOTOR_OC_ERROR)
		{
			sSingleStatus[i] |= MOTOR_OC_ERROR;
		}
		if (sSingleCommands[i].sOCState == MOTOR_OC_DATA_DOWNLOADING)
		{
			sSingleStatus[i] |= MOTOR_OC_DATA_DOWNLOADING;
		}
		if (sSingleCommands[i].sOCState == MOTOR_OC_DATA_DOWNLOADED)
		{
			sSingleStatus[i] |= MOTOR_OC_DATA_DOWNLOADED;
		}
		if (sSingleCommands[i].sOCState == MOTOR_OC_IS_DISABLED)
		{
			sSingleStatus[i] |= MOTOR_OC_IS_DISABLED;
		}
		if (sSingleCommands[i].sOCState == MOTOR_OC_IS_ENABLED)
		{
			sSingleStatus[i] |= MOTOR_OC_IS_ENABLED;
		}
		if (sSingleCommands[i].sDIErrState == MOTOR_DI_ERROR)
		{
			sSingleStatus[i] |= MOTOR_DI_ERROR;
		}
	}

	for (i = 0; i < MAX_VECTORS; i++)
	{
		sGroupStatus[i] = 0;
		if (!(giGroupStatus[i] & NC_GROUP_DISABLED_MASK))
		{
			sGroupStatus[i] |= GROUP_ENABLED;
		}
		if (giGroupStatus[i] & NC_GROUP_MOVING_MASK)
		{
			sGroupStatus[i] |= GROUP_INMOTION;
			gsGroupInPosition[i] = 0;
		}
		if (gsGroupInPosition[i] == 1)
		{
			sGroupStatus[i] |= GROUP_INPOSITION;
		}
		if ((giGroupStatus[i] & NC_GROUP_ERROR_STOP_MASK) || (sGroup[i].cGmasInErr))
		{
			sGroupStatus[i] |= GROUP_INERROR;
		}
		if (gsDispensingFinished[i] == 1)
		{
			sGroupStatus[i] |= GROUP_DISPENSING_FINISHED;
		}
		if (gsDispensingInProcess[i] == 1)
		{
			sGroupStatus[i] |= GROUP_DISPENSING_IN_PROCESS;
		}
		if (gsGroupInValveChg[i] == 1)
		{
			sGroupStatus[i] |= GROUP_IN_VALVE_CHG_EVENT;
		}
		else
		{
			sGroup[i].sValveChgCode = 0;
			sGroup[i].sValveAckCode = 0;
		}
	}

	if (giModbusCmdReset == 1)
	{
		mbus_write_in.startRef = MODBUS_W_COMMAND_FRAMAE;
		mbus_write_in.regArr[0] = gsCmdFrame.lCommand;
		mbus_write_in.refCnt = 2;
		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);
	}

	mbus_write_in.startRef = MODBUS_R_VALVE_PARAMETER_EVENT;
	mbus_write_in.refCnt = MAX_VECTORS;
	for (i = 0; i < MAX_VECTORS; i++)
	{
		mbus_write_in.regArr[i] = sGroup[i].sValveChgCode;
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	try
	{
		iProcessCode = cAxes[eX].GetBoolParameter(MMC_I_COMM_EV_USR_2_PARAM, 0);
		iUnitCode = cAxes[eX].GetBoolParameter(MMC_I_COMM_EV_USR_3_PARAM, 0);
		iLineCode = cAxes[eX].GetBoolParameter(MMC_I_COMM_EV_USR_4_PARAM, 0);

		mbus_write_in.startRef = MODBUS_R_PROCESS_CODE;
		mbus_write_in.refCnt = 1;

		mbus_write_in.regArr[0] = (short)iProcessCode;

		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

		mbus_write_in.startRef = MODBUS_R_UNIT_CODE;
		mbus_write_in.refCnt = 2;
		InsertLongVarToModbusShortArr(&mbus_write_in.regArr[0], (long)iUnitCode);
		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

		mbus_write_in.startRef = MODBUS_R_LINE_CODE;
		mbus_write_in.refCnt = 2;
		InsertLongVarToModbusShortArr(&mbus_write_in.regArr[0], (long)iLineCode);
		cHost.MbusWriteHoldingRegisterTable(mbus_write_in);
	}
	catch (CMMCException &ex)
	{
		//	printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[i].GetRef(), ex.what(), (short)ex.error(), ex.status());
	}

	mbus_write_in.startRef = MODBUS_R_AXIS_STATUS;
	mbus_write_in.refCnt = MAX_AXES;
	for (i = 0; i < MAX_AXES; i++)
	{
		mbus_write_in.regArr[i] = sSingleStatus[i];
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_AXIS_POSITION;
	mbus_write_in.refCnt = 2 * MAX_AXES;
	for (i = 0; i < MAX_AXES; i++)
	{
		InsertLongVarToModbusShortArr(&mbus_write_in.regArr[2 * i], (long)giPos[i]);
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_ERROR_AXISCODE;
	mbus_write_in.refCnt = 2 * MAX_AXES;

	for (i = 0; i < MAX_AXES; i++)
	{
		try
		{
			cPIReadVar[i].ReadPI(usiReadValue[i]);
			sSingleCommands[i].lLastDriveMotorFault = (long)usiReadValue[i];
		}
		catch (CMMCException &ex)
		{
			;
		}
		InsertLongVarToModbusShortArr(&mbus_write_in.regArr[2 * i], sSingleCommands[i].lLastDriveMotorFault);
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_ERROR_GMASCODE;
	mbus_write_in.refCnt = MAX_AXES;
	for (i = 0; i < MAX_AXES; i++)
	{
		mbus_write_in.regArr[i] = sSingleCommands[i].sLastGMASError;
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_GROUP_STATUS;
	mbus_write_in.refCnt = MAX_VECTORS;
	for (i = 0; i < MAX_VECTORS; i++)
	{
		mbus_write_in.regArr[i] = sGroupStatus[i];
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_DISPENSING_COUNTER;
	mbus_write_in.refCnt = MAX_VECTORS;
	for (i = 0; i < MAX_VECTORS; i++)
	{
		mbus_write_in.regArr[i] = sDispensingCounter[i];
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_ERROR_GROUPCODE;
	mbus_write_in.refCnt = MAX_VECTORS;
	for (i = 0; i < MAX_VECTORS; i++)
	{
		mbus_write_in.regArr[i] = sGroup[i].sLastGMASError;
	}
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	mbus_write_in.startRef = MODBUS_R_DIGITAL_IN;
	mbus_write_in.refCnt = 2;

	try
	{
		cPIDIReadVar[eX].ReadPI(iDIValue[eX]);
		cPIDIReadVar[eY].ReadPI(iDIValue[eY]);
	}
	catch (CMMCException &ex)
	{
		;
	}

	mbus_write_in.regArr[0] = (short)((iDIValue[eX] >> 16) & 0xFFFF);
	mbus_write_in.regArr[1] = (short)((iDIValue[eY] >> 16) & 0xFFFF);
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in);

	return;
}
/*
============================================================================
 Function:				StateFunction_PTP()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the PTP process.

	 Change Operation Mode.
	 Power Enable
	 Start Point To Point
	 Wait for end of Point to Point
	 Change Direction back
	 Wait for End of Point to Point
	 Power Off

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_PTP(int iAxis)
{	//
	//	Handle the sub-state machine.
	//
	//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
	//	as necessary, the relevant function that handles to process itself.
	//
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case eSubState_PTP_ChOPMode:

			// Change OP mode to Profile Position.
			if (cAxes[iAxis].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE)
			{
				cAxes[iAxis].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
			}
			sSingleCommands[iAxis].sSubState = eSubState_PTP_WChOPMode;
			break;
		case eSubState_PTP_WChOPMode:

			// Check we are in Profile Position in all axes:
			if (cAxes[iAxis].GetOpMode() == OPM402_CYCLIC_SYNC_POSITION_MODE)
			{
				sSingleCommands[iAxis].sSubState = eSubState_PTP_Move;
			}
			break;

		case eSubState_PTP_Move:

			cAxes[iAxis].m_fAcceleration = (float)sSingleCommands[iAxis].lAcceleration;
			cAxes[iAxis].m_fDeceleration = (float)sSingleCommands[iAxis].lDeceleration;
			cAxes[iAxis].m_fJerk = (float)sSingleCommands[iAxis].lJerk;
			cAxes[iAxis].m_eDirection = sSingleCommands[iAxis].eDirection;
			cAxes[iAxis].m_fVelocity = (float)sSingleCommands[iAxis].lVelocity;
			cAxes[iAxis].m_ucExecute = 1;

			cAxes[iAxis].MoveAbsolute((double)sSingleCommands[iAxis].lPosition, MC_ABORTING_MODE);

			sSingleCommands[iAxis].sSubState = eSubState_PTP_WMove;
			break;

		case eSubState_PTP_WMove:

			if (giStatus[iAxis] & NC_AXIS_STAND_STILL_MASK)
			{
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;

			//
			//		The default case. Should not happen, the user can implement error handling.
			//
		default:
		{
			break;
		}
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	//
	return;
}
void StateFunction_ACKError(int iAxis)
{
	//
	//	Handle the sub-state machine.
	//
	//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
	//	as necessary, the relevant function that handles to process itself.
	//
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case eSubState_SMReset:

			if (giStatus[iAxis] & NC_AXIS_ERROR_STOP_MASK)
			{
				usiWriteValue[iAxis] = 6;
				cPIWriteVar[iAxis].WritePI(usiWriteValue[iAxis]);
				usleep(5000);
				cAxes[iAxis].Reset();
				sSingleCommands[iAxis].sSubState = eSubState_SMReset_WAIT;
			}
			else
				sSingleCommands[iAxis].sSubState = eSubState_SMReset_DONE;

			break;
		case eSubState_SMReset_WAIT:

			if (!(giStatus[iAxis] & NC_AXIS_ERROR_STOP_MASK))
			{
				sSingleCommands[iAxis].sSubState = eSubState_SMReset_DONE;
			}
			break;

		case eSubState_SMReset_DONE:

			sSingleCommands[iAxis].sLastGMASError = 0;
			sSingleCommands[iAxis].lLastDriveMotorFault = 0;
			sSingleCommands[iAxis].cInErrorDriveOrGmas = 0;
			sSingleCommands[iAxis].sState = eIDLE;
			sSingleCommands[iAxis].sHomeState = eIDLE;
			sSingleCommands[iAxis].sOCState = eIDLE;
			sSingleCommands[iAxis].sDIErrState = 0;
			cAxes[iAxis].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);

			break;
			//
			//		The default case. Should not happen, the user can implement error handling.
			//
		default:
		{
			break;
		}
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	//
	return;
}

void StateFunction_Group_ACKError(int iGroup)
{
	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_Group_SMReset:

			// Check if we are here because of PLCopen error or simple error
			if (giGroupStatus[iGroup] & NC_GROUP_ERROR_STOP_MASK)
			{
				cVector[iGroup].GroupReset();
				sGroup[iGroup].sSubState = eSubState_Group_SMReset_WAIT;
			}
			else
			{
				cVector[iGroup].GroupDisable();
				sGroup[iGroup].sSubState = eSubState_Group_SMReset_DONE;
			}

			break;
		case eSubState_Group_SMReset_WAIT:

			if (!(giGroupStatus[iGroup] & NC_GROUP_ERROR_STOP_MASK))
			{
				sGroup[iGroup].sSubState = eSubState_Group_SMReset_DONE;
				cVector[iGroup].GroupDisable();
			}
			break;

		case eSubState_Group_SMReset_DONE:

			sGroup[iGroup].cGmasInErr = eGmasNoError;
			sGroup[iGroup].sLastGMASError = 0;
			sGroup[iGroup].sState = eIDLE;
			break;

		default:
		{
			break;
		}
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
}

void StateFunction_Stop(int iAxis)
{
	//
	//	Handle the sub-state machine.
	//
	//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
	//	as necessary, the relevant function that handles to process itself.
	//
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case eSubState_Stop_Stop:
			cAxes[iAxis].Stop();
			sSingleCommands[iAxis].sSubState = eSubState_Stop_WStop;
			break;
		case eSubState_Stop_WStop:
			if (giStatus[iAxis] & NC_AXIS_STAND_STILL_MASK)
			{
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}

	//
	//		The default case. Should not happen, the user can implement error handling.
	//
	//
	return;
}

void StateFunction_Group_Stop(int iGroup)
{

	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_GroupStop_Stop:

			cVector[iGroup].GroupStop((float)sGroup[iGroup].lDeceleration, sGroup[iGroup].lJerk, MC_ABORTING_MODE);
			sGroup[iGroup].sSubState = eSubState_GroupStop_WStop;
			break;

		case eSubState_GroupStop_WStop:
			if (giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK)
			{
				sGroup[iGroup].sState = eIDLE;
				cVector[iGroup].GroupDisable();
				gsGroupInPosition[iGroup] = 1;
			}
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
}

void StateFunction_Dispensing_Stop(int iGroup)
{

	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_DispensingStop_Stop:

			usleep(10000);
			gsDispensingFinished[iGroup] = 1;
			gsDispensingInProcess[iGroup] = 0;
			gsGroupInValveChg[iGroup] = 0;

			cVector[iGroup].m_ucExecute = 1;

			GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
			GroupOfPrmWrt[0].iParameterIndex = 0;
			GroupOfPrmWrt[0].usPadding1 = 0;
			GroupOfPrmWrt[0].usPadding2 = 0;
			GroupOfPrmWrt[0].usPadding3 = 0;

			GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

			GroupOfPrmWrt[0].dbValue = 0x0;

			GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
			GroupOfPrmWrt[1].iParameterIndex = 0;
			GroupOfPrmWrt[1].usPadding1 = 0;
			GroupOfPrmWrt[1].usPadding2 = 0;
			GroupOfPrmWrt[1].usPadding3 = 0;

			GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

			GroupOfPrmWrt[1].dbValue = 0x0;

			cVector[iGroup].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 2);

			cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0

			cAxes[eY].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			cAxes[eY].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0

			if (giGroupStatus[iGroup] & NC_GROUP_MOVING_MASK)
			{
				cVector[iGroup].GroupStop((float)sGroup[iGroup].lDeceleration, sGroup[iGroup].lJerk, MC_ABORTING_MODE);
				sGroup[iGroup].sSubState = eSubState_DispensingStop_WStop;
			}
			else if ((giGroupStatus[iGroup] & NC_GROUP_DISABLED_MASK) || (giGroupStatus[iGroup] & NC_GROUP_ERROR_STOP_MASK))
			{
				stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
				rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

				sGroup[iGroup].sState = eIDLE;
			}
			else if (giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK)
			{
				sGroup[iGroup].sSubState = eSubState_DispensingStop_WStop;
			}
			break;

		case eSubState_DispensingStop_WStop:
			if (giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK)
			{
				stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
				rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

				sGroup[iGroup].sState = eIDLE;
				cVector[iGroup].GroupDisable();
			}
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
}

/*
============================================================================
 Function:				StateFunction_2()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				14/09/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the XY move process.

 The move prcess, in this simplified example consists of the following steps:

 Begin move
 Wait for end of motion

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_Home(int iAxis)
{
	//
	//	Handle the sub-state machine.
	//
	//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
	//	as necessary, the relevant function that handles to process itself.
	//
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{

		case eSubState_GO_Home_ChOPMode:

			try
			{
				cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1);
			}
			catch (CMMCException &ex)
			{
				;
			}
			cAxes[iAxis].SetOpMode(OPM402_HOMING_MODE);
			sSingleCommands[iAxis].sSubState = eSubState_GO_Home_WChOPMode;

			break;

		case eSubState_GO_Home_WChOPMode:

			if (cAxes[iAxis].GetOpMode() == OPM402_HOMING_MODE)
			{
				sSingleCommands[iAxis].sSubState = eSubState_GO_Home_Move;
			}
			break;

		case eSubState_GO_Home_Move:

			stDS402Home.dbPosition = (double)sSingleCommands[iAxis].lHomePosition;
			stDS402Home.fAcceleration = 1.0E5;
			stDS402Home.fVelocity = (float)sSingleCommands[iAxis].lHomeVelocity;
			stDS402Home.fDistanceLimit = 100000000;
			stDS402Home.fTorqueLimit = 1;
			stDS402Home.eBufferMode = MC_BUFFERED_MODE;
			stDS402Home.uiHomingMethod = sSingleCommands[iAxis].sHomeMode;
			stDS402Home.uiTimeLimit = 100000000;
			cAxes[iAxis].SetDefaultHomeDS402Params(stDS402Home);
			cAxes[iAxis].HomeDS402();
			sSingleCommands[iAxis].sSubState = eSubState_GO_Home_WMove;

			break;

		case eSubState_GO_Home_WMove:

			if (giStatus[iAxis] & NC_AXIS_STAND_STILL_MASK)
			{
				iHomeWStartTime[iAxis] = PmasTimeUpdate();
				sSingleCommands[iAxis].sSubState = eSubState_GO_Home_ChCSPMode;
			}
			break;

		case eSubState_GO_Home_ChCSPMode:

			iHomeWEndTime[iAxis] = PmasTimeUpdate();
			if ((int)(iHomeWEndTime[iAxis] - iHomeWStartTime[iAxis]) >= 2000000)
			{
				cAxes[iAxis].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
				sSingleCommands[iAxis].sSubState = eSubState_GO_Home_WChCSPMode;
			}
			break;

		case eSubState_GO_Home_WChCSPMode:

			if (cAxes[iAxis].GetOpMode() == OPM402_CYCLIC_SYNC_POSITION_MODE)
			{
				sSingleCommands[iAxis].sHomeState = MOTOR_HOMEDONE;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;

		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}

	return;
}
void StateFunction_GroupLinearAbsolute(int iGroup)
{
	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_GroupLinear_Check:

			if (giGroupStatus[iGroup] & NC_GROUP_DISABLED_MASK)
			{
				sGroup[iGroup].sSubState = eSubState_GroupLinear_Init;
			}
			break;

		case eSubState_GroupLinear_Init:
			cVector[iGroup].GroupEnable();
			cVector[iGroup].SetKinTransform(stSetKinTransform[iGroup]);
			sGroup[iGroup].sSubState = eSubState_GroupLinear_Run;
			break;

		case eSubState_GroupLinear_Run:
			if (giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK)
			{
				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eCoordSystem = sGroup[iGroup].eCoordSystem;
				cVector[iGroup].m_eTransitionMode = sGroup[iGroup].eTranstion;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_fVelocity = (float)sGroup[iGroup].lVelocity;

#ifdef SINGLE_VALVE_3D
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);
#endif

#ifdef SINGLE_VALVE_3DR
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);
#endif

#ifdef SINGLE_VALVE_3D2R
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);
#endif

#ifdef SINGLE_VALVE
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
#endif

#ifdef SINGLE_VALVE_R
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
#endif

#ifdef DOUBLE_VALVE
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
#endif

				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
				sGroup[iGroup].sSubState = eSubState_GroupLinear_WRun;
			}
			break;

		case eSubState_GroupLinear_WRun:
			if ((giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK))
			{
				cVector[iGroup].GroupDisable();
				gsGroupInPosition[iGroup] = 1;
				sGroup[iGroup].sState = eIDLE;
			}
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
}

void StateFunction_GroupCircularAbsolute(int iGroup)
{
	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_GroupCircular_Check:

			if (giGroupStatus[iGroup] & NC_GROUP_DISABLED_MASK)
			{
				sGroup[iGroup].sSubState = eSubState_GroupCircular_Init;
			}
			break;

		case eSubState_GroupCircular_Init:

			cVector[iGroup].GroupEnable();
			cVector[iGroup].SetKinTransform(stSetKinTransform[iGroup]);
			sGroup[iGroup].sSubState = eSubState_GroupCircular_Run;
			break;

		case eSubState_GroupCircular_Run:

			if (giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK)
			{
				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eCoordSystem = sGroup[iGroup].eCoordSystem;
				cVector[iGroup].m_eTransitionMode = sGroup[iGroup].eTranstion;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_fVelocity = (float)sGroup[iGroup].lVelocity;

#ifdef SINGLE_VALVE_3D
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);

				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Z_AXIS_TYPE]);
				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);

				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Z_AXIS_TYPE]);
				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Z_AXIS_TYPE]);

				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Z_AXIS_TYPE]);
				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE
				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_R
				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef DOUBLE_VALVE
				cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lAuxPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_X_AXIS_TYPE]);
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)(sGroup[iGroup].lEndPosition[NC_Y_AXIS_TYPE]);

				cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

				sGroup[iGroup].sSubState = eSubState_GroupCircular_WRun;
			}
			break;

		case eSubState_GroupCircular_WRun:
			if ((giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK))
			{
				cVector[iGroup].GroupDisable();
				gsGroupInPosition[iGroup] = 1;
				sGroup[iGroup].sState = eIDLE;
			}
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
}

void StateFunction_PowerOn(int iAxis)
{
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case eSubState_PowerOn:

			cAxes[iAxis].PowerOn();
			sSingleCommands[iAxis].sSubState = eSubState_WPowerOn;

			break;
		case eSubState_WPowerOn:
			if (!(giStatus[iAxis] & NC_AXIS_DISABLED_MASK))
			{
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

void StateFunction_PowerOff(int iAxis)
{
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case eSubState_PowerOff:

			cAxes[iAxis].PowerOff();
			sSingleCommands[iAxis].sSubState = eSubState_WPowerOff;

			break;
		case eSubState_WPowerOff:
			if (giStatus[iAxis] & NC_AXIS_DISABLED_MASK)
			{
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

void SingleAxisParameterInit()
{
	for (int k = 0; k < MAX_AXES; k++)
	{
		sSingleCommands[k].lAcceleration = 1.5E+07;
		sSingleCommands[k].lDeceleration = 1.5E+07;
		sSingleCommands[k].lJerk = 1.5E+08;
		sSingleCommands[k].eDirection = MC_POSITIVE_DIRECTION;
	}
}

void MultiAxesParameterInit()
{

#ifdef SINGLE_VALVE

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 2;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

#endif

#ifdef SINGLE_VALVE_R

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 2;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

#endif

#ifdef SINGLE_VALVE_3D

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].eType[2] = NC_Z_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].hNode[2] = cAxes[eZ].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[2] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 3;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[2][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

#endif

#ifdef SINGLE_VALVE_3DR

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].eType[2] = NC_Z_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].hNode[2] = cAxes[eZ].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[2] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 3;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[2][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

#endif

#ifdef SINGLE_VALVE_3D2R

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].eType[2] = NC_Z_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].hNode[2] = cAxes[eZ].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[2] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 3;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[2][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[2][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

#endif

#ifdef DOUBLE_VALVE

	sGroup[eV0].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV0].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV0].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV0].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV0].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV0].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV0].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV0].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV0].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV0].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV0].iNumAxes = 2;
	stSetKinTransform[eV0].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV0].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV0].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV0].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV0].ucExecute = 1;

	gsGroupInPosition[eV0] = 1;
	gsDispensingFinished[eV0] = 1;
	gsDispensingInProcess[eV0] = 0;

	sGroup[eV1].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV1].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV1].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV1].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV1].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV1].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV1].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV1].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV1].hNode[0] = cAxes[eX1].GetRef();
	stSetKinTransform[eV1].hNode[1] = cAxes[eY1].GetRef();
	stSetKinTransform[eV1].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV1].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV1].iNumAxes = 2;
	stSetKinTransform[eV1].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV1].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV1].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV1].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV1].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV1].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV1].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV1].ucExecute = 1;

	gsGroupInPosition[eV1] = 1;
	gsDispensingFinished[eV1] = 1;
	gsDispensingInProcess[eV1] = 0;

	sGroup[eV2].lAcceleration = 1.5E+07; // 1.5E+07
	sGroup[eV2].lDeceleration = 1.5E+07; // 1.5E+07
	sGroup[eV2].lJerk = 1.5E+08;		 // 1.5E+08
	sGroup[eV2].eTranstion = MC_TM_NONE_MODE;
	sGroup[eV2].eBufferMode = MC_BUFFERED_MODE;
	sGroup[eV2].eCoordSystem = MC_MCS_COORD;

	stSetKinTransform[eV2].eType[0] = NC_X_AXIS_TYPE;
	stSetKinTransform[eV2].eType[1] = NC_Y_AXIS_TYPE;
	stSetKinTransform[eV2].eType[2] = NC_N1_AXIS_TYPE;
	stSetKinTransform[eV2].eType[3] = NC_N2_AXIS_TYPE;
	stSetKinTransform[eV2].hNode[0] = cAxes[eX].GetRef();
	stSetKinTransform[eV2].hNode[1] = cAxes[eY].GetRef();
	stSetKinTransform[eV2].hNode[2] = cAxes[eX1].GetRef();
	stSetKinTransform[eV2].hNode[3] = cAxes[eY1].GetRef();
	stSetKinTransform[eV2].iMcsToAcsFuncID[0] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV2].iMcsToAcsFuncID[1] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV2].iMcsToAcsFuncID[2] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV2].iMcsToAcsFuncID[3] = NC_TR_SHIFT_FUNC;
	stSetKinTransform[eV2].iNumAxes = 4;
	stSetKinTransform[eV2].ulTrCoef[0][0] = 1;
	stSetKinTransform[eV2].ulTrCoef[0][1] = 1;
	stSetKinTransform[eV2].ulTrCoef[0][2] = 0;
	stSetKinTransform[eV2].ulTrCoef[1][0] = 1;
	stSetKinTransform[eV2].ulTrCoef[1][1] = 1;
	stSetKinTransform[eV2].ulTrCoef[1][2] = 0;
	stSetKinTransform[eV2].ulTrCoef[2][0] = 1;
	stSetKinTransform[eV2].ulTrCoef[2][1] = 1;
	stSetKinTransform[eV2].ulTrCoef[2][2] = 0;
	stSetKinTransform[eV2].ulTrCoef[3][0] = 1;
	stSetKinTransform[eV2].ulTrCoef[3][1] = 1;
	stSetKinTransform[eV2].ulTrCoef[3][2] = 0;

	stSetKinTransform[eV2].eBufferMode = MC_BUFFERED_MODE;
	stSetKinTransform[eV2].ucExecute = 1;

	gsGroupInPosition[eV2] = 1;
	gsDispensingFinished[eV2] = 1;
	gsDispensingInProcess[eV2] = 0;
#endif
}

#if 0
void StateFunction_DispensingProcess(int iGroup)
{
	try
	{
		switch (sGroup[iGroup].sSubState)
		{
		case eSubState_Dispensing_FileOpen:

			iCycleStart = PmasTimeUpdate();

			gsDispensingInProcess[iGroup] = 0;
			gsDispensingFinished[iGroup] = 0;
			gsGroupInValveChg[iGroup] = 0;
			gsOCIsInProcess = 0;
			giGlobalPause = 0;
			gsOC.ValveOCInitFlag = 0;

			fp[iGroup].open("motion_profile.txt", std::ios::in);
			if (!fp[iGroup].is_open())
			{
				fp[iGroup].clear();
				sGroup[iGroup].sSubState = eIDLE;
			}
			else
			{
				gsDispensingInProcess[iGroup] = 1;
				sDispensingCounter[iGroup]++;
				sGroup[iGroup].sSubState = eSubState_Dispensing_FileAccess;
			}
			break;
		case eSubState_Dispensing_FileAccess:
			iIndex = 0;
			memset(buff, 0x0, sizeof(buff));
			memset(iTable, 0x0, sizeof(iTable));
			memset(gsCode, 0x0, sizeof(gsCode));
			while (!fp[iGroup].eof())
			{
				fp[iGroup].getline(buff, sizeof(buff));
				lp[iGroup] = strtok(buff, "\r\n");
				if (lp[iGroup])
				{
					iPointer = 0;
					p[iGroup] = strtok(lp[iGroup], " ");

					if (p[iGroup])
						iTable[iPointer] = atoi(p[iGroup]);
					else
						break;
					iPointer++;
					while (p[iGroup])
					{
						p[iGroup] = strtok(NULL, " ");
						if (p[iGroup])
							iTable[iPointer] = atoi(p[iGroup]);
						else
							break;
						iPointer++;
					}
					gsCode[iIndex].Index = iTable[0];
					gsCode[iIndex].FunCode = iTable[1];
					gsCode[iIndex].ProcessID = iTable[2];
					gsCode[iIndex].UnitID = iTable[3];

					if (gsCode[iIndex].FunCode == 6) // �Ƚ��������������ȡ
					{
#ifdef SINGLE_VALVE_3DR
						gsCode[iIndex].Points = iTable[12];		   // �Ƚ��������
						gsCode[iIndex].PulseDuration = iTable[13]; // �Ƚ�����������
						gsCode[iIndex].Direction = iTable[14];	   // �Ƚ��������
						for (int m = 0; m < MAX_OC_POINTS; m++)
						{
							gsCode[iIndex].OCPosition[m] = 2 * iTable[16 + m]; // �Ƚ����λ������
						}
#else
						gsCode[iIndex].Points = iTable[11];		   // �Ƚ��������
						gsCode[iIndex].PulseDuration = iTable[12]; // �Ƚ�����������
						gsCode[iIndex].Direction = iTable[13];	   // �Ƚ��������
						for (int m = 0; m < MAX_OC_POINTS; m++)
						{
							gsCode[iIndex].OCPosition[m] = 2 * iTable[15 + m]; // �Ƚ����λ������
						}
#endif
					}
					else if (gsCode[iIndex].FunCode == 12)
					{
#ifdef SINGLE_VALVE_3DR
						gsCode[iIndex].StartPosition = iTable[12]; // �Ƚ������ʼλ��
						gsCode[iIndex].DeltaPosition = iTable[13]; // �Ƚ�������
						gsCode[iIndex].PulseDuration = iTable[14]; // �Ƚ�����������
#else
						gsCode[iIndex].StartPosition = iTable[11]; // �Ƚ������ʼλ��
						gsCode[iIndex].DeltaPosition = iTable[12]; // �Ƚ�������
						gsCode[iIndex].PulseDuration = iTable[13]; // �Ƚ�����������
#endif
					}
					else
					{
#ifdef SINGLE_VALVE

						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].Para_1 = iTable[11];
						gsCode[iIndex].Para_2 = iTable[12];
						gsCode[iIndex].Para_3 = iTable[13];
						gsCode[iIndex].Para_4 = iTable[14];
						gsCode[iIndex].Para_5 = iTable[18];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[15];
							gsCode[iIndex].Deceleration = iTable[15];
							gsCode[iIndex].Jerk = iTable[16];
						}

#endif

#ifdef SINGLE_VALVE_3D
						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].XAUXPosition = iTable[8];
						gsCode[iIndex].YAUXPosition = iTable[9];
						gsCode[iIndex].ZAUXPosition = iTable[10];
						gsCode[iIndex].Para_1 = iTable[11];
						gsCode[iIndex].Para_2 = iTable[12];
						gsCode[iIndex].Para_3 = iTable[13];
						gsCode[iIndex].Para_4 = iTable[14];
						gsCode[iIndex].Para_5 = iTable[18];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[15];
							gsCode[iIndex].Deceleration = iTable[15];
							gsCode[iIndex].Jerk = iTable[16];
						}
#endif

#ifdef SINGLE_VALVE_3DR
						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].XAUXPosition = iTable[8];
						gsCode[iIndex].YAUXPosition = iTable[9];
						gsCode[iIndex].ZAUXPosition = iTable[10];
						gsCode[iIndex].UPosition = iTable[11];
						gsCode[iIndex].Para_1 = iTable[12];
						gsCode[iIndex].Para_2 = iTable[13];
						gsCode[iIndex].Para_3 = iTable[14];
						gsCode[iIndex].Para_4 = iTable[15];
						gsCode[iIndex].Para_5 = iTable[18];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[16];
							gsCode[iIndex].Deceleration = iTable[16];
							gsCode[iIndex].Jerk = iTable[17];
						}
#endif

#ifdef SINGLE_VALVE_3D2R
						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].XAUXPosition = iTable[8];
						gsCode[iIndex].YAUXPosition = iTable[9];
						gsCode[iIndex].ZAUXPosition = iTable[10];
						gsCode[iIndex].N1Position = iTable[11];
						gsCode[iIndex].N2Position = iTable[12];
						gsCode[iIndex].Para_1 = iTable[13];
						gsCode[iIndex].Para_2 = iTable[14];
						gsCode[iIndex].Para_3 = iTable[15];
						gsCode[iIndex].Para_4 = iTable[16];
						gsCode[iIndex].Para_5 = iTable[19];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[17];
							gsCode[iIndex].Deceleration = iTable[17];
							gsCode[iIndex].Jerk = iTable[18];
						}
#endif

#ifdef SINGLE_VALVE_R
						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].N1Position = iTable[8];
						gsCode[iIndex].N2Position = iTable[9];
						gsCode[iIndex].Para_1 = iTable[11];
						gsCode[iIndex].Para_2 = iTable[12];
						gsCode[iIndex].Para_3 = iTable[13];
						gsCode[iIndex].Para_4 = iTable[14];
						gsCode[iIndex].Para_5 = iTable[18];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[15];
							gsCode[iIndex].Deceleration = iTable[15];
							gsCode[iIndex].Jerk = iTable[16];
						}
#endif

#ifdef DOUBLE_VALVE
						gsCode[iIndex].Velocity = iTable[4];
						gsCode[iIndex].XPosition = iTable[5];
						gsCode[iIndex].YPosition = iTable[6];
						gsCode[iIndex].ZPosition = iTable[7];
						gsCode[iIndex].X1Position = iTable[8];
						gsCode[iIndex].Y1Position = iTable[9];
						gsCode[iIndex].Z1Position = iTable[10];
						gsCode[iIndex].Para_1 = iTable[11];
						gsCode[iIndex].Para_2 = iTable[12];
						gsCode[iIndex].Para_3 = iTable[13];
						gsCode[iIndex].Para_4 = iTable[14];
						gsCode[iIndex].Para_5 = iTable[18];
						if (gsCode[iIndex].Para_3 == 50)
						{
							gsCode[iIndex].Acceleration = iTable[15];
							gsCode[iIndex].Deceleration = iTable[15];
							gsCode[iIndex].Jerk = iTable[16];
						}
#endif
					}

					iIndex++;
					giTotal[iGroup] = iIndex;
				}
				else
					break;
			}

			fp[iGroup].close();

			sGroup[iGroup].sSubState = eSubState_Dispensing_Init;

			break;

		case eSubState_Dispensing_Init:

			cVector[iGroup].GroupEnable();
			cVector[iGroup].SetKinTransform(stSetKinTransform[iGroup]);

			stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
			rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

			// try
			// {

				GroupOfPrmWrt[0].eParameterNumber = MMC_I_COMM_EV_USR_1_PARAM; // Clear Pause Flag
				GroupOfPrmWrt[0].iParameterIndex = 0;
				GroupOfPrmWrt[0].usPadding1 = 0;
				GroupOfPrmWrt[0].usPadding2 = 0;
				GroupOfPrmWrt[0].usPadding3 = 0;

				GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();
				GroupOfPrmWrt[0].dbValue = 0;

				GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Clear Process ID
				GroupOfPrmWrt[1].iParameterIndex = 0;
				GroupOfPrmWrt[1].usPadding1 = 0;
				GroupOfPrmWrt[1].usPadding2 = 0;
				GroupOfPrmWrt[1].usPadding3 = 0;

				GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
				GroupOfPrmWrt[1].dbValue = 0;

				GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Clear Unit ID
				GroupOfPrmWrt[2].iParameterIndex = 0;
				GroupOfPrmWrt[2].usPadding1 = 0;
				GroupOfPrmWrt[2].usPadding2 = 0;
				GroupOfPrmWrt[2].usPadding3 = 0;

				GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();
				GroupOfPrmWrt[2].dbValue = 0;

				GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Clear Line ID
				GroupOfPrmWrt[3].iParameterIndex = 0;
				GroupOfPrmWrt[3].usPadding1 = 0;
				GroupOfPrmWrt[3].usPadding2 = 0;
				GroupOfPrmWrt[3].usPadding3 = 0;

				GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();
				GroupOfPrmWrt[3].dbValue = 0;

				cVector[iGroup].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 4);
			// }
			// catch (CMMCException &ex)
			// {
			// 	printf("AxisRef: %d, ERROR: %s error %d, status %d\n", cAxes[eX].GetRef(), ex.what(), (short)ex.error(), ex.status());
			// }

			iIndex = 0;
			giFirstPush = 0;
			sGroup[iGroup].sSubState = eSubState_Dispensing_WInit;

			break;
		case eSubState_Dispensing_WInit:

			if ((giGroupStatus[iGroup] & NC_GROUP_STANDBY_MASK))
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
		case eSubState_Dispensing_Start:

			if (giTotal[iGroup] == 0)
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_End;
			}
			else
			{

				if (gsCode[iIndex].FunCode == 0) // �㶨λ����ʱ�㽺
				{

					GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[0].iParameterIndex = 0;
					GroupOfPrmWrt[0].usPadding1 = 0;
					GroupOfPrmWrt[0].usPadding2 = 0;
					GroupOfPrmWrt[0].usPadding3 = 0;

					GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

					GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16); // DO_1:Low Level;DO_3:High Level

					GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Process ID
					GroupOfPrmWrt[1].iParameterIndex = 0;
					GroupOfPrmWrt[1].usPadding1 = 0;
					GroupOfPrmWrt[1].usPadding2 = 0;
					GroupOfPrmWrt[1].usPadding3 = 0;

					GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
					GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

					GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Unit ID
					GroupOfPrmWrt[2].iParameterIndex = 0;
					GroupOfPrmWrt[2].usPadding1 = 0;
					GroupOfPrmWrt[2].usPadding2 = 0;
					GroupOfPrmWrt[2].usPadding3 = 0;

					GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

					GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

					GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
					GroupOfPrmWrt[3].iParameterIndex = 0;
					GroupOfPrmWrt[3].usPadding1 = 0;
					GroupOfPrmWrt[3].usPadding2 = 0;
					GroupOfPrmWrt[3].usPadding3 = 0;

					GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

					GroupOfPrmWrt[3].dbValue = (double)iIndex;

					GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[4].iParameterIndex = 0;
					GroupOfPrmWrt[4].usPadding1 = 0;
					GroupOfPrmWrt[4].usPadding2 = 0;
					GroupOfPrmWrt[4].usPadding3 = 0;

					GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

					GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

					cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 5);

					cVector[iGroup].Dwell((unsigned long)gsCode[iIndex].Para_1);

					GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[0].iParameterIndex = 0;
					GroupOfPrmWrt[0].usPadding1 = 0;
					GroupOfPrmWrt[0].usPadding2 = 0;
					GroupOfPrmWrt[0].usPadding3 = 0;

					GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

					GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

					GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
					GroupOfPrmWrt[1].iParameterIndex = 0;
					GroupOfPrmWrt[1].usPadding1 = 0;
					GroupOfPrmWrt[1].usPadding2 = 0;
					GroupOfPrmWrt[1].usPadding3 = 0;

					GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

					GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

					cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);

					iIndex++;
					giTotal[iGroup]--;
				}
				else if (gsCode[iIndex].FunCode == 1) // ֱ�߶����㽺
				{

					cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
					cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
					cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
					cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

#ifdef SINGLE_VALVE
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
#endif

#ifdef SINGLE_VALVE_R
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
#endif

#ifdef SINGLE_VALVE_3D
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

#ifdef SINGLE_VALVE_3DR
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

#ifdef SINGLE_VALVE_3D2R
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

#ifdef DOUBLE_VALVE
					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_N1_AXIS_TYPE] = (double)gsCode[iIndex].X1Position;
					cVector[iGroup].m_dEndPoint[NC_N2_AXIS_TYPE] = (double)gsCode[iIndex].Y1Position;
#endif

					if (gsCode[iIndex].Para_3 == 0)
					{

						// try
						// {

							if ((gsOCIsInProcess == 1) || (giGlobalPause == 0))
							{
								GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[0].iParameterIndex = 0;
								GroupOfPrmWrt[0].usPadding1 = 0;
								GroupOfPrmWrt[0].usPadding2 = 0;
								GroupOfPrmWrt[0].usPadding3 = 0;

								GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16);

								GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Process ID
								GroupOfPrmWrt[1].iParameterIndex = 0;
								GroupOfPrmWrt[1].usPadding1 = 0;
								GroupOfPrmWrt[1].usPadding2 = 0;
								GroupOfPrmWrt[1].usPadding3 = 0;

								GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
								GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

								GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Unit ID
								GroupOfPrmWrt[2].iParameterIndex = 0;
								GroupOfPrmWrt[2].usPadding1 = 0;
								GroupOfPrmWrt[2].usPadding2 = 0;
								GroupOfPrmWrt[2].usPadding3 = 0;

								GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

								GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
								GroupOfPrmWrt[3].iParameterIndex = 0;
								GroupOfPrmWrt[3].usPadding1 = 0;
								GroupOfPrmWrt[3].usPadding2 = 0;
								GroupOfPrmWrt[3].usPadding3 = 0;

								GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[3].dbValue = (double)iIndex;

								GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[4].iParameterIndex = 0;
								GroupOfPrmWrt[4].usPadding1 = 0;
								GroupOfPrmWrt[4].usPadding2 = 0;
								GroupOfPrmWrt[4].usPadding3 = 0;

								GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

								GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

								cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 5);

								cVector[iGroup].m_ucExecute = 1;
								cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
								cVector[iGroup].m_fTransitionParameter[0] = 0;
								cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);

								/************************************************************************/
								GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[0].iParameterIndex = 0;
								GroupOfPrmWrt[0].usPadding1 = 0;
								GroupOfPrmWrt[0].usPadding2 = 0;
								GroupOfPrmWrt[0].usPadding3 = 0;

								GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

								GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[1].iParameterIndex = 0;
								GroupOfPrmWrt[1].usPadding1 = 0;
								GroupOfPrmWrt[1].usPadding2 = 0;
								GroupOfPrmWrt[1].usPadding3 = 0;

								GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

								GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

								cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
								/************************************************************************/

								sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
							}
						// }
						// catch (CMMCException &ex)
						// {
						// 	if ((short)ex.error() == 1006)
						// 	{
						// 		iIndex++;
						// 		giTotal[iGroup]--;
						// 	}
						// 	else
						// 	{
						// 		throw ex;
						// 	}
						// }
					}
					else
					{
						GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[0].iParameterIndex = 0;
						GroupOfPrmWrt[0].usPadding1 = 0;
						GroupOfPrmWrt[0].usPadding2 = 0;
						GroupOfPrmWrt[0].usPadding3 = 0;

						GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16);

						GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Process ID
						GroupOfPrmWrt[1].iParameterIndex = 0;
						GroupOfPrmWrt[1].usPadding1 = 0;
						GroupOfPrmWrt[1].usPadding2 = 0;
						GroupOfPrmWrt[1].usPadding3 = 0;

						GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
						GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

						GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Unit ID
						GroupOfPrmWrt[2].iParameterIndex = 0;
						GroupOfPrmWrt[2].usPadding1 = 0;
						GroupOfPrmWrt[2].usPadding2 = 0;
						GroupOfPrmWrt[2].usPadding3 = 0;

						GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

						GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
						GroupOfPrmWrt[3].iParameterIndex = 0;
						GroupOfPrmWrt[3].usPadding1 = 0;
						GroupOfPrmWrt[3].usPadding2 = 0;
						GroupOfPrmWrt[3].usPadding3 = 0;

						GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[3].dbValue = (double)iIndex;

						GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[4].iParameterIndex = 0;
						GroupOfPrmWrt[4].usPadding1 = 0;
						GroupOfPrmWrt[4].usPadding2 = 0;
						GroupOfPrmWrt[4].usPadding3 = 0;

						GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

						GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

						cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 5);

						try
						{

							if (gsCode[iIndex].Para_3 == 100)
							{

								if ((gsCode[iIndex + 1].FunCode != 1) && (gsCode[iIndex + 1].FunCode != 2))
								{
									sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
									cVector[iGroup].m_ucExecute = 1;
									cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
									cVector[iGroup].m_fTransitionParameter[0] = 100;
									cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
									/************************************************************************/
									GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[0].iParameterIndex = 0;
									GroupOfPrmWrt[0].usPadding1 = 0;
									GroupOfPrmWrt[0].usPadding2 = 0;
									GroupOfPrmWrt[0].usPadding3 = 0;

									GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

									GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

									GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[1].iParameterIndex = 0;
									GroupOfPrmWrt[1].usPadding1 = 0;
									GroupOfPrmWrt[1].usPadding2 = 0;
									GroupOfPrmWrt[1].usPadding3 = 0;

									GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

									GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

									cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);

									/************************************************************************/
								}
								else
								{
									if (gsCode[iIndex + 1].Para_3 == 0)
									{
										sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
										cVector[iGroup].m_ucExecute = 1;
										cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = 100;
										cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
										/************************************************************************/
										GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[0].iParameterIndex = 0;
										GroupOfPrmWrt[0].usPadding1 = 0;
										GroupOfPrmWrt[0].usPadding2 = 0;
										GroupOfPrmWrt[0].usPadding3 = 0;

										GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

										GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

										GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[1].iParameterIndex = 0;
										GroupOfPrmWrt[1].usPadding1 = 0;
										GroupOfPrmWrt[1].usPadding2 = 0;
										GroupOfPrmWrt[1].usPadding3 = 0;

										GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

										GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

										cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
										/************************************************************************/
									}
									else
									{
										if (giFirstPush == 0)
										{
											giFirstPush = 1;
											usleep(1000);
										}
										cVector[iGroup].m_ucExecute = 0;
										cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = 100;
										cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
										iIndex++;
										giTotal[iGroup]--;
									}
								}
							}
							else if (gsCode[iIndex].Para_3 == 50)
							{

								if ((gsCode[iIndex + 1].FunCode != 1) && (gsCode[iIndex + 1].FunCode != 2))
								{
									sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
									cVector[iGroup].m_ucExecute = 1;
									cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
									cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
									cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
									cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
									cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
									cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
									cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
									/************************************************************************/
									GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[0].iParameterIndex = 0;
									GroupOfPrmWrt[0].usPadding1 = 0;
									GroupOfPrmWrt[0].usPadding2 = 0;
									GroupOfPrmWrt[0].usPadding3 = 0;

									GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

									GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

									GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[1].iParameterIndex = 0;
									GroupOfPrmWrt[1].usPadding1 = 0;
									GroupOfPrmWrt[1].usPadding2 = 0;
									GroupOfPrmWrt[1].usPadding3 = 0;

									GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

									GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

									cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
									/************************************************************************/
								}
								else
								{
									if (gsCode[iIndex + 1].Para_3 == 0)
									{
										sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
										cVector[iGroup].m_ucExecute = 1;
										cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
										cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
										cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
										cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
										cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
										cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
										/************************************************************************/
										GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[0].iParameterIndex = 0;
										GroupOfPrmWrt[0].usPadding1 = 0;
										GroupOfPrmWrt[0].usPadding2 = 0;
										GroupOfPrmWrt[0].usPadding3 = 0;

										GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

										GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

										GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[1].iParameterIndex = 0;
										GroupOfPrmWrt[1].usPadding1 = 0;
										GroupOfPrmWrt[1].usPadding2 = 0;
										GroupOfPrmWrt[1].usPadding3 = 0;

										GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

										GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

										cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
										/************************************************************************/
									}
									else
									{
										if (giFirstPush == 0)
										{
											giFirstPush = 1;
											usleep(1000);
										}
										cVector[iGroup].m_ucExecute = 0;
										cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
										cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
										cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
										cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
										cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
										cVector[iGroup].MoveLinearAbsolute(MC_BLENDING_HIGH_MODE);
										iIndex++;
										giTotal[iGroup]--;
									}
								}
							}
						}
						catch (CMMCException &ex)
						{
							if ((short)ex.error() == 1006)
							{
								iIndex++;
								giTotal[iGroup]--;
							}
							else
							{
								throw ex;
							}
						}
					}

					//	iIndex++;
					//	giTotal[iGroup]--;
				}
				else if (gsCode[iIndex].FunCode == 2) // Բ�������㽺
				{

					cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
					cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
					cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
					cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

#ifdef DOUBLE_VALVE
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_N1_AXIS_TYPE] = (double)gsCode[iIndex].X1Position;
					cVector[iGroup].m_dEndPoint[NC_N2_AXIS_TYPE] = (double)gsCode[iIndex].Y1Position;
#endif

#ifdef SINGLE_VALVE
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
#endif

#ifdef SINGLE_VALVE_R
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
#endif

#ifdef SINGLE_VALVE_3D
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZAUXPosition;

					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

#ifdef SINGLE_VALVE_3DR
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZAUXPosition;

					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

#ifdef SINGLE_VALVE_3D2R
					cVector[iGroup].m_dAuxPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YAUXPosition;
					cVector[iGroup].m_dAuxPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZAUXPosition;

					cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsCode[iIndex].XPosition;
					cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsCode[iIndex].YPosition;
					cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)gsCode[iIndex].ZPosition;
#endif

					if (gsCode[iIndex].Para_3 == 0)
					{
						try
						{

							if ((gsOCIsInProcess == 1) || (giGlobalPause == 0))
							{
								GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[0].iParameterIndex = 0;
								GroupOfPrmWrt[0].usPadding1 = 0;
								GroupOfPrmWrt[0].usPadding2 = 0;
								GroupOfPrmWrt[0].usPadding3 = 0;

								GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16);

								GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Process ID
								GroupOfPrmWrt[1].iParameterIndex = 0;
								GroupOfPrmWrt[1].usPadding1 = 0;
								GroupOfPrmWrt[1].usPadding2 = 0;
								GroupOfPrmWrt[1].usPadding3 = 0;

								GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
								GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

								GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Unit ID
								GroupOfPrmWrt[2].iParameterIndex = 0;
								GroupOfPrmWrt[2].usPadding1 = 0;
								GroupOfPrmWrt[2].usPadding2 = 0;
								GroupOfPrmWrt[2].usPadding3 = 0;

								GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

								GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
								GroupOfPrmWrt[3].iParameterIndex = 0;
								GroupOfPrmWrt[3].usPadding1 = 0;
								GroupOfPrmWrt[3].usPadding2 = 0;
								GroupOfPrmWrt[3].usPadding3 = 0;

								GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[3].dbValue = (double)iIndex;

								GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[4].iParameterIndex = 0;
								GroupOfPrmWrt[4].usPadding1 = 0;
								GroupOfPrmWrt[4].usPadding2 = 0;
								GroupOfPrmWrt[4].usPadding3 = 0;

								GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

								GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

								cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 5);

								cVector[iGroup].m_ucExecute = 1;
								cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
								cVector[iGroup].m_fTransitionParameter[0] = 0;

#ifdef SINGLE_VALVE_3D
								cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
								cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
								cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE
								cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BUFFERED_MODE);
#endif

#ifdef SINGLE_VALVE_R
								cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BUFFERED_MODE);
#endif

#ifdef DOUBLE_VALVE
								cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BUFFERED_MODE);
#endif

								/************************************************************************/
								GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[0].iParameterIndex = 0;
								GroupOfPrmWrt[0].usPadding1 = 0;
								GroupOfPrmWrt[0].usPadding2 = 0;
								GroupOfPrmWrt[0].usPadding3 = 0;

								GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

								GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

								GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
								GroupOfPrmWrt[1].iParameterIndex = 0;
								GroupOfPrmWrt[1].usPadding1 = 0;
								GroupOfPrmWrt[1].usPadding2 = 0;
								GroupOfPrmWrt[1].usPadding3 = 0;

								GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

								GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

								cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
								/************************************************************************/

								sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
							}
						}
						catch (CMMCException &ex)
						{
							if ((short)ex.error() == 1007)
							{
								iIndex++;
								giTotal[iGroup]--;
							}
							else
							{
								throw ex;
							}
						}
					}
					else
					{

						GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[0].iParameterIndex = 0;
						GroupOfPrmWrt[0].usPadding1 = 0;
						GroupOfPrmWrt[0].usPadding2 = 0;
						GroupOfPrmWrt[0].usPadding3 = 0;

						GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16);

						GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Process ID
						GroupOfPrmWrt[1].iParameterIndex = 0;
						GroupOfPrmWrt[1].usPadding1 = 0;
						GroupOfPrmWrt[1].usPadding2 = 0;
						GroupOfPrmWrt[1].usPadding3 = 0;

						GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
						GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

						GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Unit ID
						GroupOfPrmWrt[2].iParameterIndex = 0;
						GroupOfPrmWrt[2].usPadding1 = 0;
						GroupOfPrmWrt[2].usPadding2 = 0;
						GroupOfPrmWrt[2].usPadding3 = 0;

						GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

						GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
						GroupOfPrmWrt[3].iParameterIndex = 0;
						GroupOfPrmWrt[3].usPadding1 = 0;
						GroupOfPrmWrt[3].usPadding2 = 0;
						GroupOfPrmWrt[3].usPadding3 = 0;

						GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[3].dbValue = (double)iIndex;

						GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[4].iParameterIndex = 0;
						GroupOfPrmWrt[4].usPadding1 = 0;
						GroupOfPrmWrt[4].usPadding2 = 0;
						GroupOfPrmWrt[4].usPadding3 = 0;

						GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

						GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

						cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 5);

						try
						{
							if (gsCode[iIndex].Para_3 == 100)
							{
								if ((gsCode[iIndex + 1].FunCode != 1) && (gsCode[iIndex + 1].FunCode != 2))
								{
									sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
									cVector[iGroup].m_ucExecute = 1;
									cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
									cVector[iGroup].m_fTransitionParameter[0] = 100;
#ifdef SINGLE_VALVE_3D
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
									/************************************************************************/
									GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[0].iParameterIndex = 0;
									GroupOfPrmWrt[0].usPadding1 = 0;
									GroupOfPrmWrt[0].usPadding2 = 0;
									GroupOfPrmWrt[0].usPadding3 = 0;

									GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

									GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

									GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[1].iParameterIndex = 0;
									GroupOfPrmWrt[1].usPadding1 = 0;
									GroupOfPrmWrt[1].usPadding2 = 0;
									GroupOfPrmWrt[1].usPadding3 = 0;

									GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

									GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

									cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
									/************************************************************************/
								}
								else
								{
									if (gsCode[iIndex + 1].Para_3 == 0)
									{
										sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
										cVector[iGroup].m_ucExecute = 1;
										cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = 100;
#ifdef SINGLE_VALVE_3D
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
										/************************************************************************/
										GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[0].iParameterIndex = 0;
										GroupOfPrmWrt[0].usPadding1 = 0;
										GroupOfPrmWrt[0].usPadding2 = 0;
										GroupOfPrmWrt[0].usPadding3 = 0;

										GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

										GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

										GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[1].iParameterIndex = 0;
										GroupOfPrmWrt[1].usPadding1 = 0;
										GroupOfPrmWrt[1].usPadding2 = 0;
										GroupOfPrmWrt[1].usPadding3 = 0;

										GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

										GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

										cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
										/************************************************************************/
									}
									else
									{
										if (giFirstPush == 0)
										{
											giFirstPush = 1;
											usleep(1000);
										}
										cVector[iGroup].m_ucExecute = 0;
										cVector[iGroup].m_eTransitionMode = MC_TM_DEFINED_VELOCITY_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = 100;
#ifdef SINGLE_VALVE_3D
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
										iIndex++;
										giTotal[iGroup]--;
									}
								}
							}
							else if (gsCode[iIndex].Para_3 == 50)
							{
								if ((gsCode[iIndex + 1].FunCode != 1) && (gsCode[iIndex + 1].FunCode != 2))
								{
									sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
									cVector[iGroup].m_ucExecute = 1;
									cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
									cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
									cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
									cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
									cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
									cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
#ifdef SINGLE_VALVE_3D
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
									cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
									cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
									/************************************************************************/
									GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[0].iParameterIndex = 0;
									GroupOfPrmWrt[0].usPadding1 = 0;
									GroupOfPrmWrt[0].usPadding2 = 0;
									GroupOfPrmWrt[0].usPadding3 = 0;

									GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

									GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

									GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
									GroupOfPrmWrt[1].iParameterIndex = 0;
									GroupOfPrmWrt[1].usPadding1 = 0;
									GroupOfPrmWrt[1].usPadding2 = 0;
									GroupOfPrmWrt[1].usPadding3 = 0;

									GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

									GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

									cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
									/************************************************************************/
								}
								else
								{
									if (gsCode[iIndex + 1].Para_3 == 0)
									{
										sGroup[iGroup].sSubState = eSubState_Dispensing_QueueWait;
										cVector[iGroup].m_ucExecute = 1;
										cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
										cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
										cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
										cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
										cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
#ifdef SINGLE_VALVE_3D
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
										/************************************************************************/
										GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[0].iParameterIndex = 0;
										GroupOfPrmWrt[0].usPadding1 = 0;
										GroupOfPrmWrt[0].usPadding2 = 0;
										GroupOfPrmWrt[0].usPadding3 = 0;

										GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

										GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_5 << 16);

										GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
										GroupOfPrmWrt[1].iParameterIndex = 0;
										GroupOfPrmWrt[1].usPadding1 = 0;
										GroupOfPrmWrt[1].usPadding2 = 0;
										GroupOfPrmWrt[1].usPadding3 = 0;

										GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

										GroupOfPrmWrt[1].dbValue = (double)((long)gsCode[iIndex].Para_5 << 8);

										cVector[iGroup].WriteGroupOfParametersQueued(GroupOfPrmWrt, 2);
										/************************************************************************/
									}
									else
									{
										if (giFirstPush == 0)
										{
											giFirstPush = 1;
											usleep(1000);
										}
										cVector[iGroup].m_ucExecute = 0;
										cVector[iGroup].m_fVelocity = (float)gsCode[iIndex].Velocity;
										cVector[iGroup].m_fAcceleration = (float)gsCode[iIndex].Acceleration;
										cVector[iGroup].m_fDeceleration = (float)gsCode[iIndex].Deceleration;
										cVector[iGroup].m_fJerk = (float)gsCode[iIndex].Jerk;
										cVector[iGroup].m_eTransitionMode = MC_TM_CORNER_DISTANCE_MODE;
										cVector[iGroup].m_fTransitionParameter[0] = (double)gsCode[iIndex].Para_4;
#ifdef SINGLE_VALVE_3D
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3DR
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_3D2R
										cVector[iGroup].MoveCircularAbsolute(MC_NONE_ARC_CHOICE, MC_NONE_PATH_CHOICE, MC_BORDER_CIRC_MODE, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef SINGLE_VALVE_R
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif

#ifdef DOUBLE_VALVE
										cVector[iGroup].MoveCircularAbsoluteAngle((double)gsCode[iIndex].Para_1 / 100, MC_BLENDING_HIGH_MODE);
#endif
										iIndex++;
										giTotal[iGroup]--;
									}
								}
							}
						}
						catch (CMMCException &ex)
						{
							if ((short)ex.error() == 1007)
							{
								iIndex++;
								giTotal[iGroup]--;
							}
							else
							{
								throw ex;
							}
						}
					}

					//	iIndex++;
					//	giTotal[iGroup]--;
				}
				else if (gsCode[iIndex].FunCode == 3) // Z�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_ZMove;
					}
				}
				else if (gsCode[iIndex].FunCode == 4) // DO���1
				{

					if (giGlobalPause == 0)
					{
						GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[0].iParameterIndex = 0;
						GroupOfPrmWrt[0].usPadding1 = 0;
						GroupOfPrmWrt[0].usPadding2 = 0;
						GroupOfPrmWrt[0].usPadding3 = 0;

						GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[0].dbValue = (double)((long)gsCode[iIndex].Para_2 << 16);

						GroupOfPrmWrt[1].eParameterNumber = MMC_I_COMM_EV_USR_2_PARAM; // Command code
						GroupOfPrmWrt[1].iParameterIndex = 0;
						GroupOfPrmWrt[1].usPadding1 = 0;
						GroupOfPrmWrt[1].usPadding2 = 0;
						GroupOfPrmWrt[1].usPadding3 = 0;

						GroupOfPrmWrt[1].usAxisRef = cAxes[eX].GetRef();
						GroupOfPrmWrt[1].dbValue = (double)gsCode[iIndex].ProcessID;

						GroupOfPrmWrt[2].eParameterNumber = MMC_I_COMM_EV_USR_3_PARAM; // Line code
						GroupOfPrmWrt[2].iParameterIndex = 0;
						GroupOfPrmWrt[2].usPadding1 = 0;
						GroupOfPrmWrt[2].usPadding2 = 0;
						GroupOfPrmWrt[2].usPadding3 = 0;

						GroupOfPrmWrt[2].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[2].dbValue = (double)gsCode[iIndex].UnitID;

						GroupOfPrmWrt[3].eParameterNumber = MMC_I_COMM_EV_USR_4_PARAM; // Line ID
						GroupOfPrmWrt[3].iParameterIndex = 0;
						GroupOfPrmWrt[3].usPadding1 = 0;
						GroupOfPrmWrt[3].usPadding2 = 0;
						GroupOfPrmWrt[3].usPadding3 = 0;

						GroupOfPrmWrt[3].usAxisRef = cAxes[eX].GetRef();

						GroupOfPrmWrt[3].dbValue = (double)iIndex;

						GroupOfPrmWrt[4].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
						GroupOfPrmWrt[4].iParameterIndex = 0;
						GroupOfPrmWrt[4].usPadding1 = 0;
						GroupOfPrmWrt[4].usPadding2 = 0;
						GroupOfPrmWrt[4].usPadding3 = 0;

						GroupOfPrmWrt[4].usAxisRef = cAxes[eY].GetRef();

						GroupOfPrmWrt[4].dbValue = (double)((long)gsCode[iIndex].Para_2 << 8);

						cVector[iGroup].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 5);

						if (gsCode[iIndex].Para_3 == 0)
						{
							iIndex++;
							giTotal[iGroup]--;
						}
						else
						{
							iDIStartTime[iGroup] = PmasTimeUpdate();
							sGroup[iGroup].sSubState = eSubState_Dispensing_DIWait;
						}
					}
				}
				else if (gsCode[iIndex].FunCode == 5) // �ȴ���ʱ
				{
					cVector[iGroup].Dwell((unsigned long)gsCode[iIndex].Para_1);

					iIndex++;
					giTotal[iGroup]--;
				}
				else if (gsCode[iIndex].FunCode == 6) // ��������
				{
					if (giGlobalPause == 0)
					{
						gsOCIsInProcess = 1;
						sGroup[iGroup].sSubState = eSubState_Dispensing_StartVisionOC;
					}
				}
				else if (gsCode[iIndex].FunCode == 7) // ֹͣ����
				{
					sGroup[iGroup].sSubState = eSubState_Dispensing_StopVisionOC;
				}
				else if (gsCode[iIndex].FunCode == 8) // ����������
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_ValveChg;
					}
				}
#ifdef DOUBLE_VALVE
				else if (gsCode[iIndex].FunCode == 9) // Z1�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_Z1Move;
					}
				}
#endif

#ifdef SINGLE_VALVE_R
				else if (gsCode[iIndex].FunCode == 10) // N1�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_N1Move;
					}
				}
				else if (gsCode[iIndex].FunCode == 11) // N2�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_N2Move;
					}
				}
#endif

#ifdef SINGLE_VALVE_3D2R
				else if (gsCode[iIndex].FunCode == 10) // N1�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_N1Move;
					}
				}
				else if (gsCode[iIndex].FunCode == 11) // N2�ᶨλ
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_N2Move;
					}
				}
#endif
				else if (gsCode[iIndex].FunCode == 12) // ��������㽺
				{
					if (giGlobalPause == 0)
					{
						gsOCIsInProcess = 1;
						sGroup[iGroup].sSubState = eSubState_Dispensing_StartValveOC;
					}
				}
				else if (gsCode[iIndex].FunCode == 13) // ֹͣ����㽺
				{
					sGroup[iGroup].sSubState = eSubState_Dispensing_StopValveOC;
				}
#ifdef SINGLE_VALVE_3DR
				else if (gsCode[iIndex].FunCode == 14) // 3DR������U���ƶ�
				{
					if (giGlobalPause == 0)
					{
						sGroup[iGroup].sSubState = eSubState_Dispensing_UMove;
					}
				}
#endif
				else if (gsCode[iIndex].FunCode == 15) // ������ʱ
				{
					sGroup[iGroup].sSubState = eSubState_Dispensing_FunDelay;
				}

				if ((giGlobalPause == 1) && (gsOCIsInProcess == 0))
				{
					sGroup[iGroup].sSubState = eSubState_Dispensing_WVPause;
				}
			}
			break;

		case eSubState_Dispensing_DIWait:

			iDIEndTime[iGroup] = PmasTimeUpdate();
			if ((unsigned int)gsCode[iIndex].Para_3 & iDIValue[eX])
			{
				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			else
			{
				if ((int)(iDIEndTime[iGroup] - iDIStartTime[iGroup]) >= 2000000)
				{
					sSingleCommands[eX].sDIErrState = MOTOR_DI_ERROR;
					sSingleCommands[eX].cInErrorDriveOrGmas = eGMASErr;
					cVector[iGroup].GroupDisable();
					sGroup[iGroup].sSubState = eIDLE;
				}
			}

			break;

		case eSubState_Dispensing_QueueWait:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{
				giFirstPush = 0;
				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;

		case eSubState_Dispensing_ZMove:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_ucExecute = 1;

				try
				{
					cAxes[eZ].MoveAbsolute((double)gsCode[iIndex].ZPosition, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
					;
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_ZWait;
			}

			break;
		case eSubState_Dispensing_ZWait:

			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{

				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;

#ifdef SINGLE_VALVE_3DR

		case eSubState_Dispensing_UMove:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eU].m_fAcceleration = (float)sSingleCommands[eU].lAcceleration;
				cAxes[eU].m_fDeceleration = (float)sSingleCommands[eU].lDeceleration;
				cAxes[eU].m_fJerk = (float)sSingleCommands[eU].lJerk;
				cAxes[eU].m_eDirection = sSingleCommands[eU].eDirection;
				cAxes[eU].m_ucExecute = 1;

				try
				{
					cAxes[eU].MoveAbsolute((double)gsCode[iIndex].UPosition, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
					;
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_UWait;
			}

			break;
		case eSubState_Dispensing_UWait:

			if (giStatus[eU] & NC_AXIS_STAND_STILL_MASK)
			{

				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

		case eSubState_Dispensing_StartVisionOC:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				gsOC.Points = gsCode[iIndex].Points;			   // �Ƚ��������
				gsOC.PulseDuration = gsCode[iIndex].PulseDuration; // �Ƚ�����������
				gsOC.Direction = gsCode[iIndex].Direction;		   // �Ƚ��������
				for (int m = 0; m < MAX_OC_POINTS; m++)
				{
					gsOC.Position[m] = gsCode[iIndex].OCPosition[m]; // �Ƚ����λ������
				}

				sStartVisionOCStep[eX] = 1;
				sGroup[iGroup].sSubState = eSubState_Dispensing_WStartVisionOC;
			}
			break;
		case eSubState_Dispensing_WStartVisionOC:
			StartVisionOCInProcess(eX);
			if (sSingleCommands[eX].sOCState == MOTOR_OC_IS_ENABLED)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			else if (sSingleCommands[eX].sOCState == MOTOR_OC_ERROR)
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_OCInError;
			}

			break;
		case eSubState_Dispensing_StopVisionOC:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				if (cAxes[eX].SendSdoUpload(0x01, 4, 0x316E, 1) == -1)
				{
					sStopVisionOCStep[eX] = 1;
					sGroup[iGroup].sSubState = eSubState_Dispensing_WStopVisionOC;
				}
				else if (sSingleCommands[eX].sOCState == MOTOR_OC_ERROR)
				{
					sGroup[iGroup].sSubState = eSubState_Dispensing_OCInError;
				}
			}
			break;
		case eSubState_Dispensing_WStopVisionOC:
			StopVisionOCInProcess(eX);
			if (sSingleCommands[eX].sOCState == MOTOR_OC_IS_DISABLED)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				gsOCIsInProcess = 0;
				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			else if (sSingleCommands[eX].sOCState == MOTOR_OC_ERROR)
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_OCInError;
			}

			break;

		case eSubState_Dispensing_StartValveOC:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				gsOC.StartPosition = gsCode[iIndex].StartPosition; // �Ƚ������ʼλ��
				gsOC.PulseDuration = gsCode[iIndex].PulseDuration; // �Ƚ�����������
				gsOC.DeltaPosition = gsCode[iIndex].DeltaPosition; // �Ƚ�������
				if (gsOC.ValveOCInitFlag == 1)
					sStartValveOCStep[eY] = 1;
				else
					sValveOCInitStep[eY] = 1;

				sGroup[iGroup].sSubState = eSubState_Dispensing_WStartValveOC;
			}
			break;
		case eSubState_Dispensing_WStartValveOC:
			if (gsOC.ValveOCInitFlag == 1)
				StartValveOCInProcess(eY, iGroup);
			else
				ValveOCInInit(eY, iGroup);
			if (sSingleCommands[eY].sOCState == MOTOR_OC_IS_ENABLED)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			else if (sSingleCommands[eY].sOCState == MOTOR_OC_ERROR)
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_OCInError;
			}

			break;
		case eSubState_Dispensing_StopValveOC:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{
				sStopValveOCStep[eY] = 1;
				sGroup[iGroup].sSubState = eSubState_Dispensing_WStopValveOC;
			}
			break;
		case eSubState_Dispensing_WStopValveOC:
			StopValveOCInProcess(eY);
			if (sSingleCommands[eY].sOCState == MOTOR_OC_IS_DISABLED)
			{

				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				gsOCIsInProcess = 0;
				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			else if (sSingleCommands[eY].sOCState == MOTOR_OC_ERROR)
			{
				sGroup[iGroup].sSubState = eSubState_Dispensing_OCInError;
			}

			break;

		case eSubState_Dispensing_ValveChg:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{
				sGroup[iGroup].sValveChgCode = (short)gsCode[iIndex].Para_1;
				gsGroupInValveChg[iGroup] = 1;
				sGroup[iGroup].sSubState = eSubState_Dispensing_ValveAck;
			}
			break;

		case eSubState_Dispensing_ValveAck:

			if (sGroup[iGroup].sValveChgCode == sGroup[iGroup].sValveAckCode)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;
				gsGroupInValveChg[iGroup] = 0;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;

#ifdef DOUBLE_VALVE
		case eSubState_Dispensing_Z1Move:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{
				cAxes[eZ1].m_fAcceleration = (float)sSingleCommands[eZ1].lAcceleration;
				cAxes[eZ1].m_fDeceleration = (float)sSingleCommands[eZ1].lDeceleration;
				cAxes[eZ1].m_fJerk = (float)sSingleCommands[eZ1].lJerk;
				cAxes[eZ1].m_eDirection = sSingleCommands[eZ1].eDirection;
				cAxes[eZ1].m_ucExecute = 1;

				try
				{
					cAxes[eZ1].MoveAbsolute((double)gsCode[iIndex].Z1Position, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
					;
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_Z1Wait;
			}

			break;
		case eSubState_Dispensing_Z1Wait:

			if (giStatus[eZ1] & NC_AXIS_STAND_STILL_MASK)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;
#endif

#ifdef SINGLE_VALVE_R
		case eSubState_Dispensing_N1Move:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eN1].m_fAcceleration = (float)sSingleCommands[eN1].lAcceleration;
				cAxes[eN1].m_fDeceleration = (float)sSingleCommands[eN1].lDeceleration;
				cAxes[eN1].m_fJerk = (float)sSingleCommands[eN1].lJerk;
				cAxes[eN1].m_eDirection = sSingleCommands[eN1].eDirection;
				cAxes[eN1].m_ucExecute = 1;

				try
				{
					cAxes[eN1].MoveAbsolute((double)gsCode[iIndex].N1Position, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_N1Wait;
			}

			break;
		case eSubState_Dispensing_N1Wait:

			if (giStatus[eN1] & NC_AXIS_STAND_STILL_MASK)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;

		case eSubState_Dispensing_N2Move:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eN2].m_fAcceleration = (float)sSingleCommands[eN2].lAcceleration;
				cAxes[eN2].m_fDeceleration = (float)sSingleCommands[eN2].lDeceleration;
				cAxes[eN2].m_fJerk = (float)sSingleCommands[eN2].lJerk;
				cAxes[eN2].m_eDirection = sSingleCommands[eN2].eDirection;
				cAxes[eN2].m_ucExecute = 1;

				try
				{
					cAxes[eN2].MoveAbsolute((double)gsCode[iIndex].N2Position, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_N2Wait;
			}

			break;
		case eSubState_Dispensing_N2Wait:

			if (giStatus[eN2] & NC_AXIS_STAND_STILL_MASK)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;
#endif

#ifdef SINGLE_VALVE_3D2R
		case eSubState_Dispensing_N1Move:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eN1].m_fAcceleration = (float)sSingleCommands[eN1].lAcceleration;
				cAxes[eN1].m_fDeceleration = (float)sSingleCommands[eN1].lDeceleration;
				cAxes[eN1].m_fJerk = (float)sSingleCommands[eN1].lJerk;
				cAxes[eN1].m_eDirection = sSingleCommands[eN1].eDirection;
				cAxes[eN1].m_ucExecute = 1;

				try
				{
					cAxes[eN1].MoveAbsolute((double)gsCode[iIndex].N1Position, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_N1Wait;
			}

			break;
		case eSubState_Dispensing_N1Wait:

			if (giStatus[eN1] & NC_AXIS_STAND_STILL_MASK)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;

		case eSubState_Dispensing_N2Move:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{

				cAxes[eN2].m_fAcceleration = (float)sSingleCommands[eN2].lAcceleration;
				cAxes[eN2].m_fDeceleration = (float)sSingleCommands[eN2].lDeceleration;
				cAxes[eN2].m_fJerk = (float)sSingleCommands[eN2].lJerk;
				cAxes[eN2].m_eDirection = sSingleCommands[eN2].eDirection;
				cAxes[eN2].m_ucExecute = 1;

				try
				{
					cAxes[eN2].MoveAbsolute((double)gsCode[iIndex].N2Position, (float)gsCode[iIndex].Velocity, MC_BUFFERED_MODE);
				}
				catch (CMMCException &ex)
				{
					if ((short)ex.error() == 1009)
					{
						;
					}
					else
					{
						throw ex;
					}
				}

				sGroup[iGroup].sSubState = eSubState_Dispensing_N2Wait;
			}

			break;
		case eSubState_Dispensing_N2Wait:

			if (giStatus[eN2] & NC_AXIS_STAND_STILL_MASK)
			{
				cAxes[eX].SetBoolParameter((int)iIndex, MMC_I_COMM_EV_USR_4_PARAM, 0);

				iIndex++;
				giTotal[iGroup]--;

				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;
#endif

		case eSubState_Dispensing_FunDelay:

			iTimeStart = PmasTimeUpdate();

			sGroup[iGroup].sSubState = eSubState_Dispensing_WFunDelay;

			break;

		case eSubState_Dispensing_WFunDelay:

			iTimeEnd = PmasTimeUpdate();

			if ((int)(iTimeEnd - iTimeStart) >= gsCode[iIndex].Para_1)
			{
				iIndex++;
				giTotal[iGroup]--;
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}
			break;

		case eSubState_Dispensing_WVPause:

			GroupOfPrmWrt[0].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
			GroupOfPrmWrt[0].iParameterIndex = 0;
			GroupOfPrmWrt[0].usPadding1 = 0;
			GroupOfPrmWrt[0].usPadding2 = 0;
			GroupOfPrmWrt[0].usPadding3 = 0;

			GroupOfPrmWrt[0].usAxisRef = cAxes[eX].GetRef();

			GroupOfPrmWrt[0].dbValue = 0x0;

			GroupOfPrmWrt[1].eParameterNumber = MMC_ETHERCAT_DRV_OUTPUT;
			GroupOfPrmWrt[1].iParameterIndex = 0;
			GroupOfPrmWrt[1].usPadding1 = 0;
			GroupOfPrmWrt[1].usPadding2 = 0;
			GroupOfPrmWrt[1].usPadding3 = 0;

			GroupOfPrmWrt[1].usAxisRef = cAxes[eY].GetRef();

			GroupOfPrmWrt[1].dbValue = 0x0;

			cVector[iGroup].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 2);

			cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			cAxes[eX].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0

			usleep(5000);

			stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
			rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

			cVector[iGroup].GroupDisable();

#ifdef SINGLE_VALVE
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];
#endif

#ifdef DOUBLE_VALVE
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];
			gsPause.X1Position = giPos[eX1];
			gsPause.Y1Position = giPos[eY1];
			gsPause.Z1Position = giPos[eZ1];
#endif

#ifdef SINGLE_VALVE_R
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];
			gsPause.N1Position = giPos[eN1];
			gsPause.N2Position = giPos[eN2];

#endif

#ifdef SINGLE_VALVE_3D
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];

#endif

#ifdef SINGLE_VALVE_3DR
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];
			gsPause.N1Position = giPos[eN1];
#endif

#ifdef SINGLE_VALVE_3D2R
			gsPause.XPosition = giPos[eX];
			gsPause.YPosition = giPos[eY];
			gsPause.ZPosition = giPos[eZ];
			gsPause.N1Position = giPos[eN1];
			gsPause.N2Position = giPos[eN2];
#endif

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZPause;

			break;

		case eSubState_Dispensing_WZPause:

#ifdef SINGLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(5000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;
			}

			break;
#endif

#ifdef DOUBLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(5000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;
			}

			break;
#endif

#ifdef SINGLE_VALVE_R
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(5000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D

			sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3DR

			sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3D2R

			sGroup[iGroup].sSubState = eSubState_Dispensing_ZToSafe;

			break;
#endif

		case eSubState_Dispensing_ZToSafe:

#ifdef SINGLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

#ifdef DOUBLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_R
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3D

			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3DR

			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3D2R

			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZToSafe;

			break;
#endif

		case eSubState_Dispensing_WZToSafe:

#ifdef SINGLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

#ifdef DOUBLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

#ifdef SINGLE_VALVE_R
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				cVector[iGroup].GroupDisable();
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3DR
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				cVector[iGroup].GroupDisable();
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D2R
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				cVector[iGroup].GroupDisable();
				sGroup[iGroup].sSubState = eSubState_Dispensing_PauseIsDone;
			}

			break;
#endif

		case eSubState_Dispensing_PauseIsDone:

			sGroup[iGroup].sSubState = eIDLE;
			break;

		case eSubState_Dispensing_ZBackToSafe:
			giGlobalPause = 0;
			giFirstPush = 0;

#ifdef SINGLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;

			break;
#endif

#ifdef DOUBLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_R
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute(0, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3D

			try
			{

				stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
				rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;

			break;
#endif

#ifdef SINGLE_VALVE_3DR

			try
			{
				stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
				rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;
			break;
#endif

#ifdef SINGLE_VALVE_3D2R

			try
			{
				stMMC_CLEARFBLIST_IN[iGroup].usAxisRef = cVector[iGroup].GetRef();
				rc[iGroup] = MMC_ClearNodeFbListCmd(gConnHndl, &stMMC_CLEARFBLIST_IN[iGroup], &stMMC_CLEARFBLIST_OUT[iGroup]);

				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;
				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = 0;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToSafe;
			break;
#endif

		case eSubState_Dispensing_WZBackToSafe:

#ifdef SINGLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

#ifdef DOUBLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

#ifdef SINGLE_VALVE_R
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3DR
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D2R
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_VBackToPause;
			}

			break;
#endif

		case eSubState_Dispensing_VBackToPause:

#ifdef SINGLE_VALVE
			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;

			break;
#endif

#ifdef DOUBLE_VALVE
			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;
				cVector[iGroup].m_dEndPoint[NC_N1_AXIS_TYPE] = (double)gsPause.X1Position;
				cVector[iGroup].m_dEndPoint[NC_N2_AXIS_TYPE] = (double)gsPause.Y1Position;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_R
			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_3D

			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)giPos[eZ];

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;

			break;

#endif

#ifdef SINGLE_VALVE_3DR
			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)giPos[eZ];

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;
			break;
#endif

#ifdef SINGLE_VALVE_3D2R
			try
			{
				cVector[iGroup].GroupEnable();

				usleep(5000);

				cVector[iGroup].m_fVelocity = 1.0E+5;

				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (double)gsPause.XPosition;
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (double)gsPause.YPosition;
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (double)giPos[eZ];

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WVBackToPause;
			break;
#endif

		case eSubState_Dispensing_WVBackToPause:

			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(5000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_ZBackToPause;
			}

			break;

		case eSubState_Dispensing_ZBackToPause:

#ifdef SINGLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute((double)gsPause.ZPosition, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

#ifdef DOUBLE_VALVE
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute((double)gsPause.ZPosition, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			try
			{
				cAxes[eZ1].m_fAcceleration = (float)sSingleCommands[eZ1].lAcceleration;
				cAxes[eZ1].m_fDeceleration = (float)sSingleCommands[eZ1].lDeceleration;
				cAxes[eZ1].m_fJerk = (float)sSingleCommands[eZ1].lJerk;
				cAxes[eZ1].m_eDirection = sSingleCommands[eZ1].eDirection;
				cAxes[eZ1].m_fVelocity = 1.0E+5;
				cAxes[eZ1].m_ucExecute = 1;

				cAxes[eZ1].MoveAbsolute((double)gsPause.Z1Position, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_R
			try
			{
				cAxes[eZ].m_fAcceleration = (float)sSingleCommands[eZ].lAcceleration;
				cAxes[eZ].m_fDeceleration = (float)sSingleCommands[eZ].lDeceleration;
				cAxes[eZ].m_fJerk = (float)sSingleCommands[eZ].lJerk;
				cAxes[eZ].m_eDirection = sSingleCommands[eZ].eDirection;
				cAxes[eZ].m_fVelocity = 1.0E+5;
				cAxes[eZ].m_ucExecute = 1;

				cAxes[eZ].MoveAbsolute((double)gsPause.ZPosition, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			try
			{
				cAxes[eN1].m_fAcceleration = (float)sSingleCommands[eN1].lAcceleration;
				cAxes[eN1].m_fDeceleration = (float)sSingleCommands[eN1].lDeceleration;
				cAxes[eN1].m_fJerk = (float)sSingleCommands[eN1].lJerk;
				cAxes[eN1].m_eDirection = sSingleCommands[eN1].eDirection;
				cAxes[eN1].m_fVelocity = 1.0E+5;
				cAxes[eN1].m_ucExecute = 1;

				cAxes[eN1].MoveAbsolute((double)gsPause.N1Position, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_3D

			try
			{
				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (float)giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (float)giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (float)gsPause.ZPosition;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_3DR

			try
			{
				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (float)giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (float)giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (float)gsPause.ZPosition;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			try
			{
				cAxes[eN1].m_fAcceleration = (float)sSingleCommands[eN1].lAcceleration;
				cAxes[eN1].m_fDeceleration = (float)sSingleCommands[eN1].lDeceleration;
				cAxes[eN1].m_fJerk = (float)sSingleCommands[eN1].lJerk;
				cAxes[eN1].m_eDirection = sSingleCommands[eN1].eDirection;
				cAxes[eN1].m_fVelocity = 1.0E+5;
				cAxes[eN1].m_ucExecute = 1;

				cAxes[eN1].MoveAbsolute((double)gsPause.N1Position, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

#ifdef SINGLE_VALVE_3D2R

			try
			{
				cVector[iGroup].m_fVelocity = 1.0E+5;
				cVector[iGroup].m_fAcceleration = (float)sGroup[iGroup].lAcceleration;
				cVector[iGroup].m_fDeceleration = (float)sGroup[iGroup].lDeceleration;
				cVector[iGroup].m_fJerk = (float)sGroup[iGroup].lJerk;

				cVector[iGroup].m_dEndPoint[NC_X_AXIS_TYPE] = (float)giPos[eX];
				cVector[iGroup].m_dEndPoint[NC_Y_AXIS_TYPE] = (float)giPos[eY];
				cVector[iGroup].m_dEndPoint[NC_Z_AXIS_TYPE] = (float)gsPause.ZPosition;

				cVector[iGroup].m_ucExecute = 1;
				cVector[iGroup].m_eTransitionMode = MC_TM_NONE_MODE;
				cVector[iGroup].m_fTransitionParameter[0] = 0;
				cVector[iGroup].MoveLinearAbsolute(MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1006)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			try
			{
				cAxes[eN1].m_fAcceleration = (float)sSingleCommands[eN1].lAcceleration;
				cAxes[eN1].m_fDeceleration = (float)sSingleCommands[eN1].lDeceleration;
				cAxes[eN1].m_fJerk = (float)sSingleCommands[eN1].lJerk;
				cAxes[eN1].m_eDirection = sSingleCommands[eN1].eDirection;
				cAxes[eN1].m_fVelocity = 1.0E+5;
				cAxes[eN1].m_ucExecute = 1;

				cAxes[eN1].MoveAbsolute((double)gsPause.N1Position, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}

			try
			{
				cAxes[eN2].m_fAcceleration = (float)sSingleCommands[eN2].lAcceleration;
				cAxes[eN2].m_fDeceleration = (float)sSingleCommands[eN2].lDeceleration;
				cAxes[eN2].m_fJerk = (float)sSingleCommands[eN2].lJerk;
				cAxes[eN2].m_eDirection = sSingleCommands[eN2].eDirection;
				cAxes[eN2].m_fVelocity = 1.0E+5;
				cAxes[eN2].m_ucExecute = 1;

				cAxes[eN2].MoveAbsolute((double)gsPause.N2Position, MC_BUFFERED_MODE);
			}
			catch (CMMCException &ex)
			{
				if ((short)ex.error() == 1009)
				{
					;
				}
				else
				{
					throw ex;
				}
			}
			sGroup[iGroup].sSubState = eSubState_Dispensing_WZBackToPause;

			break;
#endif

		case eSubState_Dispensing_WZBackToPause:

#ifdef SINGLE_VALVE
			if (giStatus[eZ] & NC_AXIS_STAND_STILL_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

#ifdef DOUBLE_VALVE
			if ((giStatus[eZ] & NC_AXIS_STAND_STILL_MASK) && (giStatus[eZ1] & NC_AXIS_STAND_STILL_MASK))
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

#ifdef SINGLE_VALVE_R
			if ((giStatus[eZ] & NC_AXIS_STAND_STILL_MASK) && (giStatus[eN1] & NC_AXIS_STAND_STILL_MASK))
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D
			if (cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK)
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3DR
			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (giStatus[eN1] & NC_AXIS_STAND_STILL_MASK))
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

#ifdef SINGLE_VALVE_3D2R
			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (giStatus[eN1] & NC_AXIS_STAND_STILL_MASK) && (giStatus[eN2] & NC_AXIS_STAND_STILL_MASK))
			{
				usleep(100000);
				sGroup[iGroup].sSubState = eSubState_Dispensing_Start;
			}

			break;
#endif

			break;

		case eSubState_Dispensing_OCInError:

			gsOCIsInProcess = 0;
			sGroup[iGroup].cGmasInErr = eGMASErr;
			cVector[iGroup].GroupDisable();
			sGroup[iGroup].sSubState = eIDLE;

			break;

		case eSubState_Dispensing_End:

			if ((cVector[iGroup].GroupReadStatus() & NC_GROUP_STANDBY_MASK) && (cVector[iGroup].GetFbDepth() == 0))
			{
				cVector[iGroup].GroupDisable();
				gsDispensingInProcess[iGroup] = 0;
				gsDispensingFinished[iGroup] = 1;
				sGroup[iGroup].sSubState = eIDLE;
				iCycleEnd = PmasTimeUpdate();
				iDTime = iCycleEnd - iCycleStart;
				printf("Timer is:  %ld\n", (unsigned long int)iDTime);
			}

			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sGroup[iGroup].sState = eIDLE;
		sGroup[iGroup].cGmasInErr = eGMASErr;
		sGroup[iGroup].sLastGMASError = (short)ex.error();
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	14/09/2011																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interuprt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CallbackFunc(unsigned char *recvBuffer, short recvBufferSize, void *lpsock)
{
	// Whcih function ID was received ...
	switch (recvBuffer[1])
	{
	case ASYNC_REPLY_EVT:
		printf("ASYNC event Reply\r\n");
		break;
	case EMCY_EVT:
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
	case FB_NOTIFICATION_EVT:
		printf("FB notification Event received\r\n");
		break;
		/* This is commented as a specific event was written for this function. Once it occurs
		 * the ModbusWrite_Received will be called
			case MODBUS_WRITE_EVT:
			// TODO Update additional data to be read such as function parameters.
			// TODO Remove return 0 if you want to handle as part of callback.
			return 0;
			printf("Modbus Write Event received - Updating Outputs\r\n") ;

			break ;
		*/
	}
	//
	return 1;
}

void InsertLongVarToModbusShortArr(short *spArr, long lVal)
{
	*spArr = (short)(lVal & 0xFFFF);
	*(spArr + 1) = (short)((lVal >> 16) & 0xFFFF);
}
int OnRunTimeError(const char *msg, unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("OnRunTimeError: %s,axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	14/09/2011
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void TerminateApplication(int iSigNum)
{
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1;
	sigignore(SIGALRM);
	//
	// Wait till other threads exit properly.
	sleep(1);
	return;
}
//
// Callback Function once a Modbus message is received.;
void ModbusWrite_Received()
{
	printf("Modbus Write Received\n");
}
//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	sSingleCommands[usAxisRef].cInErrorDriveOrGmas = eEMGCYErr;
	sSingleCommands[usAxisRef].lLastDriveMotorFault = (unsigned short)sEmcyCode;
	sSingleCommands[usAxisRef].sState = eIDLE;
	printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, (unsigned short)sEmcyCode);
	for (int k = 0; k < MAX_VECTORS; k++)
	{
		if (gsDispensingInProcess[k] == 1)
		{
			sGroup[k].cGmasInErr = eGMASErr;
		}
	}
}

long GetModbusLong(unsigned short iOffset)
{
	return (mbus_read_out.regArr[iOffset + 1] << 16 & 0xFFFF0000) | (mbus_read_out.regArr[iOffset] & 0xFFFF);
}

unsigned short Input_CalcCRC_IML(int usLength, int usOffSet)
{
	int nComNum = usLength;
	char comChar[2 * usLength];

	for (int i = 0; i < nComNum; i++)
	{
		comChar[2 * i] = mbus_read_out.regArr[i + usOffSet] & 0xff;
		comChar[2 * i + 1] = (mbus_read_out.regArr[i + usOffSet] >> 8) & 0xff;
	}

	return (COMM_CrcValueCalc(comChar, sizeof(comChar)));
}

unsigned short OutPut_CalcCRC_IML(int usLength, int usOffSet)
{
	int nComNum = usLength;
	char comChar[2 * usLength];

	for (int i = 0; i < nComNum; i++)
	{
		comChar[2 * i] = mbus_write_in.regArr[i + usOffSet] & 0xff;
		comChar[2 * i + 1] = (mbus_write_in.regArr[i + usOffSet] >> 8) & 0xff;
	}

	return (COMM_CrcValueCalc(comChar, sizeof(comChar)));
}
unsigned short CalcCRC_IML(void)
{
	int nComNum = 21;
	char comChar[2 * 21];

	for (int i = 0; i < nComNum; i++)
	{
		comChar[2 * i] = mbus_read_out.regArr[i] & 0xff;
		comChar[2 * i + 1] = (mbus_read_out.regArr[i] >> 8) & 0xff;
	}

	return (COMM_CrcValueCalc(comChar, sizeof(comChar)));
}
unsigned short COMM_CrcValueCalc(const char *data, int length)
{
	unsigned short crcValue = 0xffff;
	int i;

	while (length--)
	{
		unsigned short tmp = (*data++) & 0x00ff;
		crcValue ^= tmp;
		for (i = 0; i < 8; i++)
		{
			if (crcValue & 0x0001)
			{
				crcValue = (crcValue >> 1) ^ 0xA001;
			}
			else
			{
				crcValue = crcValue >> 1;
			}
		}
	}
	return (unsigned short)(crcValue);
}

void StateFunction_VisionOC_Is_Downloading(int iAxis)
{
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case 1:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_DATA_DOWNLOADING;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			cAxes[iAxis].SendSdoDownload(0, 0x06, 4, 0x316E, 1); // OC[6]=0, sensor selection
			cAxes[iAxis].SendSdoDownload(3, 0x07, 4, 0x316E, 1); // OC[7]=3, table selection
			cAxes[iAxis].SendSdoDownload(1, 0x08, 4, 0x316E, 1); // OC[8]=1, start index
			if ((gsOC.Points >= 1) && (gsOC.Points <= 20))
			{
				cAxes[iAxis].SendSdoDownload((long)gsOC.Points, 0x09, 4, 0x316E, 1); // OC[9], end index
			}
			else if (gsOC.Points == 0)
			{
				sSingleCommands[iAxis].sSubState = 100;
				break;
			}

			if ((gsOC.PulseDuration > 1) && (gsOC.PulseDuration < 127))
			{
				gsOC.VisionPulseWidth = gsOC.PulseDuration;
			}
			else if (gsOC.PulseDuration > 127)
			{
				gsOC.VisionPulseWidth = 1.0 * gsOC.PulseDuration / 100 - 1 + 127;
			}
			else if (gsOC.PulseDuration == 0)
			{
				sSingleCommands[iAxis].sSubState = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload((long)gsOC.VisionPulseWidth, 0x04, 4, 0x316E, 1); // OC[4], pulse duration, us
			if ((gsOC.Direction == 1) || (gsOC.Direction == 2))
			{
				cAxes[iAxis].SendSdoDownload((long)gsOC.Direction, 0x0A, 4, 0x316E, 1); // OC[10], oc direction, 1: positive, 2:negative
			}
			else if (gsOC.Direction == 0)
			{
				sSingleCommands[iAxis].sSubState = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload(0, 0x0B, 4, 0x316E, 1); // OC[11]=0, convert table
			cAxes[iAxis].SendSdoDownload(2, 0x0E, 4, 0x30AA, 1); // GO[14]=2, port C, A
			cAxes[iAxis].SendSdoDownload(1, 0x0E, 4, 0x3177, 1); // OL[14]=1, positive

			ucOCIndex = 0;
			sSingleCommands[iAxis].sSubState = 2;

			break;
		case 2:

			cAxes[iAxis].SendSdoDownload((long)gsOC.Position[ucOCIndex], ucOCIndex + 1, 4, 0x307B, 1); // ET
			sSingleCommands[iAxis].sSubState = 3;

			break;
		case 3:
			ucOCIndex++;
			if (ucOCIndex == gsOC.Points)
			{
				sSingleCommands[iAxis].sSubState = 4;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 2;
			}
			break;
		case 4:
			if (cAxes[iAxis].SendSdoUpload(0x06, 4, 0x316E, 1) == 0) // OC[6]=0, sensor selection
			{
				sSingleCommands[iAxis].sSubState = 5;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 5:
			if (cAxes[iAxis].SendSdoUpload(0x07, 4, 0x316E, 1) == 3) // OC[7]=3, table selection
			{
				sSingleCommands[iAxis].sSubState = 6;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 6:
			if (cAxes[iAxis].SendSdoUpload(0x08, 4, 0x316E, 1) == 1) // OC[8]=1, start index
			{
				sSingleCommands[iAxis].sSubState = 7;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 7:
			if (cAxes[iAxis].SendSdoUpload(0x09, 4, 0x316E, 1) == (long)gsOC.Points) // OC[9], end index
			{
				sSingleCommands[iAxis].sSubState = 8;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 8:
			if (cAxes[iAxis].SendSdoUpload(0x04, 4, 0x316E, 1) == (long)gsOC.VisionPulseWidth) // OC[4], pulse duration, us
			{
				sSingleCommands[iAxis].sSubState = 9;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 9:
			if (cAxes[iAxis].SendSdoUpload(0x0A, 4, 0x316E, 1) == (long)gsOC.Direction) // OC[10], oc direction, 1: positive, 2:negative
			{
				sSingleCommands[iAxis].sSubState = 10;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 10:
			if (cAxes[iAxis].SendSdoUpload(0x0B, 4, 0x316E, 1) == 0) // OC[11]=0, convert table
			{
				sSingleCommands[iAxis].sSubState = 11;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;

		case 11:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x30AA, 1) == 2) // GO[14]=2, digital output selection
			{
				sSingleCommands[iAxis].sSubState = 12;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 12:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x3177, 1) == 1) // OL[14]=1, positive
			{
				ucOCIndex = 0;
				sSingleCommands[iAxis].sSubState = 13;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;

		case 13:
			if (cAxes[iAxis].SendSdoUpload(ucOCIndex + 1, 4, 0x307B, 1) == (long)gsOC.Position[ucOCIndex]) // Read ET table
			{
				sSingleCommands[iAxis].sSubState = 14;
			}
			else
			{
				sSingleCommands[iAxis].sSubState = 100;
			}
			break;
		case 14:
			ucOCIndex++;
			if (ucOCIndex == gsOC.Points)
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_DATA_DOWNLOADED;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			else
				sSingleCommands[iAxis].sSubState = 13;
			break;
		case 100:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
			sSingleCommands[iAxis].sState = eIDLE;
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}
void StateFunction_VisionOC_Enable(int iAxis)
{
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case 1:
			sSingleCommands[iAxis].sOCState = NOTHING;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC

			lSDOValue = 1;
			cAxes[iAxis].SendSdoDownload(lSDOValue, 0x08, 4, 0x316E, 1); // OC[8]=1, start index
			lSDOValue = (long)gsOC.Points;
			cAxes[iAxis].SendSdoDownload(lSDOValue, 0x09, 4, 0x316E, 1); // OC[9], end index

			cAxes[iAxis].SendSdoDownload(4, 0x01, 4, 0x316E, 1); // OC[1]=4, enable OC
			sSingleCommands[iAxis].sSubState = 2;

			break;
		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 1) // OC[1]=1, enable OC
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_ENABLED;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			else
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}
void StateFunction_VisionOC_Disable(int iAxis)
{
	try
	{
		switch (sSingleCommands[iAxis].sSubState)
		{
		case 1:
			sSingleCommands[iAxis].sOCState = NOTHING;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			sSingleCommands[iAxis].sSubState = 2;

			break;
		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 0) // OC[1]=1, disabe OC
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_DISABLED;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			else
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
				sSingleCommands[iAxis].sState = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sState = eIDLE;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}
void StartVisionOCInProcess(int iAxis)
{
	try
	{
		switch (sStartVisionOCStep[iAxis])
		{
		case 1:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_DATA_DOWNLOADING;
			gsOC.ValveOCInitFlag = 0;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			cAxes[iAxis].SendSdoDownload(0, 0x06, 4, 0x316E, 1); // OC[6]=0, sensor selection
			cAxes[iAxis].SendSdoDownload(3, 0x07, 4, 0x316E, 1); // OC[7]=3, table selection
			cAxes[iAxis].SendSdoDownload(1, 0x08, 4, 0x316E, 1); // OC[8]=1, start index
			if ((gsOC.Points >= 1) && (gsOC.Points <= MAX_OC_POINTS))
			{
				cAxes[iAxis].SendSdoDownload((long)gsOC.Points, 0x09, 4, 0x316E, 1); // OC[9], end index
			}
			else if (gsOC.Points == 0)
			{
				sStartVisionOCStep[iAxis] = 100;
				break;
			}
			if ((gsOC.PulseDuration > 1) && (gsOC.PulseDuration < 127))
			{
				gsOC.VisionPulseWidth = gsOC.PulseDuration;
			}
			else if (gsOC.PulseDuration > 127)
			{
				gsOC.VisionPulseWidth = 1.0 * gsOC.PulseDuration / 100 - 1 + 127;
			}
			else if (gsOC.PulseDuration == 0)
			{
				sStartVisionOCStep[iAxis] = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload((long)gsOC.VisionPulseWidth, 0x04, 4, 0x316E, 1); // OC[4], pulse duration, us
			if ((gsOC.Direction == 1) || (gsOC.Direction == 2))
			{
				cAxes[iAxis].SendSdoDownload((long)gsOC.Direction, 0x0A, 4, 0x316E, 1); // OC[10], oc direction, 1: positive, 2:negative
			}
			else if (gsOC.Direction == 0)
			{
				sStartVisionOCStep[iAxis] = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload(0, 0x0B, 4, 0x316E, 1); // OC[11]=0, do not convert table
			cAxes[iAxis].SendSdoDownload(2, 0x0E, 4, 0x30AA, 1); // GO[14]=2, port C, A
			cAxes[iAxis].SendSdoDownload(1, 0x0E, 4, 0x3177, 1); // OL[14]=1, positive

			ucOCIndex = 0;
			sStartVisionOCStep[iAxis] = 2;

			break;
		case 2:

			cAxes[iAxis].SendSdoDownload((long)gsOC.Position[ucOCIndex], ucOCIndex + 1, 4, 0x307B, 1); // ET
			sStartVisionOCStep[iAxis] = 3;

			break;
		case 3:
			ucOCIndex++;
			if (ucOCIndex == gsOC.Points)
			{
				sStartVisionOCStep[iAxis] = 4;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 2;
			}
			break;
		case 4:
			if (cAxes[iAxis].SendSdoUpload(0x06, 4, 0x316E, 1) == 0) // OC[6]=0, sensor selection
			{
				sStartVisionOCStep[iAxis] = 5;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 5:
			if (cAxes[iAxis].SendSdoUpload(0x07, 4, 0x316E, 1) == 3) // OC[7]=3, table selection
			{
				sStartVisionOCStep[iAxis] = 6;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 6:
			if (cAxes[iAxis].SendSdoUpload(0x08, 4, 0x316E, 1) == 1) // OC[8]=1, start index
			{
				sStartVisionOCStep[iAxis] = 7;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 7:
			if (cAxes[iAxis].SendSdoUpload(0x09, 4, 0x316E, 1) == (long)gsOC.Points) // OC[9], end index
			{
				sStartVisionOCStep[iAxis] = 8;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 8:
			if (cAxes[iAxis].SendSdoUpload(0x04, 4, 0x316E, 1) == (long)gsOC.VisionPulseWidth) // OC[4], pulse duration, us
			{
				sStartVisionOCStep[iAxis] = 9;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 9:
			if (cAxes[iAxis].SendSdoUpload(0x0A, 4, 0x316E, 1) == (long)gsOC.Direction) // OC[10], oc direction, 1: positive, 2:negative
			{
				sStartVisionOCStep[iAxis] = 10;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 10:
			if (cAxes[iAxis].SendSdoUpload(0x0B, 4, 0x316E, 1) == 0) // OC[11]=0, convert table
			{
				sStartVisionOCStep[iAxis] = 11;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;

		case 11:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x30AA, 1) == 2) // GO[14]=2, digital output selection
			{
				sStartVisionOCStep[iAxis] = 12;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 12:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x3177, 1) == 1) // OL[14]=1, positive
			{
				ucOCIndex = 0;
				sStartVisionOCStep[iAxis] = 13;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;

		case 13:
			if (cAxes[iAxis].SendSdoUpload(ucOCIndex + 1, 4, 0x307B, 1) == (long)gsOC.Position[ucOCIndex]) // Read ET table
			{
				sStartVisionOCStep[iAxis] = 14;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 14:
			ucOCIndex++;
			if (ucOCIndex == gsOC.Points)
			{
				cAxes[iAxis].SendSdoDownload(4, 0x01, 4, 0x316E, 1); // OC[1]=4, Enable OC
				sStartVisionOCStep[iAxis] = 15;
			}
			else
				sStartVisionOCStep[iAxis] = 13;
			break;
		case 15:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 1) // OC[1]=1
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_ENABLED;
				sStartVisionOCStep[iAxis] = eIDLE;
			}
			else
			{
				sStartVisionOCStep[iAxis] = 100;
			}
			break;
		case 100:
			sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
			sSingleCommands[iAxis].sLastGMASError = OC_IN_ERROR;
			sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
			sStartVisionOCStep[iAxis] = eIDLE;
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sStartVisionOCStep[iAxis] = eIDLE;
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}
void StopVisionOCInProcess(int iAxis)
{
	try
	{
		switch (sStopVisionOCStep[iAxis])
		{
		case 1:
			sSingleCommands[iAxis].sOCState = NOTHING;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disabe OC
			sStopVisionOCStep[iAxis] = 2;

			break;
		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 0) // OC[1]=1, disabe OC
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_DISABLED;
				sStopVisionOCStep[iAxis] = eIDLE;
			}
			else
			{
				sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
				sSingleCommands[iAxis].sLastGMASError = OC_IN_ERROR;
				sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
				sStopVisionOCStep[iAxis] = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sStopVisionOCStep[iAxis] = eIDLE;
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

void ValveOCInInit(int iAxis, int iGroup)
{
	try
	{
		switch (sValveOCInitStep[iAxis])
		{
		case 1:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_DATA_DOWNLOADING;

			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1);  // OC[1]=0		Disable OC
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x3068, 1);  // EA[1]=0		Disable emulation
			cAxes[iAxis].SendSdoDownload(29, 0x2A, 4, 0x3034, 1); // CA[42]=29		Map Fast Reference for socket 2
			cAxes[iAxis].SendSdoDownload(13, 0x2B, 4, 0x3034, 1); // CA[43]=13		Use socket 2 for emulation
			cAxes[iAxis].SendSdoDownload(2, 0x04, 4, 0x3068, 1);  // EA[4]=2		Emulation sensor
			cAxes[iAxis].SendSdoDownload(1, 0x08, 4, 0x3068, 1);  // EA[8]=1		Use emulation for OC
			cAxes[iAxis].SendSdoDownload(2, 0x02, 4, 0x3068, 1);  // EA[2]=2		Emulation pulse width
			cAxes[iAxis].SendSdoDownload(0, 0x03, 4, 0x3068, 1);  // EA[3]=0		Emulation direction
			cAxes[iAxis].SendSdoDownload(1, 0x06, 4, 0x3068, 1);  // EA[6]=1		Multiplier
			cAxes[iAxis].SendSdoDownload(0, 0x07, 4, 0x3068, 1);  // EA[7]=0		Divider
			cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x30AA, 1);  // GO[1]=1		Use digital output 1 for OC
			cAxes[iAxis].SendSdoDownload(5, 0x0E, 4, 0x30AA, 1);  // GO[14]=5		Use A of port C for emulation
			cAxes[iAxis].SendSdoDownload(5, 0x0F, 4, 0x30AA, 1);  // GO[15]=5		Use B of port c for emulation
			cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x3177, 1);  // OL[1]=1		Set digital output 1 to high
			cAxes[iAxis].SendSdoDownload(1, 0x0E, 4, 0x3177, 1);  // OL[14]=1		Set digital output 14 to high
			cAxes[iAxis].SendSdoDownload(1, 0x0F, 4, 0x3177, 1);  // OL[15]=1		Set digital output 15 to high

			cAxes[iAxis].SendSdoDownload(3, 0x06, 4, 0x316E, 1);						// OC[6]=3
			cAxes[iAxis].SendSdoDownload((long)gsOC.StartPosition, 0x02, 4, 0x316E, 1); // OC[2]=0
			cAxes[iAxis].SendSdoDownload((long)gsOC.DeltaPosition, 0x03, 4, 0x316E, 1); // OC[3]

			if ((gsOC.PulseDuration > 1) && (gsOC.PulseDuration < 127))
			{
				gsOC.ValvePulseWidth = gsOC.PulseDuration;
			}
			else if (gsOC.PulseDuration > 127)
			{
				gsOC.ValvePulseWidth = 1.0 * gsOC.PulseDuration / 100 - 1 + 127;
			}
			else if (gsOC.PulseDuration == 0)
			{
				sValveOCInitStep[iAxis] = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload((long)gsOC.ValvePulseWidth, 0x04, 4, 0x316E, 1); // OC[4], pulse duration, us
			cAxes[iAxis].SendSdoDownload(100000000, 0x05, 4, 0x316E, 1);				  // OC[5]=100000000

			GroupOfPrmWrt[0].eParameterNumber = MMC_FAST_REFERENCE_MODE; // MMC_FAST_REFERENCE_MODE : 0;
			GroupOfPrmWrt[0].iParameterIndex = 0;

			GroupOfPrmWrt[0].usAxisRef = cAxes[iAxis].GetRef();
			GroupOfPrmWrt[0].dbValue = 0;

			GroupOfPrmWrt[1].eParameterNumber = MMC_FAST_REFERENCE; // MMC_FAST_REFERENCE
			GroupOfPrmWrt[1].iParameterIndex = 0;

			GroupOfPrmWrt[1].usAxisRef = cAxes[iAxis].GetRef();
			GroupOfPrmWrt[1].dbValue = (cVector[iGroup].GetRef() << 16) + 0x03;

			cAxes[iAxis].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 2);

			GroupOfPrmWrt[0].eParameterNumber = MMC_MCS_S_DIRECTION; // Clear S length
			GroupOfPrmWrt[0].iParameterIndex = 0;

			GroupOfPrmWrt[0].usAxisRef = cVector[iGroup].GetRef();
			GroupOfPrmWrt[0].dbValue = 0;

			cAxes[iAxis].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 1);

			cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x3068, 1); // EA[1]=1
			cAxes[iAxis].SendSdoDownload(0, 0x03, 4, 0x3091, 1); // FP[3]=0

			sValveOCInitStep[iAxis] = 2;

			break;

		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x2A, 4, 0x3034, 1) == 29) // CA[42]=29
			{
				sValveOCInitStep[iAxis] = 3;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 3:
			if (cAxes[iAxis].SendSdoUpload(0x2B, 4, 0x3034, 1) == 13) // CA[43]=13
			{
				sValveOCInitStep[iAxis] = 4;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 4:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x3068, 1) == 1) // EA[1]=1
			{
				sValveOCInitStep[iAxis] = 5;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 5:
			if (cAxes[iAxis].SendSdoUpload(0x04, 4, 0x3068, 1) == 2) // EA[4]=2
			{
				sValveOCInitStep[iAxis] = 6;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 6:
			if (cAxes[iAxis].SendSdoUpload(0x08, 4, 0x3068, 1) == 1) // EA[8]=1
			{
				sValveOCInitStep[iAxis] = 7;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 7:
			if (cAxes[iAxis].SendSdoUpload(0x02, 4, 0x3068, 1) == 2) // EA[2]=2
			{
				sValveOCInitStep[iAxis] = 8;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 8:
			if (cAxes[iAxis].SendSdoUpload(0x03, 4, 0x3068, 1) == 0) // EA[3]=0
			{
				sValveOCInitStep[iAxis] = 9;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 9:
			if (cAxes[iAxis].SendSdoUpload(0x06, 4, 0x3068, 1) == 1) // EA[6]=1
			{
				sValveOCInitStep[iAxis] = 10;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 10:
			if (cAxes[iAxis].SendSdoUpload(0x07, 4, 0x3068, 1) == 0) // EA[7]=0
			{
				sValveOCInitStep[iAxis] = 11;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 11:
			if (cAxes[iAxis].SendSdoUpload(0x06, 4, 0x316E, 1) == 3) // OC[6]=3
			{
				sValveOCInitStep[iAxis] = 12;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 12:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x30AA, 1) == 1) // GO[1]=1
			{
				sValveOCInitStep[iAxis] = 13;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 13:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x30AA, 1) == 5) // GO[14]=5
			{
				sValveOCInitStep[iAxis] = 14;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 14:
			if (cAxes[iAxis].SendSdoUpload(0x0F, 4, 0x30AA, 1) == 5) // GO[15]=5
			{
				sValveOCInitStep[iAxis] = 15;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 15:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x3177, 1) == 1) // OL[1]=1
			{
				sValveOCInitStep[iAxis] = 16;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 16:
			if (cAxes[iAxis].SendSdoUpload(0x0E, 4, 0x3177, 1) == 1) // OL[14]=1
			{
				sValveOCInitStep[iAxis] = 17;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 17:
			if (cAxes[iAxis].SendSdoUpload(0x0F, 4, 0x3177, 1) == 1) // OL[15]=1
			{
				sValveOCInitStep[iAxis] = 18;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 18:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 0) // OC[1]=0
			{
				sValveOCInitStep[iAxis] = 19;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 19:
			if (cAxes[iAxis].SendSdoUpload(0x02, 4, 0x316E, 1) == (long)gsOC.StartPosition) // OC[2]
			{
				sValveOCInitStep[iAxis] = 20;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 20:
			if (cAxes[iAxis].SendSdoUpload(0x03, 4, 0x316E, 1) == (long)gsOC.DeltaPosition) // OC[3]
			{
				sValveOCInitStep[iAxis] = 21;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 21:
			if (cAxes[iAxis].SendSdoUpload(0x04, 4, 0x316E, 1) == (long)gsOC.ValvePulseWidth) // OC[4]
			{
				sValveOCInitStep[iAxis] = 22;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 22:
			if (cAxes[iAxis].SendSdoUpload(0x05, 4, 0x316E, 1) == 100000000) // OC[5]=100000000
			{
				sValveOCInitStep[iAxis] = 23;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 23:
			if (cAxes[iAxis].SendSdoUpload(0x03, 4, 0x3091, 1) == 0) // FP[3]=0
			{
				cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x316E, 1); // OC[1]=1
				sValveOCInitStep[iAxis] = 24;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;

		case 24:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 2) // OC[1]=2
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_ENABLED;
				sValveOCInitStep[iAxis] = eIDLE;
				gsOC.ValveOCInitFlag = 1;
			}
			else
			{
				sValveOCInitStep[iAxis] = 100;
			}
			break;
		case 100:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
			sValveOCInitStep[iAxis] = eIDLE;
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sValveOCInitStep[iAxis] = eIDLE;
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

void StartValveOCInProcess(int iAxis, int iGroup)
{
	try
	{
		switch (sStartValveOCStep[iAxis])
		{
		case 1:
			sSingleCommands[iAxis].sOCState = MOTOR_OC_DATA_DOWNLOADING;

			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1);						// OC[1]=0
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x3068, 1);						// EA[1]=0
			cAxes[iAxis].SendSdoDownload((long)gsOC.StartPosition, 0x02, 4, 0x316E, 1); // OC[2]
			cAxes[iAxis].SendSdoDownload((long)gsOC.DeltaPosition, 0x03, 4, 0x316E, 1); // OC[3]

			if ((gsOC.PulseDuration > 1) && (gsOC.PulseDuration < 127))
			{
				gsOC.ValvePulseWidth = gsOC.PulseDuration;
			}
			else if (gsOC.PulseDuration > 127)
			{
				gsOC.ValvePulseWidth = 1.0 * gsOC.PulseDuration / 100 - 1 + 127;
			}
			else if (gsOC.PulseDuration == 0)
			{
				sStartValveOCStep[iAxis] = 100;
				break;
			}
			cAxes[iAxis].SendSdoDownload((long)gsOC.ValvePulseWidth, 0x04, 4, 0x316E, 1); // OC[4], pulse duration, us

			GroupOfPrmWrt[0].eParameterNumber = MMC_MCS_S_DIRECTION; // Clear S length
			GroupOfPrmWrt[0].iParameterIndex = 0;
			GroupOfPrmWrt[0].usPadding1 = 0;
			GroupOfPrmWrt[0].usPadding2 = 0;
			GroupOfPrmWrt[0].usPadding3 = 0;

			GroupOfPrmWrt[0].usAxisRef = cVector[iGroup].GetRef();

			GroupOfPrmWrt[0].dbValue = 0;
			cAxes[iAxis].WriteGroupOfParametersImmediate(GroupOfPrmWrt, 1);

			cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x3068, 1); // EA[1]=1
			cAxes[iAxis].SendSdoDownload(0, 0x03, 4, 0x3091, 1); // FP[3]=0

			sStartValveOCStep[iAxis] = 2;

			break;

		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x3068, 1) == 1) // EA[1]=1
			{
				sStartValveOCStep[iAxis] = 3;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 3:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 0) // OC[1]=0
			{
				sStartValveOCStep[iAxis] = 4;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 4:
			if (cAxes[iAxis].SendSdoUpload(0x02, 4, 0x316E, 1) == (long)gsOC.StartPosition) // OC[2]
			{
				sStartValveOCStep[iAxis] = 5;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 5:
			if (cAxes[iAxis].SendSdoUpload(0x03, 4, 0x316E, 1) == (long)gsOC.DeltaPosition) // OC[3]
			{
				sStartValveOCStep[iAxis] = 6;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 6:
			if (cAxes[iAxis].SendSdoUpload(0x04, 4, 0x316E, 1) == (long)gsOC.ValvePulseWidth) // OC[4]
			{
				sStartValveOCStep[iAxis] = 7;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 7:
			if (abs(cAxes[iAxis].SendSdoUpload(0x03, 4, 0x3091, 1)) <= 100) // FP[3]=0
			{
				cAxes[iAxis].SendSdoDownload(1, 0x01, 4, 0x316E, 1); // OC[1]=1
				sStartValveOCStep[iAxis] = 8;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;

		case 8:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) >= 1) // OC[1]>=1
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_ENABLED;
				sStartValveOCStep[iAxis] = eIDLE;
			}
			else
			{
				sStartValveOCStep[iAxis] = 100;
			}
			break;
		case 100:
			sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
			sSingleCommands[iAxis].sLastGMASError = OC_IN_ERROR;
			sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
			sStartValveOCStep[iAxis] = eIDLE;
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sStartValveOCStep[iAxis] = eIDLE;
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

void StopValveOCInProcess(int iAxis)
{
	try
	{
		switch (sStopValveOCStep[iAxis])
		{
		case 1:
			sSingleCommands[iAxis].sOCState = NOTHING;
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x316E, 1); // OC[1]=0, disable OC
			cAxes[iAxis].SendSdoDownload(0, 0x01, 4, 0x3068, 1); // EA[1]=0
			sStopValveOCStep[iAxis] = 2;

			break;

		case 2:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x3068, 1) == 0)
			{
				sStopValveOCStep[iAxis] = 3;
			}
			else
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
				sStopValveOCStep[iAxis] = eIDLE;
			}
			break;

		case 3:
			if (cAxes[iAxis].SendSdoUpload(0x01, 4, 0x316E, 1) == 0) // OC[1]=0, disable OC
			{
				sSingleCommands[iAxis].sOCState = MOTOR_OC_IS_DISABLED;
				sStopValveOCStep[iAxis] = eIDLE;
			}
			else
			{
				sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
				sSingleCommands[iAxis].sLastGMASError = OC_IN_ERROR;
				sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
				sStopValveOCStep[iAxis] = eIDLE;
			}
			break;
		default:
			break;
		}
	}
	catch (CMMCException &ex)
	{
		sSingleCommands[iAxis].cInErrorDriveOrGmas = eGMASErr;
		sSingleCommands[iAxis].sLastGMASError = (short)ex.error();
		sStopValveOCStep[iAxis] = eIDLE;
		sSingleCommands[iAxis].sOCState = MOTOR_OC_ERROR;
		printf("ERROR: %s error %d, status %d\n", ex.what(), (short)ex.error(), ex.status());
	}
	return;
}

unsigned long int PmasTimeUpdate()
{
	unsigned long int iTime;
	gettimeofday(&tv, NULL);
	iTime = tv.tv_sec * 1000000 + tv.tv_usec;
	return (iTime);
}
