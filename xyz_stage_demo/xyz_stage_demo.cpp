/*
============================================================================
 Name : xyz_stage_demo.cpp
 Author :	Elmo Motion Control.
 Version :	1.00
 Description : The following example supports the following functionalities:

- Power On.
- Group Enable.
- Move Linear Absolute to 0.
- Move Linear Absolute to 10000.
- Move Linear Absolute to 20000.
- Move Linear Absolute to 0.
- Power Off.
- Group Disable.

 The program works with group v01 of 2 axes - a01 and a02.
 The program works with CAN and EtherCAT.

============================================================================
*/
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "xyz_stage_demo.h" // Application header file.
#include <iostream>
#include <sys/time.h> // For time structure
#include <signal.h>	  // For Timer mechanism
/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				04/10/2015
 Modifications:			N/A

 Description:

 The main function of this sample project.
============================================================================
*/

int main()
{
	try
	{
		//	Initialize system, axes and all needed initializations
		MainInit();
		//
		// Changes the NC motion mode, according to communication type (ETHERCAT / CAN).
		// ChangeToRelevantMode();
		//
		/////////////////////////////////////// Motor On function - axis 1//////////////////////////////////////////
		//
		a1.PowerOn();
		//
		// Waiting for Motor on.
		// In the end of Motor On function the axis should be in StandStill state.
		giXStatus = a1.ReadStatus();
		while (!(giXStatus & NC_AXIS_STAND_STILL_MASK) && !(giXStatus & NC_AXIS_ERROR_STOP_MASK))
			giXStatus = a1.ReadStatus();
		//
		if (giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting.";
			exit(0);
		}
		//
		cout << "Axis a1 is powered on" << endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		/////////////////////////////////////// Motor On function - axis 2///////////////////////////////////////////
		//
		a2.PowerOn();
		//
		// Waiting for Motor on.
		// In the end of Motor On function the axis should be in StandStill state.
		giYStatus = a2.ReadStatus();
		while (!(giYStatus & NC_AXIS_STAND_STILL_MASK) && !(giYStatus & NC_AXIS_ERROR_STOP_MASK))
			giYStatus = a2.ReadStatus();
		//
		if (giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting.";
			exit(0);
		}
		cout << "Axis a2 is powered on" << endl;
		//
		a3.PowerOn();
		//
		// Waiting for Motor on.
		// In the end of Motor On function the axis should be in StandStill state.
		giZStatus = a3.ReadStatus();
		while (!(giZStatus & NC_AXIS_STAND_STILL_MASK) && !(giZStatus & NC_AXIS_ERROR_STOP_MASK))
			giZStatus = a3.ReadStatus();
		//
		if (giZStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a3 in Error Stop. Aborting.";
			exit(0);
		}
		//
		cout << "Axis a3 is powered on" << endl;
		//

		a1.SetOpMode(OPM402_HOMING_MODE);
		giXOpMode = (OPM402)a1.GetOpMode();
		while (giXOpMode != OPM402_HOMING_MODE)
			giXOpMode = a1.GetOpMode();
		a2.SetOpMode(OPM402_HOMING_MODE);
		giYOpMode = (OPM402)a2.GetOpMode();
		while (giYOpMode != OPM402_HOMING_MODE)
			giYOpMode = a2.GetOpMode();
		a3.SetOpMode(OPM402_HOMING_MODE);
		giZOpMode = (OPM402)a3.GetOpMode();
		while (giZOpMode != OPM402_HOMING_MODE)
			giZOpMode = a3.GetOpMode();

		MMC_HOMEDS402EX_IN home_parameter;
		home_parameter.dbDetectionVelocityLimit = 5;
		home_parameter.dbPosition = 0;
		home_parameter.eBufferMode = MC_BUFFERED_MODE;
		home_parameter.fAcceleration = 100;
		home_parameter.fDistanceLimit = 0;
		home_parameter.fTorqueLimit = 0.5;
		home_parameter.fVelocityHi = 5;
		home_parameter.fVelocityLo = 5;
		home_parameter.uiHomingMethod = -2;
		home_parameter.ucExecute = 1;
		a1.HomeDS402Ex(home_parameter);
		a2.HomeDS402Ex(home_parameter);
		a3.HomeDS402Ex(home_parameter);
		while (!((a1.ReadStatus() & NC_AXIS_STAND_STILL_MASK) && (a2.ReadStatus() & NC_AXIS_STAND_STILL_MASK) && (a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK)))
			;

		a1.SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		giXOpMode = (OPM402)a1.GetOpMode();
		while (giXOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE)
			giXOpMode = a1.GetOpMode();
		a2.SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		giYOpMode = (OPM402)a2.GetOpMode();
		while (giYOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE)
			giYOpMode = a2.GetOpMode();
		a3.SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		giZOpMode = (OPM402)a3.GetOpMode();
		while (giZOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE)
			giZOpMode = a3.GetOpMode();
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		///////////////////////////////////////////////// Group Enable //////////////////////////////////////////////
		//
		// Group Enable.
		v1.GroupEnable();
		//
		// Waiting for GroupEnable.
		// In the end of GroupEnable function the group should be in StandBy state.
		giGroupStatus = v1.ReadStatus();
		//
		while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
			giGroupStatus = v1.ReadStatus();
		//
		if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
		{
			cout << "Group v1 in Error Stop. Aborting.";
			exit(0);
		}
		//
		cout << "Group v1 is enabled" << endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		//////////////////////////////////////// Group v1 - Move Linear Absolute to 0 ////////////////////////////////////
		//
		v1.m_dEndPoint[0] = 10;
		v1.m_dEndPoint[1] = 10;
		v1.MoveLinearAbsolute(MC_BUFFERED_MODE);
		// Waiting for MoveLinearAbsolute.
		// In the end of MoveLinearAbsolute function the group should be in StandBy state.
		giGroupStatus = v1.ReadStatus();
		while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
			giGroupStatus = v1.ReadStatus();
		//
		if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
		{
			cout << "Group v1 in Error Stop. Aborting.";
			exit(0);
		}

		a3.MoveAbsolute(60);
		a3.MoveAbsolute(5);
		while (!(a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
			;

		for (;;)
		{
			v1.m_dEndPoint[0] = 10;
			v1.m_dEndPoint[1] = 120;
			//
			v1.MoveLinearAbsolute();
			//
			// Waiting for MoveLinearAbsolute.
			// In the end of MoveLinearAbsolute function the group should be in StandBy state.
			giGroupStatus = v1.ReadStatus();
			while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
				giGroupStatus = v1.ReadStatus();
			//
			if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
			{
				cout << "Group v1 in Error Stop. Aborting.";
				exit(0);
			}

			a3.MoveAbsolute(60);
			a3.MoveAbsolute(5);
			while (!(a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
				;

			double center_point[2] = {65,120};
			v1.MoveCircularAbsoluteAngle(-180, center_point, MC_BUFFERED_MODE);
			giGroupStatus = v1.ReadStatus();
			while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
				giGroupStatus = v1.ReadStatus();
			//
			if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
			{
				cout << "Group v1 in Error Stop. Aborting.";
				exit(0);
			}

			a3.MoveAbsolute(60);
			a3.MoveAbsolute(5);
			while (!(a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
				;

			v1.m_dEndPoint[0] = 120;
			v1.m_dEndPoint[1] = 70;
			//
			v1.MoveLinearAbsolute();
			//
			// Waiting for MoveLinearAbsolute.
			// In the end of MoveLinearAbsolute function the group should be in StandBy state.
			giGroupStatus = v1.ReadStatus();
			while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
				giGroupStatus = v1.ReadStatus();
			//
			if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
			{
				cout << "Group v1 in Error Stop. Aborting.";
				exit(0);
			}

			a3.MoveAbsolute(60);
			a3.MoveAbsolute(5);
			while (!(a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
				;

			center_point[0] = 65;
			center_point[1] = 70; 
			v1.MoveCircularAbsoluteAngle(-180, center_point, MC_BUFFERED_MODE);
			giGroupStatus = v1.ReadStatus();
			while (!(giGroupStatus & NC_GROUP_STANDBY_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
				giGroupStatus = v1.ReadStatus();
			//
			if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
			{
				cout << "Group v1 in Error Stop. Aborting.";
				exit(0);
			}

			a3.MoveAbsolute(60);
			a3.MoveAbsolute(5);
			while (!(a3.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
				;
		}

		// Group Disable.
		v1.GroupDisable();
		//
		// Waiting for GroupDisable.
		// In the end of GroupDisable function the group should be in Disabled state.
		giGroupStatus = v1.ReadStatus();
		//
		while (!(giGroupStatus & NC_GROUP_DISABLED_MASK) && !(giGroupStatus & NC_GROUP_ERROR_STOP_MASK))
			giGroupStatus = v1.ReadStatus();
		//
		if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
		{
			cout << "Group v1 in Error Stop. Aborting.";
			exit(0);
		}
		//
		cout << "Group v1 is disabled" << endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		// Close what needs to be closed before program termination
		MainClose();
		//
		// Terminate the application program back to the Operating System
		return 1;
	}
	catch (CMMCException &exception)
	{
		printf("Exception in function %s, axis ref=%s, err=%d, status=%d, bye\n", exception.what(), exception.axisName(), exception.error(), exception.status());
		MainClose();
		exit(0);
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught\n";
		MainClose();
		exit(0);
	}
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
void MainInit()
{
	// Here will come initialization code for all system, axes, communication etc.
	//
	struct sigaction stSigAction;
	//
	// Init the sigAction structure.
	memset(&stSigAction, 0, sizeof(stSigAction));
	stSigAction.sa_handler = &TerminateApplication;
	// Initialized case of CTRL+C.
	sigaction(SIGINT, &stSigAction, NULL);
	sigaction(SIGTERM, &stSigAction, NULL);
	//
	// InitializeCommunication to the GMAS:
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK)CallbackFunc);
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//
	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true, false);
	//
	// Register the callback function for Emergency:
	cConn.RegisterEventCallback(MMCPP_EMCY, (void *)Emergency_Received);
	//
	// Initialize default parameters of single axis. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity = 0;
	stSingleDefault.dbDistance = 100000;
	stSingleDefault.dbPosition = 0;
	stSingleDefault.fVelocity = 2000;
	stSingleDefault.fAcceleration = 5000;
	stSingleDefault.fDeceleration = 5000;
	stSingleDefault.fJerk = 10000;
	stSingleDefault.eDirection = MC_POSITIVE_DIRECTION;
	stSingleDefault.eBufferMode = MC_BUFFERED_MODE;
	stSingleDefault.ucExecute = 1;
	//
	// Initialize default parameters of group axis.
	memset(stGroupDefault.dAuxPoint, 0, sizeof(double) * NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.dEndPoint, 0, sizeof(double) * NC_MAX_NUM_AXES_IN_NODE);
	memset(stGroupDefault.fTransitionParameter, 0, sizeof(float) * NC_MAX_NUM_AXES_IN_NODE);
	stGroupDefault.fVelocity = 1000;
	stGroupDefault.fAcceleration = 2000;
	stGroupDefault.fDeceleration = 2000;
	stGroupDefault.fJerk = 5000;
	stGroupDefault.eCoordSystem = MC_MCS_COORD;
	stGroupDefault.eTransitionMode = MC_TM_NONE_MODE;
	stGroupDefault.eArcShortLong = MC_NONE_ARC_CHOICE;
	stGroupDefault.ePathChoice = MC_NONE_PATH_CHOICE;
	stGroupDefault.eCircleMode = MC_ANGLE_CIRC_MODE;
	stGroupDefault.ucSuperimposed = 0;
	stGroupDefault.m_uiExecDelayMs = 0;
	stGroupDefault.ucExecute = 1;
	//
	// TODO: Update number of necessary axes:
	//
	a1.InitAxisData("X", gConnHndl);
	a2.InitAxisData("Y", gConnHndl);
	a3.InitAxisData("Z", gConnHndl);
	v1.InitAxisData("v01", gConnHndl);
	//
	// Set default motion parameters. TODO: Update for all axes.
	a1.SetDefaultParams(stSingleDefault);
	a2.SetDefaultParams(stSingleDefault);
	a3.SetDefaultParams(stSingleDefault);
	v1.SetDefaultParams(stGroupDefault);
	//
	// You may of course change internal parameters manually:
	// a1.m_fAcceleration = 10000;
	//
	giXStatus = a1.ReadStatus();
	if (giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a1.Reset();
		giXStatus = a1.ReadStatus();
		if (giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
	giYStatus = a2.ReadStatus();
	if (giYStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a2.Reset();
		giYStatus = a2.ReadStatus();
		if (giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
	giZStatus = a3.ReadStatus();
	if (giZStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a3.Reset();
		giZStatus = a3.ReadStatus();
		if (giZStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a3 in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
	giGroupStatus = v1.ReadStatus();
	if (giGroupStatus & NC_GROUP_ERROR_STOP_MASK)
	{
		v1.Reset();
		giGroupStatus = v1.ReadStatus();

		if (giGroupStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Group v1 in Error Stop. Aborting.";
			exit(0);
		}
	}
	//
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
void MainClose()
{
	//
	//	Here will come code for all closing processes
	//
	MMC_CloseConnection(gConnHndl);
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
//
//
///////////////////////////////////////////////////////////////////////
//	Function name	:	int OnRunTimeError(const char *msg,  MMC_CONNECT_HNDL uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
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
int OnRunTimeError(const char *msg, MMC_CONNECT_HNDL uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
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
	switch (iSigNum)
	{
	// Handle ctrl+c.
	case SIGINT:
		// TODO Close what needs to be closed before program termination.
		exit(0);
		break;
	default:
		break;
	}
	return;
}
//
//
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
	printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}
//
///////////////////////////////////////////////////////////////////////
//	Function name	:	ChangeToRelevantMode
//	Created			:	Version 1.00
//	Updated			:	12/10/2015
//	Purpose			:	Called in case of working with group.
//						Changes the NC motion mode, according to communication type (ETHERCAT / CAN).
//	Input			:	N/A
//	Output			:	N/A
//	Return Value	:	void
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void ChangeToRelevantMode()
{
	double dConnectionType;
	//
	dConnectionType = cConn.GetGlobalBoolParameter(MMC_CONNECTION_TYPE_PARAM, 0);
	//
	// ETHERCAT
	if (dConnectionType == eCOMM_TYPE_ETHERCAT)
	{
		a1.SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		//
		// Waiting for Set Operation Mode
		giXOpMode = (OPM402)a1.GetOpMode();
		while (giXOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE)
			giXOpMode = a1.GetOpMode();
		//
		a2.SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
		//
		// Waiting for Set Operation Mode
		giYOpMode = a2.GetOpMode();
		while (giYOpMode != OPM402_CYCLIC_SYNC_POSITION_MODE)
			giYOpMode = a2.GetOpMode();
	}
	// CAN
	else
	{
		a1.SetOpMode(OPM402_INTERPOLATED_POSITION_MODE);
		// Waiting for Set Operation Mode
		//
		giXOpMode = a1.GetOpMode();
		while (giXOpMode != OPM402_INTERPOLATED_POSITION_MODE)
			giXOpMode = a1.GetOpMode();
		//
		a2.SetOpMode(OPM402_INTERPOLATED_POSITION_MODE);
		//
		// Waiting for Set Operation Mode
		giYOpMode = a2.GetOpMode();
		while (giYOpMode != OPM402_INTERPOLATED_POSITION_MODE)
			giYOpMode = a2.GetOpMode();
	}
}
//
//
