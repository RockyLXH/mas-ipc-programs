/*
============================================================================
 Name : ext_rs485_example.cpp
 Author :
 Version :
 Description : GMAS C++ project source file
============================================================================
*/

#include "mmcpplib.h"
#include "mmc_definitions.h"
#include "ext_rs232_example.h"		// Application header file.
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
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
		//
		// Initialize system, axes and all needed initializations
		MainInit();
//
//		// set IO 1 to UART transmit and 2 to receive
		cConn.SetGlobalBoolParameter(eCPLD_RS485_FUNCTIONALITY_UART_RS232_TRANSMIT, MMC_CPLD_RS485_FUNCTIONALITY, 3);
////		cConn.SetGlobalBoolParameter(eCPLD_RS485_FUNCTIONALITY_UART_RS232_RECEIVE, MMC_CPLD_RS485_FUNCTIONALITY, 4);
////
		cConn.SetGlobalBoolParameter(eNC_CPLD_CONFIG_RS485, MMC_CPLD_CONFIG, 0);
//
		char buff[10];
		int rc = 0;
//
//
		if(!(rc=open_uart_port(&rs232_1, 3, 115200, "Normal"))) {
			cout << "port rs232_1 open failed " << rc << endl;
			exit(-1);
		}
		cout << "port rs232_1 opened as " << rc << endl;
//
//		if(!(rc=open_uart_port(&rs232_2, 4, 115200, "Normal"))) {
//			cout << "port rs232_2 open failed " << rc << endl;
//			exit(-1);
//		}
//		cout << "port rs232_2 opened as " << rc << endl;

		rc = rs232_1.Write("abcdefgh",8);
		cout << rc << endl;

		while (1) {

		      // read up to 10 bytes into buff
//		      rc = rs232_2.Read(buff, 10);
		      if(rc) {
		           printf("#2 Read %d bytes\n", rc);
		    	   cout<<buff<<endl;
		      }

//		      rc = rs232_1.Read(buff, 10);
		      if(rc) {
		           printf("#1 Read %d bytes\n", rc);
		    	   cout<<buff<<endl;
		      }

		      // every 100 ms check for new data
		      usleep(1000000);
		}

		//	Close what needs to be closed before prram termination
		MainClose();
		//
		// Terminate the application program back to the Operating System
		return 1;

	}
	catch(CMMCException& exception)
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
static void MainInit()
{
	//
	// Here will come initialization code for all system, axes, communication etc.
	//
	struct sigaction stSigAction;
	//
	// Init the sigAction structure.
	memset(&stSigAction, 0, sizeof(stSigAction));
	stSigAction.sa_handler = &TerminateApplication;
	// Initialized case of CTRL+C.
	sigaction(SIGINT, &stSigAction, NULL);
	//
	// InitializeCommunication to the GMAS:
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	//
	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//

	// Register the callback function for Emergency:
	cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ;

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
static void MainClose()
{
//
//	Here will come code for all closing processes
//
	MMC_CloseConnection(gConnHndl) ;
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
static int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	// Which function ID was received ...
	switch(recvBuffer[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n") ;
		break ;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n") ;
		break ;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n") ;
		break ;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n") ;
		break ;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n") ;
		break ;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n") ;
		break ;
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
	return 1 ;
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
static int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
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
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines.
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
static void TerminateApplication(int iSigNum)
{
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1 ;
	sigignore(SIGALRM);
	//
	switch(iSigNum)
	{
		// Handle ctrl+c.
		case SIGINT:
			// TODO Close what needs to be closed before program termination.
			exit(0);
			break;
		default:
			break;
	}
	return ;
}

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
static void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}

static int open_uart_port(CMaestroSerialPort *sp, const int port_num, const unsigned int baud_rate, const char *mode)
{
	CMaestroSerialPort::SMaestroSerialPortCfg cfg;

	if(strcmp("Normal", mode) == 0)
		cfg.mode = sp->Normal;
	else
		cfg.mode = sp->Enhanced;

	cfg.speed = baud_rate;

	return sp->Open(port_num, cfg);
}
