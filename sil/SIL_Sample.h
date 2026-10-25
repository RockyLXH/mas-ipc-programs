/*
 ============================================================================
 Name : PID_Sample.cpp
 Author  :	Elmo Motion Control
 Version :
 Description : 	GMAS C/C++ project header file for Template
 ============================================================================
 */
#pragma once

#include "pid.h"
/*
 ============================================================================
 Project general functions prototypes
 ============================================================================
 */

void MainInit(void);
void MainClose(void);
void SILInit(void);
void MainLoop(void);
int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
				   unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);

int CallbackFunc(unsigned char *recvBuffer, short recvBufferSize, void *lpsock);

int DoSilTest(void);			 // Pos mode
int DoAnalogCmdForVelLoop(void); // Vel mode
int DoSinGenForPosLoop(void);	 // Pos mode
int DoVelLoopPidCtrl(void);		 // Torque mode
int DoRatchetEffect(void);		 // Torque mode
int DoEdgeEffect(void);			 // Torque mode
int DoSmoothEffect(void);		 // Torque mode
int DoDampEffect(void);			 // Torque mode
/*
 ============================================================================
 General constants
 ============================================================================
 */
constexpr unsigned short MAX_AXES = 1; // number of Physical axes in the system
constexpr int TEST_COUNT = 200000;
/*
 ============================================================================
 Application global variables
 ============================================================================
 */

bool is_terminate = false; // Flag to request program termination
double target_vel = 0.0;
double vel_ki = 0.0;
double vel_kp = 0.0001;

PIDController pid_vel{vel_kp, vel_ki, 0.0f, 1000.0, 1.0, 0.001};

unsigned int arr_rec[TEST_COUNT] = {0};
bool is_test_done = false;
double pos[MAX_AXES] = {0.0};
/*
 ============================================================================
 Global structures for Elmo's Function Blocks
 ============================================================================
 */
MMC_CONNECT_HNDL conn_handler; // Connection Handle
CMMCConnection conn;
CMMCHostComm mbus;

MMC_MOTIONPARAMS_SINGLE axis_default_para; // Single axis default data

CMMCRTSingleAxis axis_rt[MAX_AXES];

MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;

struct Instance
{
	int (*func)(void);
	MMC_PARAMETER_LIST_ENUM src;
	OPM402 mode;
};