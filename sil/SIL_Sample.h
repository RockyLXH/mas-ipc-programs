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
#include <unordered_map>
#include "types_alias.h"
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
int ReadResourceFile();
/*
 ============================================================================
 General constants
 ============================================================================
 */
constexpr u32 TEST_COUNT = 20000;
constexpr u16 MAX_AXIS_CNTS = 1;
/*
 ============================================================================
 Application global variables
 ============================================================================
 */
bool is_terminate = false;
f64 target_vel = 0.0f;
f64 vel_ki = 0.0f;
f64 vel_kp = 0.0f;

PIDController pid_vel{vel_kp, vel_ki, 0.0f, 1000.0, 1.0, 0.001};

u32 arr_rec[TEST_COUNT]{};
bool is_test_done = false;
std::vector<f64> actual_pos;

/*
 ============================================================================
 Global structures for Elmo's Function Blocks
 ============================================================================
 */
MMC_CONNECT_HNDL conn_handler; // Connection Handle
CMMCConnection conn;
CMMCHostComm mbus;

MMC_MOTIONPARAMS_SINGLE_EX axis_default_param; // Single axis default data

std::vector<std::string> axis_names;
std::vector<std::string> vec_names;

CMMCRTSingleAxis axis_rt[MAX_AXIS_CNTS];

MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;

struct SilConfig
{
	int (*func)(void);
	MMC_PARAMETER_LIST_ENUM src;
	OPM402 mode;
};
