/*
============================================================================
 Name : onlnspln.cpp
 Author :
 Version :
 Description : GMAS C++ project source file
============================================================================
*/
#ifdef WIN32
#pragma warning(disable : 5208)
#endif

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifndef WIN32
#include <signal.h>
#include <time.h>
#endif

#include <mmc_definitions.h>
#include <mmcpplib.h>

using namespace std;

#define _host "192.168.1.101"	//home
#define _dest "192.168.1.2"	//maestro
//#define _host "192.168.1.100"	//office
//#define _dest "192.168.1.3"		//maestro

int events_clbck(unsigned char* buff, short buffer_size, void* lpsock);
volatile int _quit = 0;
MMC_CONNECT_HNDL	_connh = 0;
CMMCConnection		_conn;
CMMCGroupAxis		_v01;

//global variables to manage dynamic point insertion for online spline mechanism.
//starting from zero, each additional packate of points will start where the previous package ends. 

//rows (points) management
uint32_t _total_rows = 201;					//total number of rows in maestro memory.
uint32_t _packet_rows = 50;					//number of rows to send on each appending command.
uint16_t _usDimension = 2;

//columns management of local buffer on this host (PC)
uint32_t _total_columns = _total_rows * (_usDimension + 1); //total number columns  +1 for the time column
uint32_t _packet_columns = _packet_rows * (_usDimension + 1); //number of columns to send on each appending command. +1 for the time column
uint32_t _index = 0;						//access index into global table _2dtable_p in local buffer.
uint32_t _rows_sent = 0;					//number of rows (points), which were allready sent.
volatile uint8_t _underflow = 0;						//a flag notifying too append next rows package.
double _tablert[NC_PVT_ECAM_MAX_ARRAY_SIZE]; //the api to append points expects an array of this size even if only partially utilized.


/*
* \fn void bye()
* \brief this function handles the final tasks befor exiting.
*/
void  bye(void)
{
	printf("That was all, folks exit_program(%d), gConnHndl(%#X)\n", _quit, _connh);
	if (_connh)
		MMC_CloseConnection(_connh);

}

/*
 * \fn void termination_handler (int signum)
 * \brief		this function is called by the system in case one of the registered (see init_connection) signals occurs.
 * \param signum	the raised signal number.
 * \return none.
 */
void termination_handler(int signum)
{
	fprintf(stderr, "%s(%d): signum(%d)" "\n", __func__, __LINE__, signum);
	_quit = 1;
}

/*
 * \fn void underflow_clbk(void)
 * \brief This function is called by the system in case that the table underflow event occurs.
 *	 underflow event occurs when reaching the threshold of points remaining to be processed. 
 *	the _underflow flag is being set on and monitored by the function, which activates the motion procedure (see mvtable_p)
 * \return none.
 */
void underflow_clbk()
{
	fprintf(stderr, "%s: underflow event received." "\n", __func__);
	_underflow = 1;
}

/*
* \fn int init_connection()
* \breif this function creates an IPC/RPC connection with maestro and initializes axis object with accordance.
* \return 0 if completed successfully, otherwise error.
*/
int init_connection()
{
	int rc = 0;
	//MMC_LOADPARAM_OUT o_params;
	//MMC_LOADPARAM_IN i_params;

#ifndef WIN32
	struct sigaction new_action, old_action;

	/*
	 * register system signals in order to catch and handle each of them via termination_handler.
	 */
	 // set up the structure to specify the new action.
	new_action.sa_handler = termination_handler;
	sigemptyset(&new_action.sa_mask);
	new_action.sa_flags = 0;

	sigaction(SIGINT, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGINT, &new_action, NULL);
	sigaction(SIGHUP, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGHUP, &new_action, NULL);
	sigaction(SIGTERM, NULL, &old_action);
	if (old_action.sa_handler != SIG_IGN)
		sigaction(SIGTERM, &new_action, NULL);
#endif
	/*
	 * register motion events of motion library.
	 */
	CMMCPPGlobal::Instance()->SetThrowFlag(true);
	CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);
	_conn.RegisterEventCallback(MMCPP_TABLE_UNDERFLOW, (void *)underflow_clbk);
	/*
	 * creating IPC connection on the maestro and initialize the axis object with accordance.
	 */
	try {
#ifndef WIN32
		_connh = _conn.ConnectIPCEx(0x7fffffff, NULL);

#else
		char host[17];
		char dest[17];
		strcpy(host, _host);
		strcpy(dest, _dest);
		//		_connh = _conn.ConnectRPCEx(host, dest,   0x7FFFFFFF, (MMC_MB_CLBK)NULL);
		_connh = _conn.ConnectRPCEx(host, dest, 0x7fffffff, (MMC_MB_CLBK)NULL);//(MMC_MB_CLBK)events_clbck);

#endif
		fprintf(stderr, "%s: connection established OK." "\n", __func__);
		_v01.InitAxisData("v01", _connh); //for PathSelect/MovePath only.
		_conn.GetLibVersion();
	}
	catch (CMMCException& e)
	{
		fprintf(stderr, "\x1b[31m" "%s: %s, error(%d)" "\x1b[0m" "\n", __func__, e.what(), e.error());
		rc = -1;
	}
	return rc;
}

/*
* source table in host's memory from which we take points dynamicaly to append into table in maestro.
* first column is for the time. irrelevant for CV type of table.
*/
double _table_p[] = 
{	//201*3=603 items (columns). 
	0, 0, 0,
	1, 142.658477692675, 46.3525483917383,
	1, 242.705100202442, 176.335573086439,
	1, 264.503369384545, 364.057643216322,
	1, 185.41020885703, 570.633905802654,
	1, 2.00961724387903E-05, 750,
	1, -278.115267415314, 855.950873608122,
	1, -617.174483041132, 849.467867245672,
	1, -970.820363010617, 705.342344371818,
	1, -1283.92627687787, 417.173004330989,
	1, -1500, 8.03846897551611E-05,
	1, -1569.24328194369, -509.877948213699,
	1, -1456.23065791337, -1058.01336047953,
	1, -1146.18135187538, -1577.58305918044,
	1, -648.935838030138, -1997.21863553296,
	1, -0.000180865552948313, -2249.99999999999,
	1, 741.64059078689, -2282.53570269936,
	1, 1498.85220540164, -2062.99347220553,
	1, 2184.3457317258, -1587.02039189523,
	1, 2710.51098176792, -880.698709954479,
	1, 2999.99999999998, -0.000321538759020644,
	1, 2995.82813587515, 973.403195134887,
	1, 2669.75631012215, 1939.69101780746,
	1, 2027.85946443137, 2791.10838064668,
	1, 1112.46162010401, 3423.80331558277,
	1, 0.00050240431263509, 3749.99999999997,
	1, -1205.16576125769, 3709.12058147105,
	1, -2380.52979769698, 3276.51917166324,
	1, -3397.87100594309, 2468.69856948379,
	1, -4137.09563697749, 1344.22456847873,
	1, -4499.99999999994, 0.00072346221179325,
	1, -4422.41303948706, -1436.9282891553,
	1, -3883.28205682879, -2821.36854507021,
	1, -2909.53770705249, -4004.63360761502,
	1, -1575.98755507863, -4850.38794595208,
	1, -0.00098471244716925, -5249.99999999991,
	1, 1668.69077882772, -5135.70550992318,
	1, 3262.20725992714, -4490.04496561879,
	1, 4611.39618566248, -3350.37687713749,
	1, 5563.68024250655, -1807.75057990373,
	1, 5999.99999999986, -0.00128615503608257,
	1, 5848.99799277941, 1900.45323027494,
	1, 5096.80789803326, 3703.04594226777,
	1, 3791.21607973876, 5218.15874008546,
	1, 2039.51364295402, 6276.97252664088,
	1, 0.0016277899705396, 6749.9999999998,
	1, -2132.21564349697, 6562.29048805574,
	1, -4143.88459209211, 5703.57085407218,
	1, -5824.92127088396, 4232.05531485632,
	1, -6990.26479835509, 2271.27674422949,
	1, -7499.99999999973, 0.00200961725054034,
	1, -7275.58299575217, -2363.97801849379,
	1, -6310.33383373555, -4584.72320940014,
	1, -4672.89458249015, -6431.68377805798,
	1, -2503.03988373014, -7703.55705764916,
	1, -0.00243163686142985, -8249.99999999964,
	1, 2595.74035526544, -7988.87551586871,
	1, 5025.56179419189, -6917.09683702338,
	1, 7038.44626160751, -5113.73388264028,
	1, 8416.8493045231, -2734.803061456,
	1, 8999.99999999953, -0.00289384884717296,
	1, 8702.16804840535, 2827.50265381185,
	1, 7523.85986393567, 5466.40034646731,
	1, 5554.57321530667, 7645.20872153258,
	1, 2966.56627740702, 9130.1415389769,
	1, 0.00339625314648536, 9749.99999999941,
	1, -3059.2649141331, 9415.46059336208,
	1, -5907.23886622645, 8130.6229144724,
	1, -8251.97115783316, 5995.41258048933,
	1, -9843.43376101057, 3198.32953158322,
	1, -10499.9999999993, 0.00393884978867693,
	1, -10128.7531507389, -3291.02713622912,
	1, -8737.38598863359, -6348.07735346927,
	1, -6436.2519781883, -8858.73357050923,
	1, -3430.09282398464, -10556.7259706241,
	1, -0.0045216388137157, -11249.9999999991,
	1, 3522.78932009997, -10842.0457205358,
	1, 6788.9158081958, -9344.14908641922,
	1, 9465.49595956084, -6877.09140840351,
	1, 11270.0181678175, -3661.85615461121,
	1, 11999.9999999989, -0.00514462014433016,
	1, 11555.3383027529, 3754.55146574563,
	1, 9950.91220782929, 7229.75423040604,
	1, 7317.930871135, 10072.258324988,
	1, 3893.619523463, 11983.3103525907,
	1, 0.00580779386312088, 12749.9999999987,
	1, -3986.31357316605, 12268.63089739,
	1, -7670.59262009994, 10557.6753528638,
	1, -10679.0206667906, 7758.7703663828,
	1, -12696.6025249438, 4125.38293053993,
	1, -13499.9999999984, 0.00651115988215821,
	1, -12981.9235044472, -4218.07564236131,
	1, -11164.4385215228, -8111.43097727756,
	1, -8199.60989414684, -11285.7829849687,
	1, -4357.14637584205, -13409.8946848768,
	1, -0.00725471824407471, -14249.9999999982,
	1, 4449.83767333132, -13695.2161239245,
	1, 8552.26930193884, -11771.2017138062,
	1, 11892.5452795223, -8640.44945442718,
	1, 14123.1868323896, -4588.90985936939,
	1, 14999.9999999978, -0.00803846900216108,
	1, 14408.5087558219, 4681.59966607618,
	1, 12377.9649297141, 8993.10759408385,
	1, 9081.28904722381, 12499.3075504515,
	1, 4820.67338112187, 14836.4789674822,
	1, 0.0088624121084557, 15749.9999999975,
	1, -4913.36162059584, 15121.8014001393,
	1, -9433.9458537125, 12984.7281692464,
	1, -13106.0697977562, 9522.12867253661,
	1, -15549.7710901547, 5052.43694109958,
	1, -16499.9999999971, 0.00972654744571899,
	1, -15835.0940568769, -5145.12353689024,
	1, -13591.4914324031, -9874.78408082494,
	1, -9962.96833036577, -13712.8320214363,
	1, -5284.20053930236, -16263.0632004071,
	1, -0.010630875243101, -17249.9999999967,
	1, 5376.88541495943, -16548.3867260346,
	1, 10315.622275421, -14198.2547191843,
	1, 14319.5942214919, -10403.8080207112,
	1, 16976.3552982393, -5515.96417573043,
	1, 17999.9999999963, -0.0115753953886913,
	1, 17261.6794076122, 5608.64725480355,
	1, 14805.0180295899, 10756.4604375007,
	1, 10844.6477435728, 14926.3563979231,
	1, 5747.72785038369, 17689.6473836514,
	1, 0.012560107749263, 18749.9999999958,
	1, -5840.40905642233, 17974.9721016101,
	1, -11197.2985670642, 15411.7813636199,
	1, -15533.1185507298, 11285.4874989508,
	1, -18402.9394566433, 5979.49156326213,
	1, -19499.9999999953, 0.0135850125859406,
	1, -18688.264808028, -6072.17081981591,
	1, -16018.5447212744, -11638.1366641113,
	1, -11726.327286845, -16139.8806799119,
	1, -6211.25531436561, -19116.231517215,
	1, -0.0146501097708265, -20249.9999999947,
	1, 6303.93254498443, -19401.5575268659,
	1, 12078.9747286421, -16625.3081025534,
	1, 16746.6427854696, -12167.1671072554,
	1, 19829.5235653666, -6443.01910369441,
	1, 20999.9999999941, -0.0157553991547066,
	1, 20114.8502581239, 6535.6942319276,
	1, 17232.0715074567, 12519.8127606565,
	1, 12608.0069601822, 17353.4048674028,
	1, 6674.78293124838, 20542.815601098,
	1, 0.0169008810306799, 21749.9999999934,
	1, -6767.45588064556, 20828.1430018021,
	1, -12960.6507601548, 17838.8349359845,
	1, -17960.1669257114, 13048.8468456252,
	1, -21256.1076244093, 6906.54679702739,
	1, -22499.9999999927, 0.0180865552548614,
	1, -21541.4357579002, -6999.21749113846,
	1, -18445.5983881367, -13401.4887271366,
	1, -13489.6867635844, -18566.9289603956,
	1, -7138.31070103172, -21969.3996353004,
	1, -0.0193124218272511, -23249.999999992,
	1, 7230.97906340601, -22254.7285264185,
	1, 13842.3266616021, -19052.3618639134,
	1, 19173.6909714552, -13930.5267140599,
	1, 22682.6916337713, -7370.07464326123,
	1, 23999.9999999912, -0.0205784805773187,
	1, 22968.0213073568, 7462.74059744835,
	1, 19659.1253633143, 14283.1645635514,
	1, 14371.3666970517, 19780.4529588903,
	1, 7601.83862371576, 23395.9836198221,
	1, 0.0218847318407958, 24749.9999999903,
	1, -7694.50209326564, 23681.3141007152,
	1, -14724.0024329843, 20265.8888863399,
	1, -20387.2149227009, 14812.2067125598,
	1, -24109.2755934527, 7833.60264239562,
	1, -25499.9999999894, 0.0232311754524811,
	1, -24394.6069064936, -7926.26355085756,
	1, -20872.6524329898, -15164.8402699008,
	1, -15253.046760584, -20993.9768628871,
	1, -8065.36669930066, -24822.5675546631,
	1, -0.0246178112258572, -26249.9999999885,
	1, 8158.02497022427, -25107.8997246922,
	1, 15605.6780743011, -21479.4160032641,
	1, 21600.7387794486, -15693.8868411246,
	1, 25535.8595034534, -8297.1307944307,
	1, 26999.9999999874, -0.0260446395286298,
	1, 25821.1925553108, 8389.78635136577,
	1, 22086.1795971629, 16046.515846185,
	1, 16134.7269541815, 22207.5006723857,
	1, 8528.89492778609, 26249.1514398234,
	1, 0.0275116601796107, 27749.9999999864,
	1, -8621.54769428224, 26534.4853983494,
	1, -16487.3535855526, 22692.9432146861,
	1, -22814.2625416983, 16575.5670997544,
	1, -26962.4433637733, 8760.65909936666,
	1, -28499.9999999852, 0.0290188729762951,
	1, -27247.7782538081, -8853.30899897332,
	1, -23299.7068558337, -16928.1912924038,
	1, -17016.4072778438, -23421.0243873863,
	1, -8992.4233091724, -27675.735275303,
	1, -0.0305662783183633, -29249.999999984,
	1, 9085.07026543918, -27961.0711216869,
	1, 17369.0289667389, -23906.4705206056,
	1, 24027.7862094498, -17457.2474884494,
	1, 28389.0271744126, -9224.18755720311,
	1, 29999.9999999828, -0.0321538760086397,
}; //_table_p[]

/*
 * \fn void events_clbck (unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
 * \brief		this function is called by the system in case of generated event.
 *
 *  in our case we have interest in table underflow event.
 * \param buff	events buffer.
 * \param buff_size	size of buff.
 * \param lpsock socket address.
 * \return none.
*/
int events_clbck(unsigned char* buff, short buffer_size, void* lpsock)
{
	fprintf(stderr, "%s: event_id(%d), axis_ref(%d)" "\n", __func__, buff[0], buff[12]);
	// Which function ID was received ...
	switch (buff[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		printf("Emergency Event received\r\n") ;
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
	case TABLE_UNDERFLOW_EVT:
		fprintf(stderr, "%s: table underfllow event received" "\n", __func__);
		_underflow = 1;
		break;
	}
	//
	return 1;
}

/*!
* \func	int is_standstill()
* \breife	read group axis state.
*
* assuming group is enabled or maybe in motion.
*
* \return 1 if completed successfuly and group axis stand by, otherwise an error 0 (not in stand by).
*/
int is_standby()
{
	int rc = 0;
	unsigned short usErrorID, usStatus;
	unsigned long ulState;

	try
	{
		ulState = _v01.ReadStatus(usErrorID, usStatus);
		return ( (ulState & NC_GROUP_STANDBY_MASK) && (usErrorID == 0) );
	}
	catch (CMMCException& e) {
		fprintf(stderr, "%s, l=%d: %s, error=%d\n", __func__, __LINE__, e.what(), e.error());
		rc = e.error();
	}
	return (rc >= 0);
}

/*
* \fn int append_p(MC_PATH_REF pr)
* \brief this function appends number of points to a table in maestro's memory.
* 
* using auto append mechanism.
* 
* \param pr index to access the specific table. returned when table was created and initialized.
* \return 0 if completed successfully and still remnant points to add. otherwise -1 (end of table).
*/
int append_p(MC_PATH_REF pr)
{
	int rc = 0;
	uint32_t start_row = 0;							//row to begin with in table on maestro. on our case this is irrelevance since we use auto append.
	uint8_t auto_append = 1;
	uint32_t remnant = _total_rows - _rows_sent;	//calculation in terms of rows (points).

	fprintf(stderr, "%s: _rows_sent(%d), _index(%u)" "\n", __func__, _rows_sent, _index);

	if (_index >= _total_columns)
		return -1; //end of table. 

	if (remnant < _packet_rows)						//well number of rows cannot be greater than remnant;
	{
		_packet_rows = remnant;
		_packet_columns = _packet_rows * (_usDimension + 1); //+1 for the time column
	}

	cerr << "remnant = " << remnant << " _index = " << _index << " _packet_rows = " << _packet_rows << " _packet_columns = " << _packet_columns << endl;
	memcpy(_tablert, &_table_p[_index], _packet_columns *sizeof(double));
	_v01.AppendPTPoints(pr, _tablert, _packet_rows, auto_append, start_row);
	_index += _packet_columns;						//calculation on global table localy in terms of columns (single coordinate).
	_rows_sent += _packet_rows;						//incrementing the number of rows (points), which were sent.
	return rc;
}

/*
* \fn int set_p(MC_PATH_REF pr)
* \brief this function adds number of points to a table in maestro's memory.
*
* the table index (base row) to place the next packet of points is managed by user.
* one can us this function to set points into table in maestro even after the table is already full.
* one must be aware for the motion segment in the table. 
*
* \param pr index to access the specific table. returned when table was created and initialized.
* \return 0 if completed successfully and still remnant points to add. otherwise -1 (end of table).
*/
int set_p(MC_PATH_REF pr)
{
	int rc = 0;
	uint32_t start_row = _rows_sent;				//row to begin with in table on maestro. on our case this is irrelevance since we use auto append.
	uint8_t auto_append = 0;
	uint32_t remnant = _total_rows - _rows_sent;	//calculation in terms of rows (points).

	fprintf(stderr, "%s: _rows_sent(%d), _index(%u)" "\n", __func__, _rows_sent, _index);

	if (_index >= _total_columns)
		return -1; //end of table. 

	if (remnant < _packet_rows)						//well number of rows cannot be greater than remnant;
	{
		_packet_rows = remnant;
		_packet_columns = _packet_rows * (_usDimension + 1); //+1 for the time column
	}

	cerr << "remnant = " << remnant << " _index = " << _index << " _packet_rows = " << _packet_rows << " _packet_columns = " << _packet_columns << endl;
	memcpy(_tablert, &_table_p[_index], _packet_columns * sizeof(double));
	_v01.AppendPTPoints(pr, _tablert, _packet_rows, auto_append, start_row);
	_index += _packet_columns;						//calculation on global table localy in terms of columns (single coordinate).
	_rows_sent += _packet_rows;						//incrementing the number of rows (points), which were sent.
	return rc;
}

/*
* \fn int mvtable_p()
* \brief this function activates the procedure of the table (online spline table) motion.
* 
* it first clears memory in maestro from all tables. then creates and  initializes the table (online spline table).
* on next stage it adds (initialises) packet of points into that table.
* only then it activates the table motion itself and waits for the underfllow event in order to add the next packet
* of points untill the table is full. motion proceeds according to the cyclic mode.
* on periodical mode (cyclic mode on) it will proceed forever or until the group motion is stopped.
* on 'one shot' mode (cyclic mode off), motion will end when it complete the last segment in the table.
* 
* \return 0 if completed successfully. otherwise error.
*/
int mvtable_p()
{
	int rc = 0;
	uint32_t ulMaxPoints = 201;//100;
	uint32_t ulUnderflowThreshold = 10;
	unsigned char ucIsCyclic = 0;//1;// 0;
	unsigned char ucIsDynamic = 1;
	unsigned char ucIsPosAbsolute = 1;
	MC_COORD_SYSTEM_ENUM eCoordSystem = MC_MCS_COORD;
	NC_ONLINE_SPLINE_MODE_ENUM eSplineMode = MC_QUINTIC_ON_PARAB_CV_DWELL;
	double ConstVelocity = 10000.0;

	try {
		if (!is_standby())
		{
			fprintf(stderr, "\x1b[31m" "%s(%d): group is not in standby state. it must be enabled!" "\x1b[0m" "\n", __func__, __LINE__);
			return -1;
		}
		fprintf(stderr, "\x1b[33m" "%s(%d): ucIsCyclic(%d)" "\x1b[0m" "\n", __func__, __LINE__, ucIsCyclic);

		//initializing memory in maestro for p table and set parameters for points insertion procedure.
		// 
		_v01.UnloadPTTable(-1); //all
		MC_PATH_REF pr = _v01.InitPTTable(ulMaxPoints, ulUnderflowThreshold, ucIsCyclic, ucIsDynamic, ucIsPosAbsolute, _usDimension, eCoordSystem, eSplineMode, ConstVelocity);
		/*
		* points initial insertion procedure. two optional functions: auto append or management by the user.
		* it is recommended to use the same chosen option on underflow event.
		*/
		if ((rc = append_p(pr)) < 0) 
		//if ((rc = set_p(pr)) < 0)
				return 0;

		//performing  motion based on the prepared p table
		_v01.MovePT(pr, eCoordSystem);
		while (!_quit)
		{
			if (is_standby())
				break;
			if (_underflow)
			{
				_underflow = 0;
				uint32_t idx = _v01.GetPTTableIndex(pr);
				fprintf(stderr, "%s: _underflow(%d), table_index(%u)" "\n", __func__, _underflow, idx);
				/*
				* points insertion procedure. two optional functions: auto append or management by the user.
				* it is recommended to use the same chosen option on both places (initial insertion and here on the underflow event).
				*/
				rc = append_p(pr);
				//rc = set_p(pr);
			}
			else
				rc = (_rows_sent >= _total_rows) ? -1 : 0;

			if (rc)  //end if source buffer.
			{
				if (!ucIsCyclic)
				{
					rc = 0;
					break; //the while loop
				}
			}
		}
		/*
		 * is it external halt by system signal (SIGTERM, SIGINT(2 ^C), etc.).
		 */
		if ( _quit) //turns on only in the termination handler as a result of halting by system signal.
			_v01.GroupStop(100000.0, 1000000.0, MC_ABORTING_MODE);
	}
	catch (CMMCException& e)
	{
		fprintf(stderr, "%s: %s, error(%d)" "\n", __func__, e.what(), e.error());
		rc = -1;
	}

	fprintf(stderr, "%s: complete." "\n", __func__);
	return rc;
}

int main()
{
	int rc = 0;

	atexit(bye);
	if ((rc = init_connection()) != 0)
		return rc;

	try {
		mvtable_p();
	}
	catch (CMMCException& e)
	{
		fprintf(stderr, "\x1b[31m" "%s: %s, error(%d)" "\x1b[0m" "\n", __func__, e.what(), e.error());
		rc = -1;
	}

	return rc;
}
