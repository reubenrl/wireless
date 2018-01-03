/*
 * cmd_msg.c
 *
 *  Created on: Mar 28, 2017
 *      Author: phytech
 */
#include <stdlib.h>
#include "bsp.h"
#include "uart_dbg_print.h"
#include "cmd_msg.h"
/*
 * Message format:
 * START_HDR DATAn END_HDR
 * START_HDR	- Start message packet	-	1 byte
 * DATAn		- Data payload			- 	n bytes
 * END_HDR		- End message packet	- 	1 byte
 * -------------------
 * |	DATAn		 |
 * |-----------------|
 * | CMD | , | VALUE |
 * -------------------
 * CMD , VALUE data type: ASCII String.
 * CMD		- Command		-	3 bytes
 * ,		- Separator		- 	1 byte
 * VALUE	- Value			-	n bytes
 */

#define CMD_SID_IDX			0
#define CMD_VTG_IDX			1
#define CMD_MON_IDX			2
#define CMD_ENS_IDX			3

#define __CMD_SID(idx)		strCommandsList[idx].cmd_name
#define __CMD_VTG(idx)		strCommandsList[idx].cmd_name
#define __CMD_MON(idx)		strCommandsList[idx].cmd_name
#define __CMD_ENS(idx)		strCommandsList[idx].cmd_name

#define CMD_SID				__CMD_SID(CMD_SID_IDX)
#define CMD_VTG				__CMD_VTG(CMD_SID_IDX)
#define CMD_MON				__CMD_MON(CMD_SID_IDX)
#define CMD_ENS				__CMD_ENS(CMD_SID_IDX)

#define COMMAND(NAME)  { #NAME }

struct command_name{
	char *cmd_name;
};



const struct command_name strCommandsList[] =
{
  COMMAND (SID),	/* Sensor ID	*/
  COMMAND (VTG),	/* Voltage		*/
  COMMAND (MON),	/* Monitor		*/
  COMMAND (ENS),	/* End Session	*/
  {(NULL)},
};


void cmdlisttest(void)
{
	uint8_t i;

	for(i=0;;i++){
		if(strCommandsList[i].cmd_name != NULL){
			printDbgStr(0, strCommandsList[i].cmd_name);
		}else
			break;
	}

	printDbgStr(0, CMD_SID);

}
