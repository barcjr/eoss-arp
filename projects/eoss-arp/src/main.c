
/*******************************************************************************
 *
 *	Author:			Austin Schaller
 *	Module:			main.c
 *	Description:	Main entry point for eoss-arp project.
 *
*******************************************************************************/

// LPC17xx ARM Include Files
#include "LPC17xx.h"
#include <cr_section_macros.h>
#include <NXP/crp.h>

//Library Include Files
// TODO: Provide future library files.




/*
 * Variable to store CRP value in. Will be placed automatically
 * by the linker when "Enable Code Read Protect selected.
 * See crp.h header for more information.
 */
__CRP const unsigned int CRP_WORD = CRP_NO_CRP;




int main(void)
{
	// TODO: Insert code here
	
	return 0;
}
