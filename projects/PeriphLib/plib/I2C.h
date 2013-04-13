
/*******************************************************************************
 *
 *	Author:			Austin Schaller
 *	Module:			I2C.h
 *	Description:	I2C0/1/2 header file for the LPC17xx ARM microprocessor.
 *	Documentation: 	"I2C - TLD", "The I2C-Bus Specification"
 *
*******************************************************************************/

#ifndef     I2C_H
#define     I2C_H



/** Defines *******************************************************************/
/*
 * TODO: Retrieve these settings from on-chip flash.
 */
#define		I2C_PORT_NUM		3
#define		IO_MODE				1
#define		BUFSIZE				64
#define		MAX_TIMEOUT			0x005B8D80

#define		RD_BIT				0x01
#define		BIT_FRQ_100_KHZ		100E3
#define		BIT_FRQ_400_KHZ		400E3


// I2C State Defines
enum
{
    I2C_IDLE,				// State 0
    I2C_STARTED,			// State 1
    I2C_RESTARTED,			// State 2
    I2C_REPEATED_START,		// State 3
    DATA_ACK,				// State 4
    DATA_NACK,				// State 5
    I2C_BUSY,				// State 6
    I2C_NO_DATA,			// State 7
    I2C_NACK_ON_ADDRESS,	// State 8
    I2C_NACK_ON_DATA,		// State 9
    I2C_ARBITRATION_LOST,	// State 10
    I2C_TIMEOUT,			// State 11
    I2C_OK					// State 12
};


// I2C Register Configuration Defines
enum
{
    // I2C Control Set Register
    I2CONSET_I2EN = (0x01 << 6),
    I2CONSET_AA = (0x01 << 2),
    I2CONSET_SI = (0x01 << 3),
    I2CONSET_STO = (0x01 << 4),
    I2CONSET_STA = (0x01 << 5),

    // I2C Control Clear Register
    I2CONCLR_AAC = (0x01 << 2),
    I2CONCLR_SIC = (0x01 << 3),
    I2CONCLR_STAC = (0x01 << 5),
    I2CONCLR_I2ENC = (0x01 << 6),

    // I2C Data/SLA Register
    I2DAT_I2C = 0x00000000,			// I2C Data Register
    I2ADR_I2C = 0x00000000,			// I2C Slave Address Register
};




/** I2C Ports *****************************************************************/

/*
 * eI2CPort restricts input arguments to the number of I2C ports available and
 * can also act as separate defines when accessed (e.g. myVar = I2C0).
 */
typedef enum
{
    I2C0,
    I2C1,
    I2C2
} eI2CPort;




/** Library Functions *********************************************************/
uint8_t I2C_Write(eI2CPort port_num, uint8_t *pWrite, uint32_t wr_len);
uint8_t I2C_Read(eI2CPort port_num, uint8_t *pWrite, uint8_t *pRead, \
				 uint32_t wr_len, uint32_t rd_len);
uint8_t I2C_GetState(eI2CPort port_num);
uint32_t I2C_GetPCLK(eI2CPort port_num);
void I2C_SetPCLK(eI2CPort port_num, uint8_t port_setting);
uint32_t I2C_GetBitFrq(eI2CPort port_num);
void I2C_SetBitFrq(eI2CPort port_num, uint32_t bit_frq);
void I2C_Init(eI2CPort port_num, uint32_t bit_frq);




/** I2C Interrupt Handlers ****************************************************/
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void I2C2_IRQHandler(void);



#endif      // I2C_H
