
/*******************************************************************************
 *
 *  Author:         Austin Schaller
 *  Module:         I2C.c
 *  Description:    Library functions that permit utilization of LPC17xx
 *                  I2C0/1/2 peripherals.
 *  Documentation:  "I2C - TLD", "The I2C-Bus Specification"
 *
*******************************************************************************/


/** Include Files *************************************************************/

// LPC17xx ARM Include Files
#include "LPC17xx.h"
#include "system_LPC17xx.h"

// Library Include File
#include "I2C.h"




/** Module Variables **********************************************************/

struct I2CPort
{
    LPC_I2C_TypeDef (* const I2CReg);       // Pointer to I2C registers
    uint32_t timeout;
    uint32_t master_state;
    uint8_t master_buffer[BUFSIZE];
    uint8_t slave_buffer[BUFSIZE];
    uint32_t read_length;
    uint32_t write_length;
    uint32_t write_index;
    uint32_t read_index;
};

typedef volatile struct I2CPort I2CPort;

I2CPort i2c_ports[I2C_PORT_NUM] =
{
    {.I2CReg = LPC_I2C0},
    {.I2CReg = LPC_I2C1},
    {.I2CReg = LPC_I2C2}
};




/** General-Purpose Functions *************************************************/

/*******************************************************************************
 *
 *  Purpose:        Commence I2C read/write operation by setting the STA flag to
 *                  change state of bus. This allows the NVIC to handle IRQ
 *                  handler cases.
 *  Passed:         port_num (eI2CPort)
 *  Returned:       State of I2C Master
 *
*******************************************************************************/
static uint8_t I2C_Engine(eI2CPort port_num)
{
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    if((port->master_state == I2C_IDLE) || (port->master_state == I2C_OK))
    {
        port->timeout = 0;      // Reset timeout
        port->master_state = I2C_BUSY;      // Set as busy while we utilize the bus
        
        port->I2CReg->I2CONSET = I2CONSET_STA;      // Set STA flag
        
        /*
         * Upon setting the start flag, the NVIC will presumably be triggered
         * because of a bus-state change. If there is a problem and the NVIC
         * never alters master_state, a timeout will occur.
         */
        while(port->master_state == I2C_BUSY)
        {
            if(port->timeout > MAX_TIMEOUT)
            {
                port->master_state = I2C_TIMEOUT;
                break;
            }
            
            port->timeout++;
        }
        
        port->I2CReg->I2CONCLR = I2CONCLR_STAC;     // Clear STA flag
    }

    return(port->master_state);
}


/*******************************************************************************
 *
 *  Purpose:        Write operation to selected I2C bus.
 *  Passed:         port_num (eI2CPort), *pWrite (pointer to write array), wr_len
 *                  (uint8_t)
 *  Returned:       State of I2C Master
 *
*******************************************************************************/
uint8_t I2C_Write(eI2CPort port_num, uint8_t *pWrite, uint32_t wr_len)
{
    uint32_t i;
    uint8_t i2c_status = I2C_IDLE;
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    port->write_length = wr_len;        // SLA+W, register address pointer, and data bytes
    port->read_length = 0;

    for(i = 0; i < wr_len; i++)
        port->master_buffer[i] = pWrite[i];

    i2c_status = I2C_Engine(port_num);

    return i2c_status;
}


/*******************************************************************************
 *
 *  Purpose:        Read operation to selected I2C bus (preceded by Write
 *                  operation due to SLA, etc.)
 *  Passed:         port_num (eI2CPort), *pWrite (pointer to write array),
 *                  *pRead (pointer to read array), wr_len (uint8_t), rd_len
 *                  (uint8_t)
 *  Returned:       State of I2C Master
 *
*******************************************************************************/
uint8_t I2C_Read(eI2CPort port_num, uint8_t *pWrite, uint8_t *pRead, \
                 uint32_t wr_len, uint32_t rd_len)
{
    uint32_t i;
    uint8_t i2c_status = I2C_IDLE;
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    port->write_length = wr_len;
    port->read_length = rd_len;

    for(i = 0; i < wr_len; i++)
        port->master_buffer[i] = pWrite[i];     // Fill the write buffer

    i2c_status = I2C_Engine(port_num);      // Initiate I2C process

    for(i = 0; i < rd_len; i++)
        pRead[i] = port->slave_buffer[i];       // Transfer contents from buffer

    return i2c_status;
}


/*******************************************************************************
 *
 *  Purpose:        Returns current status of specified I2C peripheral module.
 *  Passed:         port_num (eI2CPort)
 *  Returned:       master_state (uint8_t)
 *
*******************************************************************************/
uint8_t I2C_GetState(eI2CPort port_num)
{
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    return(port->master_state);
}


/*******************************************************************************
 *
 *  Purpose:        Returns the peripheral clock (PCLK) frequency of any I2C
 *                  peripheral module.
 *  Passed:         port_num (eI2CPort)
 *  Returned:       I2Cx PCLK Frequency (uint32_t)
 *
*******************************************************************************/
uint32_t I2C_GetPCLK(eI2CPort port_num)
{
    uint32_t i2c_pclk_div;

    switch(port_num)
    {
        case I2C0:
        {
            i2c_pclk_div = (LPC_SC->PCLKSEL0 >> 14) & 0x03;     // I2C0 PCLK
            
            break;
        }
        case I2C1:
        {
            i2c_pclk_div = (LPC_SC->PCLKSEL1 >> 6) & 0x03;      // I2C1 PCLK
            
            break;
        }
        case I2C2:
        {
            i2c_pclk_div = (LPC_SC->PCLKSEL1 >> 20) & 0x03;     // I2C2 PCLK
            
            break;
        }
    }

    switch(i2c_pclk_div)
    {
        case 0x00:
            return SystemCoreClock/4.0;
        case 0x01:
            return SystemCoreClock;
        case 0x02:
            return SystemCoreClock/2.0;
        case 0x03:
            return SystemCoreClock/8.0;
        default:
            return SystemCoreClock/4.0;
    }
}


/*******************************************************************************
 *
 *  Purpose:        Configures the peripheral clock signal that will be supplied
 *                  to any I2C peripheral module.
 *  Passed:         port_num (eI2CPort), port_setting (uint8_t)
 *  Returned:       Void
 *
 *  port_setting:
 *      - 0x00: CCLK/4
 *      - 0x01: CCLK
 *      - 0x02: CCLK/2
 *      - 0x03: CCLK/8
 *
*******************************************************************************/
void I2C_SetPCLK(eI2CPort port_num, uint8_t port_setting)
{
    uint32_t regVal;

    switch(port_num)
    {
        case I2C0:
        {
            regVal = LPC_SC->PCLKSEL0;
            regVal |= ((port_setting << 14) & 0x03);        // I2C0
            LPC_SC->PCLKSEL0 = regVal;
            
            break;
        }
        case I2C1:
        {
            regVal = LPC_SC->PCLKSEL1;
            regVal |= ((port_setting << 6) & 0x03);         // I2C1
            LPC_SC->PCLKSEL1 = regVal;
            
            break;
        }
        case I2C2:
        {
            regVal = LPC_SC->PCLKSEL1;
            regVal |= ((port_setting << 20) & 0x03);        // I2C2
            LPC_SC->PCLKSEL1 = regVal;
            
            break;
        }
    }
}


/*******************************************************************************
 *
 *  Purpose:        Returns the I2C bit frequency of selected I2C peripheral.
 *  Passed:         port_num (eI2CPort)
 *  Returned:       Void
 *  Note:           I2C Bit frequency = I2C PCLK/(SCLL + SCLH)
 *
*******************************************************************************/
uint32_t I2C_GetBitFrq(eI2CPort port_num)
{
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    return(I2C_GetPCLK(port_num)/(port->I2CReg->I2SCLL + port->I2CReg->I2SCLH));
}


/*******************************************************************************
 *
 *  Purpose:        Sets the I2C bit frequency of selected I2C peripheral (e.g.
 *                  100 kHz or 400 kHz).
 *  Passed:         port_num (eI2CPort), bit_frq (uint32_t)
 *  Returned:       Void
 *  Note:           'bit_frq' is desired bit frequency.
 *
*******************************************************************************/
void I2C_SetBitFrq(eI2CPort port_num, uint32_t bit_frq)
{
    uint32_t calc_bit_frq = I2C_GetPCLK(port_num)/bit_frq;      // Calculate bit frequency
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    if(calc_bit_frq % 2 == 0)
    {
        port->I2CReg->I2SCLL = calc_bit_frq/2;
        port->I2CReg->I2SCLH = calc_bit_frq/2;
    }
    else
    {
        port->I2CReg->I2SCLL = calc_bit_frq/2;      // I2SCLL will be floored
        port->I2CReg->I2SCLH = (calc_bit_frq/2) + 1;
    }
}




/*******************************************************************************
 *
 *  Purpose:        Initializes the selected I2C peripheral module.
 *  Passed:         port_num (eI2CPort), bit_frq (uint16_t) - usually
 *                  BIT_FRQ_100_KHZ or BIT_FRQ_400_KHZ.
 *  Returned:       Void
 *
*******************************************************************************/
void I2C_Init(eI2CPort port_num, uint32_t bit_frq)
{
    switch(port_num)
    {
        case I2C0:
        {
            LPC_SC->PCONP |= (1 << 7);      // Power to I2C0
            
            // Configure GPIO SDA/SCL
            LPC_PINCON->PINSEL1 &= ~((0x03 << 22) | (0x03 << 24));
            LPC_PINCON->PINSEL1 |= ((0x01 << 22) | (0x01 << 24));
            
            // Clear flags
            LPC_I2C0->I2CONCLR = (I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC);
            
            // Reset registers (Fast-Mode Plus not supported)
            LPC_PINCON->I2CPADCFG &= ~((0x01 << 0) | (0x01 << 2));
            
            // Set I2C bit frequency
            I2C_SetBitFrq(port_num, bit_frq);
            
            NVIC_EnableIRQ(I2C0_IRQn);      // Install interrupt handler
            
            LPC_I2C0->I2CONSET = I2CONSET_I2EN;     // Enable I2C0

            break;
        }
        case I2C1:
        {
            LPC_SC->PCONP |= (1 << 19);     // Power to I2C1
            
            // Configure GPIO SDA/SCL
            
            #if IO_MODE == 0
                // P0.19 (SDA1) & P0.20 (SCL1)
                LPC_PINCON->PINSEL1 &= ~((0x03 << 6) | (0x03 << 8));
                LPC_PINCON->PINSEL1 |= ((0x03 << 6) | (0x03 << 8));
                LPC_PINCON->PINMODE1 &= ~((0x03 << 6) | (0x03 << 8));
                LPC_PINCON->PINMODE1 |= ((0x02 << 6) | (0x02 << 8));        // No pull up or pull down
                LPC_PINCON->PINMODE_OD0 |= ((0x01 << 19) | (0x01 << 20));
            #endif
            
            #if IO_MODE == 1
                // P0.0 (SDA1) & P0.1 (SCL1)
                LPC_PINCON->PINSEL0 &= ~((0x03 << 0) | (0x03 << 2));
                LPC_PINCON->PINSEL0 |= ((0x03 << 0) | (0x03 << 2));
                LPC_PINCON->PINMODE0 &= ~((0x03 << 0) | (0x03 << 2));
                LPC_PINCON->PINMODE0 |= ((0x02 << 0) | (0x02 << 2));        // No pull up or pull down
                LPC_PINCON->PINMODE_OD0 |= ((0x01 << 0) | (0x01 << 1));     // Open drain
            #endif
            
            // Clear flags
            LPC_I2C1->I2CONCLR = (I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC);
            
            // Set I2C bit frequency
            I2C_SetBitFrq(I2C1, bit_frq);
            
            NVIC_EnableIRQ(I2C1_IRQn);      // Install interrupt handler
            
            LPC_I2C1->I2CONSET = I2CONSET_I2EN;     // Enable I2C1

            break;
        }
        case I2C2:
        {
            //LPC_SC->PCONP |= (1 << 26);       // Power to I2C2
            
            /*
             * Set P0.10 and P0.11 to I2C2 SDA and SCL function to 0x10 on both SDA and
             * SCL.
             */
            
            // Set P0.10 (SDA2) and P0.11 (SCL2) to 0x02
            LPC_PINCON->PINSEL0 &= ~((0x03 << 20) | (0x03 << 22));
            LPC_PINCON->PINSEL0 |= ((0x02 << 20) | (0x02 << 22));
            LPC_PINCON->PINMODE0 &= ~((0x03 << 20) | (0x03 << 22));
            LPC_PINCON->PINMODE0 |= ((0x02 << 20) | (0x02 << 22));      // No pull-up or pull-down
            LPC_PINCON->PINMODE_OD0 |= ((0x01 << 10) | (0x01 << 11));
            
            // Clear flags
            LPC_I2C2->I2CONCLR = (I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC);
            
            // Set I2C bit frequency
            I2C_SetBitFrq(I2C2, bit_frq);
            
            NVIC_EnableIRQ(I2C2_IRQn);      // Install interrupt handler
            
            LPC_I2C2->I2CONSET = I2CONSET_I2EN;     // Enable I2C2

            break;
        }
    }
}




/** I2C Interrupt Handlers ****************************************************/

/*******************************************************************************
 *
 *  Purpose:        Function that executes accommodating I2C routines whenever
 *                  the state of the I2C bus changes.
 *  Passed:         Void
 *  Returned:       Void
 *  Note:           I2C is in master mode only.
 *  Documentation:  "UM10360 LPC17xx User Manual" - 19.9.5 Detailed State
 *                  Tables (pg. 457/840).
 *
*******************************************************************************/
static void I2CIntHandler(uint8_t port_num)
{
    uint8_t status;
    I2CPort *port = &i2c_ports[port_num];       // Inherit port

    port->timeout = 0;      // Reset timeout

    status = port->I2CReg->I2STAT;      // Obtain current I2C status

    switch(status)
    {
        /*
         * A START condition has been transmitted and SLA+W is ready to be
         * loaded. Clear the SI interrupt flag and STA flag on exit.
         */
        case 0x08:
        {
            port->write_index = 0;
            port->I2CReg->I2DAT = port->master_buffer[port->write_index++];
            port->I2CReg->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);        // Clear SI interrupt and STA flags

            break;
        }
        
        /*
         * A Repeated START condition has been transmitted (functionally
         * identical to START); SLA+R is ready to be loaded. Clear the SI
         * interrupt flag and STA flag on exit.
         */
        case 0x10:
        {
            port->read_index = 0;
            port->I2CReg->I2DAT = port->master_buffer[port->write_index++];
            port->I2CReg->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);        // Clear SI interrupt and STA flags
            
            break;
        }
        
        /*
         * SLA+W has been transmitted and an ACK was received. A data byte is
         * ready to be transmitted from the master_buffer.
         */
        case 0x18:
        {
            /*
             * If only one byte is left in the buffer then it is the SLA. Send a
             * STOP condition and change the I2C status to indicate that no data
             * is available.
             */
            if(port->write_length == 1)
            {
                port->I2CReg->I2CONSET = I2CONSET_STO;      // Send STOP condition
                port->master_state = I2C_NO_DATA;
            }
            else
            {
                port->I2CReg->I2DAT = port->master_buffer[port->write_index++];     // Load a new data byte
            }

            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }

        /*
         * SLA+W has been transmitted and a NACK has been received. This allows
         * the I2C master and slave to exit cleanly.
         */
        case 0x20:
            break;

        /*
         * A data byte has been transmitted and an ACK was received. A new data
         * byte can be loaded if applicable, a Repeated START condition can be
         * transmitted, or a STOP condition can be transmitted. Clear the SI
         * interrupt flag on exit.
         */
        case 0x28:
        {
            if(port->write_index < port->write_length)
            {
                port->I2CReg->I2DAT = port->master_buffer[port->write_index++];     // Load a new data byte
            }
            else
            {
                if(port->read_length != 0)
                {
                    port->I2CReg->I2CONSET = I2CONSET_STA;      // Send Repeated START condition
                    port->master_state = I2C_REPEATED_START;
                }
                else
                {
                    port->I2CReg->I2CONSET = I2CONSET_STO;      // Send STOP condition
                    port->master_state = I2C_OK;
                }
            }
            
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }

        /*
         * A data byte has been transmitted and a NACK was received. Send a STOP
         * condition and clear the SI interrupt flag on exit.
         */
        case 0x30:
        {
            port->I2CReg->I2CONSET = I2CONSET_STO;      // Send STOP condition
            port->master_state = I2C_NACK_ON_DATA;
            
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }
        
        /*
         * Arbitration was lost in SLA+R/W or data bytes. In this library,
         * multiple masters are not supported.
         */
        case 0x38:
            break;
        
        /*
         * SLA+R has been transmitted and an ACK was received. Assert an ACK
         * or NACK after the data has been received automatically from the
         * slave. Clear the SI interrupt flag on exit.
         */
        case 0x40:
        {
            /*
             * If the read index plus one is less than the total read length,
             * there is more than one byte to receive (assert ACK afterward). If
             * the read index plus one is equal to the total read length, then
             * only one byte will be received (assert NACK afterward).
             */
            if((port->read_index + 1) < port->read_length)
            {
                // Will go to state 0x50
                port->I2CReg->I2CONSET = I2CONSET_AA;       // Assert ACK after data is received
            }
            
            /*
             * There is one byte left (assert a NACK).
             */
            else
            {
                // Will go to state 0x58
                port->I2CReg->I2CONCLR = I2CONCLR_AAC;  // Assert NACK after data is received
            }
            
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }
        
        /*
         * SLA+R has been transmitted and a NACK has been received. Send a STOP
         * condition and clear the SI interrupt flag on exit.
         */
        case 0x48:
        {
            port->I2CReg->I2CONSET = I2CONSET_STO;      // Send STOP condition
            port->master_state = I2C_NACK_ON_ADDRESS;
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }
        
        /*
         * A fresh data byte has been received. Read from the I2DAT register to
         * retrieve its contents and assert an ACK/NACK after it has been
         * received. Clear the SI interrupt flag on exit.
         */
        case 0x50:
        {
            port->slave_buffer[port->read_index++] = port->I2CReg->I2DAT;
            
            /*
             * If the read index plus one is less than the total read length,
             * there is more data to receive (assert an ACK). If the read index
             * plus one is equal to the total read length, then there is one
             * last byte to receive (assert a NACK).
             */
            if((port->read_index + 1) < port->read_length)
            {
                port->I2CReg->I2CONSET = I2CONSET_AA;       // Assert ACK after data is received
            }
            
            /*
             * There is one byte left (assert a NACK).
             */
            else
            {
                port->I2CReg->I2CONCLR = I2CONCLR_AAC;      // Assert NACK on last byte
            }
            
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }

        /*
         * A fresh data byte has been received and a NACK was returned. Read
         * from the I2DAT register to retrieve its contents and send a STOP
         * condition. Clear the SI interrupt flag on exit.
         */
        case 0x58:
        {
            port->slave_buffer[port->read_index++] = port->I2CReg->I2DAT;       // Load a new data byte
            port->master_state = I2C_OK;
            port->I2CReg->I2CONSET = I2CONSET_STO;      // Send STOP condition
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }
        
        /*
         * Default to "arbitration lost" and reset SI interrupt flag.
         */
        default:
        {
            port->master_state = I2C_ARBITRATION_LOST;
            port->I2CReg->I2CONCLR = I2CONCLR_SIC;      // Clear SI interrupt flag
            
            break;
        }
    }
}


/*******************************************************************************
 *
 *  Purpose:        Interrupt handler for I2C0 peripheral. Whenever the
 *                  I2C0-bus state changes, this handler is called and
 *                  controlled by the NVIC.
 *  Passed:         Void
 *  Returned:       Void
 *  Note:           I2C is in master mode only.
 *
*******************************************************************************/
void I2C0_IRQHandler(void)
{
    I2CIntHandler(I2C0);
}


/*******************************************************************************
 *
 *  Purpose:        Interrupt handler for I2C1 peripheral. Whenever the
 *                  I2C1-bus state changes, this handler is called and
 *                  controlled by the NVIC.
 *  Passed:         Void
 *  Returned:       Void
 *  Note:           I2C is in master mode only.
 *
*******************************************************************************/
void I2C1_IRQHandler(void)
{
    I2CIntHandler(I2C1);
}


/*******************************************************************************
 *
 *  Purpose:        Interrupt handler for I2C2 peripheral. Whenever the
 *                  I2C2-bus state changes, this handler is called and
 *                  controlled by the NVIC.
 *  Passed:         Void
 *  Returned:       Void
 *  Note:           I2C is in master mode only.
 *
*******************************************************************************/
void I2C2_IRQHandler(void)
{
    I2CIntHandler(I2C2);
}
