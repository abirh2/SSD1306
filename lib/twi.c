/** 
 * --------------------------------------------------------------------------------------+
 * @desc        Two Wire Interface / I2C Communication
 * --------------------------------------------------------------------------------------+
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       06.09.2020
 * @file        twi.c
 * @tested      AVR Atmega16, ATmega8, Atmega328
 *
 * @depend      twi.h
 * --------------------------------------------------------------------------------------+
 * @usage       Master Transmit Operation
 */
 
// include libraries
#include "twi.h"

struct i2c_master_module i2c_master_instance;

/**
 * @desc    TWI init - initialize frequency
 *
 * @param   void
 *
 * @return  void
 */
void TWI_Init (void)
{
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Calculation fclk:
  //
  // fclk = (fcpu)/(16+2*TWBR*4^Prescaler) m16
  // fclk = (fcpu)/(16+2*TWBR*Prescaler) m328p
  // -------------------------------------------------------------------------------------
  // Calculation TWBR:
  // 
  // TWBR = {(fcpu/fclk) - 16 } / (2*4^Prescaler)
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // @param1 value of TWBR (m16) 
  //  fclk = 400kHz; TWBR = 3
  //  fclk = 100kHz; TWBR = 20
  // @param1 value of TWBR (m328p)
  //  fclk = 400kHz; TWBR = 2
  // @param2 value of Prescaler = 1
  
  /************ CODE BEFORE EDITS *******************/
  // TWI_FREQ (2, 1);

  /************ Code edited for SAMW25 **************/
  // Initialize the I2C master driver config
    // struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);

    // Change buffer timeout to something suitable for application
    config_i2c_master.buffer_timeout = 10000; // NEED TO CHECK
    
    // Adjust the baud rate as necessary
    config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_400KHZ; // NEED TO CHECK

    // Pinmux setting for SDA/SCL - adjust to your board's specific pinout
    // PA08 - SERCOM0/PAD[0] for SDA
    // PA09 - SERCOM0/PAD[1] for SCL
    config_i2c_master.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
    config_i2c_master.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;

    // Initialize and enable the I2C master module
    i2c_master_init(&i2c_master_instance, SERCOM0, &config_i2c_master);
    i2c_master_enable(&i2c_master_instance);
}

/**
 * @desc    TWI MT Start
 *
 * @param   void
 *
 * @return  char
 */
/************ CODE BEFORE EDITS *******************/
// char TWI_MT_Start (void)
// {
//   // null status flag
//   TWI_TWSR &= ~0xA8;
//   // START
//   // -------------------------------------------------------------------------------------
//   // request for bus
//   TWI_START();
//   // wait till flag set
//   TWI_WAIT_TILL_TWINT_IS_SET();
//   // test if start or repeated start acknowledged
//   if ((TWI_STATUS != TWI_START_ACK) && (TWI_STATUS != TWI_REP_START_ACK)) {
//     // return status
//     return TWI_STATUS;
//   }
//   // success
//   return SUCCESS;
// }
/************ Code edited for SAMW25 **************/
enum status_code TWI_MT_Start(void) {
    enum status_code status;
    
    // Create a packet to send the start condition
    struct i2c_master_packet packet = {
        .address     = 0, // The address will be set in the SLAW function
        .data_length = 0, // No data for just a start condition
        .data        = NULL,
        .ten_bit_address = false,
        .high_speed      = false,
        .hs_master_code  = 0x0,
    };
    
    // Send start condition
    status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
    
    // Check status
    return status;
}

/**
 * @desc    TWI Send address + write
 *
 * @param   char
 *
 * @return  char
 */
// char TWI_MT_Send_SLAW (char address)
// {
//   // SLA+W
//   // -------------------------------------------------------------------------------------
//   TWI_TWDR = (address << 1);
//   // enable
//   TWI_ENABLE();
//   // wait till flag set
//   TWI_WAIT_TILL_TWINT_IS_SET();

//   // test if SLA with WRITE acknowledged
//   if (TWI_STATUS != TWI_MT_SLAW_ACK) {
//     // return status
//     return TWI_STATUS;
//   }
//   // success
//   return SUCCESS;
// }
/************ Code edited for SAMW25 **************/
enum status_code TWI_MT_Send_SLAW(uint8_t slaw) {
    enum status_code status;

    struct i2c_master_packet packet = {
        .address     = slaw,
        .data_length = 0,
        .data        = NULL,
        .ten_bit_address = false,
        .high_speed      = false,
        .hs_master_code  = 0x0,
    };

    status = i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);

    return status;
}

/**
 * @desc    TWI Send data
 *
 * @param   char
 *
 * @return  char
 */
// char TWI_MT_Send_Data (char data)
// {
//   // DATA
//   // -------------------------------------------------------------------------------------
//   TWI_TWDR = data;
//   // enable
//   TWI_ENABLE();
//   // wait till flag set
//   TWI_WAIT_TILL_TWINT_IS_SET();

//   // test if data acknowledged
//   if (TWI_STATUS != TWI_MT_DATA_ACK) {
//     // return status
//     return TWI_STATUS;
//   }
//   // success
//   return SUCCESS;
// }
/************ Code edited for SAMW25 **************/
enum status_code TWI_MT_Send_Data(uint8_t data) {
    enum status_code status;

    uint8_t write_data = data;
    
    struct i2c_master_packet packet = {
        .address     = 0, // Address should be set previously
        .data_length = 1,
        .data        = &write_data,
        .ten_bit_address = false,
        .high_speed      = false,
        .hs_master_code  = 0x0,
    };

    // Write data to slave
    status = i2c_master_write_packet_wait(&i2c_master_instance, &packet);

    return status;
}

/**
 * @desc    TWI Send address + read
 *
 * @param   char
 *
 * @return  char
 */
// char TWI_MR_Send_SLAR (char address)
// {
//   // SLA+R
//   // -------------------------------------------------------------------------------------
//   TWI_TWDR = (address << 1) | 0x01;
//   // enable
//   TWI_ENABLE();
//   // wait till flag set
//   TWI_WAIT_TILL_TWINT_IS_SET();

//   // test if SLA with READ acknowledged
//   if (TWI_STATUS != TWI_MR_SLAR_ACK) {
//     // return status
//     return TWI_STATUS;
//   }
//   // success
//   return SUCCESS;
// }
enum status_code TWI_MR_Send_SLAR(uint8_t slar, uint8_t *data, uint16_t length) {
    enum status_code status;

    struct i2c_master_packet packet = {
        .address     = slar,
        .data_length = length,
        .data        = data,
        .ten_bit_address = false,
        .high_speed      = false,
        .hs_master_code  = 0x0,
    };

    // Read from the slave without sending a stop condition
    status = i2c_master_read_packet_wait_no_stop(&i2c_master_instance, &packet);

    return status;
}


/**
 * @desc    TWI stop
 *
 * @param   void
 *
 * @return  void
 */
void TWI_Stop (void)
{
  // End TWI
  // -------------------------------------------------------------------------------------
  // send stop sequence
  // TWI_STOP (); // This is what was commented out
  // wait for TWINT flag is set
//  TWI_WAIT_TILL_TWINT_IS_SET();


  // Send a stop condition
  i2c_master_send_stop(&i2c_master_instance);
}
