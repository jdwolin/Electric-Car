/************************************
REVISION HISTORY
$Revision: 2000 $
$Date: 2017-4-14

Copyright (c) 2015, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2017 Linear Technology Corp. (LTC)
***********************************************************/

/*! @file
    @ingroup LTC68131
    Header for LTC6813-1 Multicell Battery Monitor
*/

#ifndef LTC6813_H
#define LTC6813_H

#include "stdint.h"
#include "LTC681x.h"

#define CELL 1
#define AUX 2
#define STAT 3


/*! Helper function to intialize register limits */
void LTC6813_init_reg_limits(uint8_t total_ic, 
							cell_asic *ic);

/*! Helper Function to initialize the CFGR data structures*/
void LTC6813_init_cfg(uint8_t total_ic, 
                      cell_asic *ic);

/*! Helper function to set appropriate bits in CFGR register based on bit function*/
void LTC6813_set_cfgr(uint8_t nIC, 
                      cell_asic *ic, 
                      bool refon, 
                      bool adcopt, 
                      bool gpio[5],
                      bool dcc[12],
					  bool dcto[4], 
					  uint16_t uv,
					  uint16_t  ov);

/*! Helper function to turn the refon bit HIGH or LOW*/
void LTC6813_set_cfgr_refon(uint8_t nIC,
                            cell_asic *ic,
                            bool refon);
                            
/*! Helper function to turn the ADCOPT bit HIGH or LOW*/
void LTC6813_set_cfgr_adcopt(uint8_t nIC,
                             cell_asic *ic, 
                             bool adcopt);

/*! Helper function to turn the GPIO bits HIGH or LOW*/
void LTC6813_set_cfgr_gpio(uint8_t nIC, 
                           cell_asic *ic,
                           bool gpio[]);

/*! Helper function to turn the DCC bits HIGH or LOW*/
void LTC6813_set_cfgr_dis(uint8_t nIC, 
                          cell_asic *ic,
                          bool dcc[]); 
/*!  Helper function to set uv field in CFGRA register*/
void LTC6813_set_cfgr_uv(uint8_t nIC, 
                         cell_asic *ic,
                         uint16_t uv);
                         
/*!  Helper function to set ov field in CFGRA register*/
void LTC6813_set_cfgr_ov(uint8_t nIC, 
                         cell_asic *ic,
                         uint16_t ov);

/*! Helper Function to initialize the CFGR B data structures*/
void LTC6813_init_cfgb(uint8_t total_ic, 
                      cell_asic *ic);
/*! Helper function to set appropriate bits in CFGR register based on bit function*/
void LTC6813_set_cfgrb(uint8_t nIC, 
                      cell_asic *ic, 
					  bool fdrf,
                      bool dtmen, 
                      bool ps[2],
                      bool gpiobits[4],
					  bool dccbits[7]);
/*! Helper function to turn the fdrf bit HIGH or LOW*/
void LTC6813_set_cfgrb_fdrf(uint8_t nIC,
                            cell_asic *ic,
                            bool fdrf);
/*! Helper function to turn the DTMEN bit HIGH or LOW*/
void LTC6813_set_cfgrb_dtmen(uint8_t nIC,
                            cell_asic *ic,
                            bool dtmen);
/*! Helper function to turn the Path Select bit HIGH or LOW*/
void LTC6813_set_cfgrb_ps(uint8_t nIC,
                            cell_asic *ic,
                            bool ps[]);
/*! Helper function to turn the GPIO bit HIGH or LOW*/
void LTC6813_set_cfgrb_gpio_b(uint8_t nIC,
                            cell_asic *ic,
                            bool gpiobits[]);
/*! Helper function to turn the dccbit bit HIGH or LOW*/
void LTC6813_set_cfgrb_dcc_b(uint8_t nIC,
                            cell_asic *ic,
                            bool dccbits[]); 
							
/*!  Write the LTC6813 configuration register */
void LTC6813_wrcfg(uint8_t nIC, //!< The number of ICs being written
                   cell_asic *ic //!< a two dimensional array of the configuration data that will be written
                   );

void LTC6813_wrcfgb(uint8_t nIC, //!< The number of ICs being written
                   cell_asic *ic //!< a two dimensional array of the configuration data that will be written
                    );
/*!  Reads configuration registers of a LTC6813 daisy chain
@return int8_t, PEC Status.
 0: Data read back has matching PEC
-1: Data read back has incorrect PEC */
int8_t LTC6813_rdcfg(uint8_t nIC, //!< number of ICs in the daisy chain
                     cell_asic *ic //!< a two dimensional array that the function stores the read configuration data
                    );

int8_t LTC6813_rdcfgb(uint8_t nIC, //!< number of ICs in the daisy chain
                     cell_asic *ic //!< a two dimensional array that the function stores the read configuration data
                    );
/*! Starts cell voltage conversion */
void LTC6813_adcv(uint8_t MD, //!< ADC Conversion Mode
                  uint8_t DCP, //!< Controls if Discharge is permitted during conversion
                  uint8_t CH //!< Sets which Cell channels are converted
                 );
/*!  Reads and parses the LTC6813 cell voltage registers.
@return int8_t, PEC Status.
0: No PEC error detected
-1: PEC error detected, retry read 
*/
uint8_t LTC6813_rdcv(uint8_t reg, //!< controls which cell voltage register is read back.
                     uint8_t total_ic, //!< the number of ICs in the daisy chain(-1 only)
                     cell_asic *ic //!< array of the parsed cell codes from lowest to highest.
                    );
/*!  Start a GPIO and Vref2 Conversion */
void LTC6813_adax(uint8_t MD, //!< ADC Conversion Mode
				  uint8_t CHG //!< Sets which GPIO channels are converted
                  );					
/*!  Reads and parses the LTC6813 auxiliary registers.
@return  int8_t, PEC Status
  0: No PEC error detected
 -1: PEC error detected, retry read 
 */
int8_t LTC6813_rdaux(uint8_t reg,        //!< controls which GPIO voltage register is read back
                     uint8_t nIC,        //!< the number of ICs in the daisy chain
                     cell_asic *ic //!< A two dimensional array of the parsed gpio voltage codes
                    );
/*!  Start a Status ADC Conversion */
void LTC6813_adstat( uint8_t MD, //!< ADC Conversion Mode
					 uint8_t CHST //!< Sets which Stat channels are converted
);					

/*!  Reads and parses the LTC6813 stat registers.
@return  int8_t, PEC Status
  0: No PEC error detected
 -1: PEC error detected, retry read
*/
int8_t LTC6813_rdstat(uint8_t reg, //Determines which Stat  register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      cell_asic *ic
                     );	
					   
/*!  Starts cell voltage  and GPIO 1&2 conversion */
void LTC6813_adcvax(uint8_t MD, //!< ADC Conversion Mode
					uint8_t DCP //!< Controls if Discharge is permitted during conversion
					);

/*!  Starts cell voltage and SOC conversion */
void LTC6813_adcvsc(uint8_t MD, //!< ADC Conversion Mode
					uint8_t DCP //!< Controls if Discharge is permitted during conversion
					);
					
/*! Starts the Mux Decoder diagnostic self test
 Running this command will start the Mux Decoder Diagnostic Self Test
 This test takes roughly 1mS to complete. The MUXFAIL bit will be updated,
 the bit will be set to 1 for a failure and 0 if the test has been passed.
*/
void LTC6813_diagn();

/*!  Starts cell voltage self test conversion */
void LTC6813_cvst(uint8_t MD, //!< ADC Conversion Mode
				  uint8_t ST //!< Self Test Mode
				 );
				 
/*!  Start an Auxiliary Register Self Test Conversion */
void LTC6813_axst(uint8_t MD, //!< ADC Conversion Mode
				  uint8_t ST //!< Sets if self test 1 or 2 is run
				 );
				 
/*!  Start a Status Register Self Test Conversion */
void LTC6813_statst(uint8_t MD, //!< ADC Conversion Mode
					uint8_t ST //!< Sets if self test 1 or 2 is run
					);
					
/*! Helper function that runs the ADC Self Tests*/
int16_t LTC6813_run_cell_adc_st(uint8_t adc_reg,
                                uint8_t total_ic,
                                cell_asic *ic,
								uint8_t md,
								bool adcopt);
								
/*!  Starts cell voltage overlap conversion */
void LTC6813_adol(uint8_t MD, //!< ADC Conversion Mode
				  uint8_t DCP //!< Discharge permitted during conversion
				 );	
				 
/*! Helper Function that runs the ADC Overlap test*/
uint16_t LTC6813_run_adc_overlap(uint8_t total_ic, 
                                 cell_asic *ic);
/*!  Start an open wire Conversion
*/
void LTC6813_adow(uint8_t MD, //!< ADC Conversion Mode
				  uint8_t PUP,//!< Controls if Discharge is permitted during 
				  uint8_t CH,
				  uint8_t DCP
				 );	
				 
/*! start GPIOs open wire adc conversion */
void LTC6813_axow(uint8_t MD, //ADC Mode
				  uint8_t PUP //Discharge Permit
				  );
				  
/*! Helper function that runs the datasheet open wire algorithm*/
void LTC6813_run_openwire(uint8_t total_ic, 
                          cell_asic *ic
						  );

/*!  Start an GPIO Redundancy test */
void LTC6813_adaxd(uint8_t MD, //!< ADC Conversion Mode
				   uint8_t CHG //!< Sets which GPIO channels are converted
				   );

/*!   Start a Status register redundancy test Conversion */
void LTC6813_adstatd(uint8_t MD, //!< ADC Mode
					 uint8_t CHST //!< Sets which Status channels are converted
					);	
					
/*! Helper function that runs the ADC Digital Redudancy commands and checks output for errors*/
int16_t LTC6813_run_adc_redundancy_st(uint8_t adc_mode, 
                                      uint8_t adc_reg, 
                                      uint8_t total_ic, 
                                      cell_asic *ic
									  );
									  
//! Sends the poll adc command
//! @returns 1 byte read back after a pladc command. If the byte is not 0xFF ADC conversion has completed
uint8_t LTC6813_pladc();

//! This function will block operation until the ADC has finished it's conversion
//! @returns the approximate time it took for the ADC function to complete.
uint32_t LTC6813_pollAdc();

/*!  Clears the LTC6813 cell voltage registers */
void LTC6813_clrcell();

/*! Clears the LTC6813 Auxiliary registers */
void LTC6813_clraux();

/*!  Clears the LTC6813 Stat registers */
void LTC6813_clrstat();

/*!  Write the LTC6813 PWM register */
void LTC6813_wrpwm(uint8_t nIC, //!< number of ICs in the daisy chain
                   uint8_t pwmReg,//!  PWM  Register A or B
                   cell_asic *ic
                  );
				  
/*!  Reads pwm registers of a LTC6813 daisy chain */
int8_t LTC6813_rdpwm(uint8_t nIC, //!< number of ICs in the daisy chain
                     uint8_t pwmReg, //! PWM  Register A or B
                     cell_asic *ic 
                    );

/*!  Write the LTC6813 Sctrl register */
void LTC6813_wrsctrl(uint8_t nIC, //!< number of ICs in the daisy chain
                     uint8_t sctrl_reg,//! SCTRL  Register A or B
                     cell_asic *ic
                    );

/*!  Reads sctrl registers of a LTC6813 daisy chain
@return int8_t, PEC Status.
  0: Data read back has matching PEC
  -1: Data read back has incorrect PEC
*/
int8_t LTC6813_rdsctrl(uint8_t nIC, //!< number of ICs in the daisy chain
                       uint8_t sctrl_reg,//! SCTRL  Register A or B
                       cell_asic *ic 
                      );
					  
/*!  Start Sctrl data communication
This command will start the sctrl pulse communication over the spins
*/
void LTC6813_stsctrl();

/*!  Clears the LTC6813 Sctrl registers */
void LTC6813_clrsctrl();

/*!  Write the LTC6813 COMM register */
void LTC6813_wrcomm(uint8_t total_ic, //!< Number of ICs in the daisy chain
                    cell_asic *ic //!< A two dimensional array of the comm data that will be written
                   );

/*!  Reads comm registers of a LTC6813 daisy chain
@return int8_t, PEC Status.

  0: Data read back has matching PEC

  -1: Data read back has incorrect PEC

*/
int8_t LTC6813_rdcomm(uint8_t total_ic, //!< number of ICs in the daisy chain
                      cell_asic *ic //!< Two dimensional array that the function stores the read comm data.
                     );

/*!  issues a stcomm command and clocks data out of the COMM register */
void LTC6813_stcomm();

/*! Helper Function to Set DCC bits in the CFGR Registers*/                   
void LTC6813_set_discharge(int Cell, 
                           uint8_t total_ic, 
                           cell_asic *ic);
/*! Helper Function to clear DCC bits in the CFGR Registers*/
void LTC6813_clear_discharge(uint8_t total_ic,
                     cell_asic *ic);                   
                                 
/*! Helper Function that counts overall PEC errors and register/IC PEC errors*/
void LTC6813_check_pec(uint8_t total_ic,
                       uint8_t reg, 
                       cell_asic *ic);

/*! Helper Function that resets the PEC error counters */
void LTC6813_reset_crc_count(uint8_t total_ic,
                             cell_asic *ic);


/*! Write the 6813 PWM/S ctrl Register B */
void LTC6813_wrpsb(uint8_t total_ic,
					cell_asic *ic);
					
/*! reading pwm/s control register B */
uint8_t LTC6813_rdpsb(uint8_t total_ic, //!< number of ICs in the daisy chain
                       cell_asic *ic //!< a two dimensional array that the function stores the read pwm data
                      );						
/*! Mutes the LTC6813 discharge transistors */
void LTC6813_mute();
/*! Clears the LTC6813 Mute Discharge */
void LTC6813_unmute();

#endif
