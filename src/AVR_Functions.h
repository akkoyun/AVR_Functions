/* *******************************************************************************
 *  Copyright (C) 2014-2023 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun.
 *
 *	Library				: AVR_Functions
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *
 *********************************************************************************/

#ifndef __AVR_Functions__
#define __AVR_Functions__

// Define B107AA Module
#define B108AA

	// Define LED Colors
	#define WHITE 	0
	#define RED 	1
	#define GREEN 	2
	#define BLUE 	3

	// Define Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Include Pin Out Definitions
	#ifndef __PinOut__
		#include "PinOut/PinOut.h"
	#endif

	// Define AVR Library
	class AVR_Functions {

		// Public Context
		public:

			// Module Pin Definitions
			inline void PinOut(void) {

				// B107AA Module
				#ifdef B107AA

					// Sample
					// DDRB |= (1 << DDB3);     // set pin 3 of Port B as output
					// PORTB |= (1 << PB3);     // set pin 3 of Port B high
					// PORTB &= ~(1 << PB3);    // set pin 3 of Port B low
					// PORTB |= (1 << PORTB3);  // set pin 3 high again

					// Set RELAY_START as Output with Pull-Down
					DDR_RELAY_START |= (1 << PIN_RELAY_START);
					PORT_RELAY_START &= ~(1 << PIN_RELAY_START);

					// Set RELAY_STOP as Output with Pull-Down
					DDR_RELAY_STOP |= (1 << PIN_RELAY_STOP);
					PORT_RELAY_STOP &= ~(1 << PIN_RELAY_STOP);

					// Set INT_ENERGY_1 as Input with Pull-Up
					DDR_INT_ENERGY_1 &= ~(1 << PIN_INT_ENERGY_1);
					PORT_INT_ENERGY_1 |= (1 << PIN_INT_ENERGY_1);

					// Set INT_ENERGY_2 as Input with Pull-Up
					DDR_INT_ENERGY_2 &= ~(1 << PIN_INT_ENERGY_2);
					PORT_INT_ENERGY_2 |= (1 << PIN_INT_ENERGY_2);

					// Set INT_ENV as Input with Pull-Up
					DDR_INT_ENV &= ~(1 << PIN_INT_ENV);
					PORT_INT_ENV |= (1 << PIN_INT_ENV);

					// Set INT_RTC as Input with Pull-Up
					DDR_INT_RTC &= ~(1 << PIN_INT_RTC);
					PORT_INT_RTC |= (1 << PIN_INT_RTC);

					// Set 3V8_EN as Output with Pull-Down
					DDR_3V8_EN |= (1 << PIN_3V8_EN);
					PORT_3V8_EN &= ~(1 << PIN_3V8_EN);

					// Set RS485_DIR as Output with Pull-Down
					DDR_RS485_DIR |= (1 << PIN_RS485_DIR);
					PORT_RS485_DIR &= ~(1 << PIN_RS485_DIR);

					// Set INT_RS485 as Input with Pull-Down
					DDR_INT_RS485 &= ~(1 << PIN_INT_RS485);
					PORT_INT_RS485 &= ~(1 << PIN_INT_RS485);

					// Set INT_CHARGER as Input with Pull-Up
					DDR_INT_CHARGER &= ~(1 << PIN_INT_CHARGER);
					PORT_INT_CHARGER |= (1 << PIN_INT_CHARGER);

					// Set INT_GAUGE as Input with Pull-Up
					DDR_INT_GAUGE &= ~(1 << PIN_INT_GAUGE);
					PORT_INT_GAUGE |= (1 << PIN_INT_GAUGE);

					// Set 3V3_BUZZER as Output with Pull-Down
					DDR_3V3_BUZZER |= (1 << PIN_3V3_BUZZER);
					PORT_3V3_BUZZER &= ~(1 << PIN_3V3_BUZZER);

					// Set LCD_SENSE as Input with Pull-Down
					DDR_LCD_SENSE &= ~(1 << PIN_LCD_SENSE);
					PORT_LCD_SENSE &= ~(1 << PIN_LCD_SENSE);

					// Set FOTA_POWER_EN as Output with Pull-Down
					DDR_FOTA_POWER_EN |= (1 << PIN_FOTA_POWER_EN);
					PORT_FOTA_POWER_EN &= ~(1 << PIN_FOTA_POWER_EN);

					// Set SD_SENSE as Input with Pull-Down
					DDR_SD_SENSE &= ~(1 << PIN_SD_SENSE);
					PORT_SD_SENSE &= ~(1 << PIN_SD_SENSE);

					// Set SD_EN as Output with Pull-Down
					DDR_SD_EN |= (1 << PIN_SD_EN);
					PORT_SD_EN &= ~(1 << PIN_SD_EN);

					// Set TERMINAL_SENSE as Input with Pull-Down
					DDR_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE);
					PORT_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE);

					// Set GSM_RING as Input with Pull-Down
					DDR_GSM_RING &= ~(1 << PIN_GSM_RING);
					PORT_GSM_RING &= ~(1 << PIN_GSM_RING);

					// Set GSM_PMON as Input with Pull-Down
					DDR_GSM_PMON &= ~(1 << PIN_GSM_PMON);
					PORT_GSM_PMON &= ~(1 << PIN_GSM_PMON);

					// Set GSM_SWREADY as Input with Pull-Down
					DDR_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY);
					PORT_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY);

					// Set GSM_COMM_EN as Output with Pull-Up
					DDR_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);
					PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);

					// Set GSM_ONOFF as Output with Pull-Down
					DDR_GSM_ONOFF |= (1 << PIN_GSM_ONOFF);
					PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF);

					// Set GSM_SDOWN as Output with Pull-Down
					DDR_GSM_SDOWN |= (1 << PIN_GSM_SDOWN);
					PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN);

					// Set 3V3_Sense_1 as Input with Pull-Down
					DDR_3V3_Sense_1 &= ~(1 << PIN_3V3_Sense_1);
					PORT_3V3_Sense_1 &= ~(1 << PIN_3V3_Sense_1);

					// Set 3V3_Sense_2 as Input with Pull-Down
					DDR_3V3_Sense_2 &= ~(1 << PIN_3V3_Sense_2);
					PORT_3V3_Sense_2 &= ~(1 << PIN_3V3_Sense_2);

					// Set 3V3_Sense_3 as Input with Pull-Down
					DDR_3V3_Sense_3 &= ~(1 << PIN_3V3_Sense_3);
					PORT_3V3_Sense_3 &= ~(1 << PIN_3V3_Sense_3);

					// Set 3V3_Sense_4 as Input with Pull-Down
					DDR_3V3_Sense_4 &= ~(1 << PIN_3V3_Sense_4);
					PORT_3V3_Sense_4 &= ~(1 << PIN_3V3_Sense_4);

					// Set 3V3_Sense_5 as Input with Pull-Down
					DDR_3V3_Sense_5 &= ~(1 << PIN_3V3_Sense_5);
					PORT_3V3_Sense_5 &= ~(1 << PIN_3V3_Sense_5);

					// Set 3V3_Sense_6 as Input with Pull-Down
					DDR_3V3_Sense_6 &= ~(1 << PIN_3V3_Sense_6);
					PORT_3V3_Sense_6 &= ~(1 << PIN_3V3_Sense_6);

					// Set 3V3_Sense_7 as Input with Pull-Down
					DDR_3V3_Sense_7 &= ~(1 << PIN_3V3_Sense_7);
					PORT_3V3_Sense_7 &= ~(1 << PIN_3V3_Sense_7);

					// Set 3V3_Sense_8 as Input with Pull-Down
					DDR_3V3_Sense_8 &= ~(1 << PIN_3V3_Sense_8);
					PORT_3V3_Sense_8 &= ~(1 << PIN_3V3_Sense_8);

					// Set MCU_LED_RED as Output with Pull-Down
					DDR_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
					PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);

					// Set MCU_LED_GREEN as Output with Pull-Down
					DDR_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
					PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);

					// Set MCU_LED_BLUE as Output with Pull-Down
					DDR_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);
					PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

					// Set HEARTBEAT as Output with Pull-Down
					DDR_HEARTBEAT |= (1 << PIN_HEARTBEAT);
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

					// Pin Read Macros
					#define PIN_READ_ENERGY_INT1 (((PORT_INT_ENERGY_1) >> (PIN_INT_ENERGY_1)) & 0x01)
					#define PIN_READ_ENERGY_INT2 (((PORT_INT_ENERGY_2) >> (PIN_INT_ENERGY_2)) & 0x01)
					#define PIN_READ_ENV_INT (((PORT_INT_ENV) >> (PIN_INT_ENV)) & 0x01)
					#define PIN_READ_RTC_INT (((PORT_INT_RTC) >> (PIN_INT_RTC)) & 0x01)
					#define PIN_READ_RS485_INT (((PORT_INT_RS485) >> (PIN_INT_RS485)) & 0x01)
					#define PIN_READ_CHARGER_INT (((PORT_INT_CHARGER) >> (PIN_INT_CHARGER)) & 0x01)
					#define PIN_READ_GAUGE_INT (((PORT_INT_GAUGE) >> (PIN_INT_GAUGE)) & 0x01)
					#define PIN_READ_LCD_SENSE (((PORT_LCD_SENSE) >> (PIN_LCD_SENSE)) & 0x01)
					#define PIN_READ_SD_SENSE (((PORT_SD_SENSE) >> (PIN_SD_SENSE)) & 0x01)
					#define PIN_READ_TERMINAL_SENSE (((PORT_TERMINAL_SENSE) >> (PIN_TERMINAL_SENSE)) & 0x01)
					#define PIN_READ_GSM_RING (((PORT_GSM_RING) >> (PIN_GSM_RING)) & 0x01)
					#define PIN_READ_GSM_POWER_MONITOR (((PORT_GSM_PMON) >> (PIN_GSM_PMON)) & 0x01)
					#define PIN_READ_GSM_SOFTWARE_READY (((PORT_GSM_SWREADY) >> (PIN_GSM_SWREADY)) & 0x01)
					#define PIN_READ_PHASE_R (((PORT_3V3_Sense_1) >> (PIN_3V3_Sense_1)) & 0x01)
					#define PIN_READ_PHASE_S (((PORT_3V3_Sense_2) >> (PIN_3V3_Sense_2)) & 0x01)
					#define PIN_READ_PHASE_T (((PORT_3V3_Sense_3) >> (PIN_3V3_Sense_3)) & 0x01)
					#define PIN_READ_M1_RELAY (((PORT_3V3_Sense_4) >> (PIN_3V3_Sense_4)) & 0x01)
					#define PIN_READ_M2_RELAY (((PORT_3V3_Sense_5) >> (PIN_3V3_Sense_5)) & 0x01)
					#define PIN_READ_M3_RELAY (((PORT_3V3_Sense_6) >> (PIN_3V3_Sense_6)) & 0x01)
					#define PIN_READ_THERMIC_RELAY (((PORT_3V3_Sense_7) >> (PIN_3V3_Sense_7)) & 0x01)
					#define PIN_READ_MOTOR_PROTECTION (((PORT_3V3_Sense_8) >> (PIN_3V3_Sense_8)) & 0x01)

					// Pin Write Macros
					#define PIN_WRITE_START(_State) (_State ? PORT_RELAY_START |= (1 << PIN_RELAY_START) : PORT_RELAY_START &= ~(1 << PIN_RELAY_START))
					#define PIN_WRITE_STOP(_State) (_State ? PORT_RELAY_STOP |= (1 << PIN_RELAY_STOP) : PORT_RELAY_STOP &= ~(1 << PIN_RELAY_STOP))
					#define PIN_WRITE_3V8_EN(_State) (_State ? PORT_3V8_EN |= (1 << PIN_3V8_EN) : PORT_3V8_EN &= ~(1 << PIN_3V8_EN))
					#define PIN_WRITE_RS485_DIR(_State) (_State ? PORT_RS485_DIR |= (1 << PIN_RS485_DIR) : PORT_RS485_DIR &= ~(1 << PIN_RS485_DIR))
					#define PIN_WRITE_BUZZER(_State) (_State ? PORT_3V3_BUZZER |= (1 << PIN_3V3_BUZZER) : PORT_3V3_BUZZER &= ~(1 << PIN_3V3_BUZZER))
					#define PIN_WRITE_FOTA_POWER_EN(_State) (_State ? PORT_FOTA_POWER_EN |= (1 << PIN_FOTA_POWER_EN) : PORT_FOTA_POWER_EN &= ~(1 << PIN_FOTA_POWER_EN))
					#define PIN_WRITE_SD_EN(_State) (_State ? PORT_SD_EN |= (1 << PIN_SD_EN) : PORT_SD_EN &= ~(1 << PIN_SD_EN))
					#define PIN_WRITE_GSM_COMMUNICATION_ENABLE(_State) (_State ? PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN) : PORT_GSM_COMM_EN &= ~(1 << PIN_GSM_COMM_EN))
					#define PIN_WRITE_GSM_ON_OFF(_State) (_State ? PORT_GSM_ONOFF |= (1 << PIN_GSM_ONOFF) : PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF))
					#define PIN_WRITE_GSM_SHUTDOWN(_State) (_State ? PORT_GSM_SDOWN |= (1 << PIN_GSM_SDOWN) : PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN))

				#endif

				// B108AA Module
				#ifdef B108AA

					// Define I2C Port
					#define I2C_MUX_DS28C		1
					#define I2C_MUX_HDC2010		2
					#define I2C_MUX_SI1145		3
					#define I2C_MUX_MPL3115		4
					#define I2C_MUX_SDP810_X	5
					#define I2C_MUX_SDP810_Y	6
					#define I2C_MUX_SDP810_Z	7
					#define I2C_MUX_Power		8

					// Set GSM_POWER_EN as Output with Pull-Down
					DDR_GSM_POWER_EN |= (1 << PIN_GSM_POWER_EN);
					PORT_GSM_POWER_EN &= ~(1 << PIN_GSM_POWER_EN);

					// Set GSM_COMM_EN as Output with Pull-Up
					DDR_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);
					PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);

					// Set GSM_ONOFF as Output with Pull-Down
					DDR_GSM_ONOFF |= (1 << PIN_GSM_ONOFF);
					PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF);

					// Set GSM_SDOWN as Output with Pull-Down
					DDR_GSM_SDOWN |= (1 << PIN_GSM_SDOWN);
					PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN);

					// Set GSM_GNSS_EN as Output with Pull-Down
					DDR_GSM_GNSS_EN |= (1 << PIN_GSM_GNSS_EN);
					PORT_GSM_GNSS_EN &= ~(1 << PIN_GSM_GNSS_EN);

					// Set RS485_EN_4 as Output with Pull-Down
					DDR_RS485_EN_4 |= (1 << PIN_RS485_EN_4);
					PORT_RS485_EN_4 &= ~(1 << PIN_RS485_EN_4);

					// Set RS485_EN_2 as Output with Pull-Down
					DDR_RS485_EN_2 |= (1 << PIN_RS485_EN_2);
					PORT_RS485_EN_2 &= ~(1 << PIN_RS485_EN_2);

					// Set RS485_DIR_2 as Output with Pull-Down
					DDR_RS485_DIR_2 |= (1 << PIN_RS485_DIR_2);
					PORT_RS485_DIR_2 &= ~(1 << PIN_RS485_DIR_2);

					// Set RS485_DIR_4 as Output with Pull-Down
					DDR_RS485_DIR_4 |= (1 << PIN_RS485_DIR_4);
					PORT_RS485_DIR_4 &= ~(1 << PIN_RS485_DIR_4);

					// Set RS485_DIR_3 as Output with Pull-Down
					DDR_RS485_DIR_3 |= (1 << PIN_RS485_DIR_3);
					PORT_RS485_DIR_3 &= ~(1 << PIN_RS485_DIR_3);

					// Set RS485_DIR_1 as Output with Pull-Down
					DDR_RS485_DIR_1 |= (1 << PIN_RS485_DIR_1);
					PORT_RS485_DIR_1 &= ~(1 << PIN_RS485_DIR_1);

					// Set RS485_EN_1 as Output with Pull-Down
					DDR_RS485_EN_1 |= (1 << PIN_RS485_EN_1);
					PORT_RS485_EN_1 &= ~(1 << PIN_RS485_EN_1);

					// Set RS485_EN_3 as Output with Pull-Down
					DDR_RS485_EN_3 |= (1 << PIN_RS485_EN_3);
					PORT_RS485_EN_3 &= ~(1 << PIN_RS485_EN_3);
					
					// Set RS485_MUX_B as Output with Pull-Down
					DDR_RS485_MUX_B |= (1 << PIN_RS485_MUX_B);
					PORT_RS485_MUX_B &= ~(1 << PIN_RS485_MUX_B);

					// Set RS485_MUX_A as Output with Pull-Down
					DDR_RS485_MUX_A |= (1 << PIN_RS485_MUX_A);
					PORT_RS485_MUX_A &= ~(1 << PIN_RS485_MUX_A);

					// Set RS485_MUX_EN as Output with Pull-Up
					DDR_RS485_MUX_EN |= (1 << PIN_RS485_MUX_EN);
					PORT_RS485_MUX_EN |= (1 << PIN_RS485_MUX_EN);

					// Set DONE as Output with Pull-Down
					DDR_DONE |= (1 << PIN_DONE);
					PORT_DONE &= ~(1 << PIN_DONE);
					
					// Set GSM_PMON as Input with Pull-Down
					DDR_GSM_PWMon &= ~(1 << PIN_GSM_PWMon);
					PORT_GSM_PWMon &= ~(1 << PIN_GSM_PWMon);

					// Set GSM_RING as Input with Pull-Up
					DDR_GSM_RING &= ~(1 << PIN_GSM_RING);
					PORT_GSM_RING |= (1 << PIN_GSM_RING);

					// Set GSM_SWREADY as Input with Pull-Down
					DDR_GSM_SWReady &= ~(1 << PIN_GSM_SWReady);
					PORT_GSM_SWReady &= ~(1 << PIN_GSM_SWReady);

					// Set MCU_LED_RED as Output with Pull-Down
					DDR_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
					PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);

					// Set MCU_LED_GREEN as Output with Pull-Down
					DDR_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
					PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);

					// Set MCU_LED_BLUE as Output with Pull-Down
					DDR_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);
					PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

					// Set RS485_SENSE_1 as Input with Pull-Down
					DDR_RS485_SENSE_1 &= ~(1 << PIN_RS485_SENSE_1);
					PORT_RS485_SENSE_1 &= ~(1 << PIN_RS485_SENSE_1);

					// Set RS485_SENSE_2 as Input with Pull-Down
					DDR_RS485_SENSE_2 &= ~(1 << PIN_RS485_SENSE_2);
					PORT_RS485_SENSE_2 &= ~(1 << PIN_RS485_SENSE_2);

					// Set RS485_SENSE_3 as Input with Pull-Down
					DDR_RS485_SENSE_3 &= ~(1 << PIN_RS485_SENSE_3);
					PORT_RS485_SENSE_3 &= ~(1 << PIN_RS485_SENSE_3);

					// LED Macros
					#define PIN_WRITE_RED_LED(_State) (_State ? PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED) : PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED))
					#define PIN_WRITE_GREEN_LED(_State) (_State ? PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN) : PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN))
					#define PIN_WRITE_BLUE_LED(_State) (_State ? PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE) : PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE))

					// Power Macros
					#define SLEEP() (PORT_DONE |= (1 << PIN_DONE))

				#endif

			}

			// Module 1 Second Timer
			inline void AVR_Timer(void) {

				// Clear Registers
				TCCR5A = 0x00;
				TCCR5B = 0x00;

				// Clear Counter
				TCNT5 = 0;

				// Set Counter Value
				OCR5A = (F_CPU / (1024)) - 1;

				// Set CTC Mod
				TCCR5B |= (1 << WGM52);

				// Set Rescale (1024)
				TCCR5B |= (1 << CS52) | (1 << CS50);

				// Start Timer
				TIMSK5 |= (1 << OCIE5A);

			}

			// Module Interrupt Enable Function
			inline void AVR_Interrupt(void) {

				/*

					PCICR Register
					--------------

					8   7   6   5   4   3     2     1     0
					-   -   -   -   -   -   PCIE2 PCIE1 PCIE0
					
					PCINT0  - PCINT7   : PCIE0 Mask
					PCINT8  - PCINT15  : PCIE1 Mask
					PCINT16 - PCINT23  : PCIE2 Mask

				*/

				// B107AA Module
				#ifdef B107AA

					// Set RS485_INT, CHARGER_INT, GAUGE_INT Interrupt falling edge triggered Interrupt
					EIMSK |= (1 << INT4) | (1 << INT5) | (1 << INT6);
					EICRB |= (1 << ISC41) | (1 << ISC50) | (1 << ISC60);

					// Set INT_Energy_1, INT_Energy_2, INT_ENV, INT_RTC Interrupt
					PCICR |= (1 << PCIE0);
					PCMSK0 |= (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT6) | (1 << PCINT7);

					// Set GSM_RING, GSM_PMON, GSM_SWREADY Interrupt
					PCICR |= (1 << PCIE1);
					PCMSK1 |= (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);

					// Set MP, Thermic, M3_Relay, M2_Relay, M1_Relay, PHASE_T, PHASE_S, PHASE_R Interrupt
					PCICR |= (1 << PCIE2);
					PCMSK2 |= (1 << PCINT16) | (1 << PCINT17) | (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);

				#endif

				// Start Interrupts
				sei();

			}

			// LED Function
			void Module_LED(const uint8_t _Color, const uint8_t _Blink, const uint16_t _Interval) {

				switch (_Color)	{

					// Red Color
					case RED: {

						// Blink
						for (size_t i = 0; i < _Blink; i++) {

							// Turn ON Red LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
							#endif

							// Delay
							delay(_Interval);

							// Turn OFF Red LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);
							#endif

							// Delay
							delay(_Interval);

						}

						// End Case
						break;

					}

					// Green Color
					case GREEN: {

						// Blink
						for (size_t i = 0; i < _Blink; i++) {

							// Turn ON Green LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
							#endif

							// Delay
							delay(_Interval);

							// Turn OFF Green LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);
							#endif

							// Delay
							delay(_Interval);

						}

						// End Case
						break;

					}

					// Blue Color
					case BLUE: {

						// Blink
						for (size_t i = 0; i < _Blink; i++) {

							// Turn ON Blue LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);
							#endif

							// Delay
							delay(_Interval);

							// Turn OFF Blue LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);
							#endif

							// Delay
							delay(_Interval);

						}

						// End Case
						break;

					}

					// White Color
					case WHITE: {

						// Blink
						for (size_t i = 0; i < _Blink; i++) {

							// Turn ON White LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
								PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
								PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);
							#endif

							// Delay
							delay(_Interval);

							// Turn OFF White LED
							#if defined(B107AA) || defined(B108AA)
								PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);
								PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);
								PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);
							#endif

							// Delay
							delay(_Interval);

						}

						// End Case
						break;

					}

					// Default
					default: {

						// Turn OFF all LED
						#if defined(B107AA) || defined(B108AA)
							PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);
							PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);
							PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);
						#endif

						// End Case
						break;

					}

				}

			}

			// Heartbeat Function
			inline void Heartbeat(void) {

				// B107AA Module
				#ifdef B107AA

					// Turn ON HeartBeat
					PORT_HEARTBEAT |= (1 << PIN_HEARTBEAT);

					// HeartBeat LED Blink
					this->Module_LED(GREEN, 1, 100);

					// Turn OFF HeartBeat
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

				#endif

			}

		// Private Context
		private:
		
	};

#endif /* defined(AVR_Functions) */
