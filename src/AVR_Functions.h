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

	// Define Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Include Pin Out Definitions
	#ifndef __PinOut__
		#include "PinOut/PinOut.h"
	#endif

	// Include Pin Name Definitions
	#ifndef __Macro__
		#include "Macro.h"
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


					//  PORT A
					DDRA = 0b11111111; PORTA = 0b00000000;

					//  PORT B
					DDRB &= 0b00001111; PORTB |= 0b11110000;	// Set NC pins.

					//  PORT C
					DDRC = 0b11111111; PORTC = 0b00000000;

					//  PORT D
					DDRD |= 0b11110000; PORTD &= 0b00001111;	// Set NC pins.

					//  PORT E
					DDRE |= 0b10001100; DDRE &= 0b01100111;	// Set NC pins.
					PORTE |= 0b01100100; PORTE &= 0b01100111;	// Set NC pins.

					//  PORT F
					DDRF = 0b11111111; PORTF = 0b00000000;

					// PORT G
					DDRG |= 0b00111111; PORTG &= 0b11000000;	// Set NC pins.

					//  PORT H
					DDRH |= (1 << PH2); PORTH &= ~(1 << PH2);	// NC pin.
					DDRH &= ~(1 << PH3); PORTH &= ~(1 << PH3);	// LCD_SENSE pin.
					DDRH |= (1 << PH4); PORTH &= ~(1 << PH4);	// FOTA_POWER_EN pin.
					DDRH &= ~(1 << PH5); PORTH &= ~(1 << PH5);	// SD_SENSE pin.
					DDRH |= (1 << PH6); PORTH &= ~(1 << PH6);	// SD_MUX_SEL pin.
					DDRH &= ~(1 << PH7); PORTH &= ~(1 << PH7);	// Terminal_SENSE pin.

					//  PORT J
//					DDRJ = 0b11100000;
//					PORTJ = 0b00100100;
					DDRJ |= 0b11100000; DDRJ &= 0b11100011;
					PORTJ |= 0b00000100; PORTJ &= 0b00000111;

					//  PORT K
					DDRK = 0b00000000; PORTK = 0b00000000;		// Set NC pins.

					//  PORT L
					DDRL = 0b11111111; PORTL = 0b00000000;

				#endif

				// B108AA Module
				#ifdef B108AA

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

		// Private Context
		private:
		
	};

#endif /* defined(AVR_Functions) */
