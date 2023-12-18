#ifndef __Macro__
#define __Macro__

	// Define Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Define LED Colors
	#define WHITE 	0
	#define RED 	1
	#define GREEN 	2
	#define BLUE 	3

	// B107AA Module
	#ifdef B107AA

		// Pin Read Macros
		#define PIN_READ_ENERGY_INT1 (((PORTB) >> (PB4)) & 0x01)
		#define PIN_READ_ENERGY_INT2 (((PORTB) >> (PB5)) & 0x01)
		#define PIN_READ_ENV_INT (((PORTB) >> (PB6)) & 0x01)
		#define PIN_READ_RTC_INT (((PORTB) >> (PB7)) & 0x01)
		#define PIN_READ_RS485_INT (((PORTE) >> (PE4)) & 0x01)
		#define PIN_READ_CHARGER_INT (((PORTE) >> (PE5)) & 0x01)
		#define PIN_READ_GAUGE_INT (((PORTE) >> (PE6)) & 0x01)
		#define PIN_READ_LCD_SENSE (((PORTH) >> (PH3)) & 0x01)
		#define PIN_READ_SD_SENSE (((PORTH) >> (PH5)) & 0x01)
		#define PIN_READ_TERMINAL_SENSE (((PORTH) >> (PH7)) & 0x01)
		#define PIN_READ_GSM_RING (((PORTJ) >> (PJ2)) & 0x01)
		#define PIN_READ_GSM_POWER_MONITOR (((PORTJ) >> (PJ3)) & 0x01)
		#define PIN_READ_GSM_SOFTWARE_READY (((PORTJ) >> (PJ4)) & 0x01)
		#define PIN_READ_PHASE_R (((PORTK) >> (PK0)) & 0x01)
		#define PIN_READ_PHASE_S (((PORTK) >> (PK1)) & 0x01)
		#define PIN_READ_PHASE_T (((PORTK) >> (PK2)) & 0x01)
		#define PIN_READ_M1_RELAY (((PORTK) >> (PK3)) & 0x01)
		#define PIN_READ_M2_RELAY (((PORTK) >> (PK4)) & 0x01)
		#define PIN_READ_M3_RELAY (((PORTK) >> (PK5)) & 0x01)
		#define PIN_READ_THERMIC_RELAY (((PORTK) >> (PK6)) & 0x01)
		#define PIN_READ_MOTOR_PROTECTION (((PORTK) >> (PK7)) & 0x01)

		// Pin Write Macros
		#define PIN_WRITE_START(_State) (_State ? PORTA |= 0b00000001 : PORTA &= 0b11111110)
		#define PIN_WRITE_STOP(_State) (_State ? PORTA |= 0b00000010 : PORTA &= 0b11111101)
		#define PIN_WRITE_3V8_EN(_State) (_State ? PORTE |= 0b00000100 : PORTE &= 0b11111011)
		#define PIN_WRITE_RS485_DIR(_State) (_State ? PORTE |= 0b00001000 : PORTE &= 0b11110111)
		#define PIN_WRITE_BUZZER(_State) (_State ? PORTE |= 0b10000000 : PORTE &= 0b01111111)
		#define PIN_WRITE_FOTA_POWER_EN(_State) (_State ? PORTH |= 0b00010000 : PORTH &= 0b11101111)
		#define PIN_WRITE_SD_MUX_SEL(_State) (_State ? PORTH |= 0b01000000 : PORTH &= 0b10111111)
		#define PIN_WRITE_GSM_COMMUNICATION_ENABLE(_State) (_State ? PORTJ &= 0b11011111 : PORTJ |= 0b00100000)
		#define PIN_WRITE_GSM_ON_OFF(_State) (_State ? PORTJ |= 0b01000000 : PORTJ &= 0b10111111)
		#define PIN_WRITE_GSM_SHUTDOWN(_State) (_State ? PORTJ |= 0b10000000 : PORTJ &= 0b01111111)

	#endif

	#ifdef B108AA

		// LED Macros
		#define PIN_WRITE_RED_LED(_State) (_State ? PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED) : PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED))
		#define PIN_WRITE_GREEN_LED(_State) (_State ? PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN) : PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN))
		#define PIN_WRITE_BLUE_LED(_State) (_State ? PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE) : PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE))

		// Power Macros
		#define SLEEP() (PORT_DONE |= (1 << PIN_DONE))

	#endif

#endif