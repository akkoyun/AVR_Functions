#ifndef __PinOut__
#define __PinOut__

    // B107AA Module
    #if defined(B107AA)

		// RELAY_START Pin Definition
		#define DDR_RELAY_START 			DDRA
		#define PORT_RELAY_START			PORTA
		#define PIN_REGISTER_RELAY_START	PINA
		#define PIN_RELAY_START			    PA0

		// RELAY_STOP Pin Definition
		#define DDR_RELAY_STOP				DDRA
		#define PORT_RELAY_STOP				PORTA
		#define PIN_REGISTER_RELAY_STOP		PINA
		#define PIN_RELAY_STOP				PA1

		// INT_ENERGY_1 Pin Definition
		#define DDR_INT_ENERGY_1			DDRB
		#define PORT_INT_ENERGY_1			PORTB
		#define PIN_REGISTER_INT_ENERGY_1	PINB
		#define PIN_INT_ENERGY_1			PB4

		// INT_ENERGY_2 Pin Definition
		#define DDR_INT_ENERGY_2			DDRB
		#define PORT_INT_ENERGY_2			PORTB
		#define PIN_REGISTER_INT_ENERGY_2	PINB
		#define PIN_INT_ENERGY_2			PB5

		// INT_ENV Pin Definition
		#define DDR_INT_ENV					DDRB
		#define PORT_INT_ENV				PORTB
		#define PIN_REGISTER_INT_ENV		PINB
		#define PIN_INT_ENV					PB6

		// INT_RTC Pin Definition
		#define DDR_INT_RTC					DDRB
		#define PORT_INT_RTC				PORTB
		#define PIN_REGISTER_INT_RTC		PINB
		#define PIN_INT_RTC					PB7

		// 3V8_EN Pin Definition
		#define DDR_3V8_EN					DDRE
		#define PORT_3V8_EN					PORTE
		#define PIN_REGISTER_3V8_EN			PINE
		#define PIN_3V8_EN					PE2

		// RS485_DIR Pin Definition
		#define DDR_RS485_DIR				DDRE
		#define PORT_RS485_DIR				PORTE
		#define PIN_REGISTER_RS485_DIR		PINE
		#define PIN_RS485_DIR				PE3

		// INT_RS485 Pin Definition
		#define DDR_INT_RS485				DDRE
		#define PORT_INT_RS485				PORTE
		#define PIN_REGISTER_INT_RS485		PINE
		#define PIN_INT_RS485				PE4

		// INT_CHARGER Pin Definition
		#define DDR_INT_CHARGER				DDRE
		#define PORT_INT_CHARGER			PORTE
		#define PIN_REGISTER_INT_CHARGER	PINE
		#define PIN_INT_CHARGER				PE5

		// INT_GAUGE Pin Definition
		#define DDR_INT_GAUGE				DDRE
		#define PORT_INT_GAUGE				PORTE
		#define PIN_REGISTER_INT_GAUGE		PINE
		#define PIN_INT_GAUGE				PE6

		// 3V3_BUZZER Pin Definition
		#define DDR_3V3_BUZZER				DDRE
		#define PORT_3V3_BUZZER				PORTE
		#define PIN_REGISTER_3V3_BUZZER		PINE
		#define PIN_3V3_BUZZER				PE7

		// LCD_SENSE Pin Definition
		#define DDR_LCD_SENSE				DDRH
		#define PORT_LCD_SENSE				PORTH
		#define PIN_REGISTER_LCD_SENSE		PINH
		#define PIN_LCD_SENSE				PH3

		// FOTA_POWER_EN Pin Definition
		#define DDR_FOTA_POWER_EN			DDRH
		#define PORT_FOTA_POWER_EN			PORTH
		#define PIN_REGISTER_FOTA_POWER_EN	PINH
		#define PIN_FOTA_POWER_EN			PH4

		// SD_SENSE Pin Definition
		#define DDR_SD_SENSE				DDRH
		#define PORT_SD_SENSE				PORTH
		#define PIN_REGISTER_SD_SENSE		PINH
		#define PIN_SD_SENSE				PH5

		// SD_EN Pin Definition
		#define DDR_SD_EN					DDRH
		#define PORT_SD_EN					PORTH
		#define PIN_REGISTER_SD_EN			PINH
		#define PIN_SD_EN					PH6

		// TERMINAL_SENSE Pin Definition
		#define DDR_TERMINAL_SENSE			DDRH
		#define PORT_TERMINAL_SENSE			PORTH
		#define PIN_REGISTER_TERMINAL_SENSE	PINH
		#define PIN_TERMINAL_SENSE			PH7

		// GSM_RING Pin Definition
		#define DDR_GSM_RING				DDRJ
		#define PORT_GSM_RING				PORTJ
		#define PIN_REGISTER_GSM_RING		PINJ
		#define PIN_GSM_RING				PJ2

		// GSM_PMON Pin Definition
		#define DDR_GSM_PMON				DDRJ
		#define PORT_GSM_PMON				PORTJ
		#define PIN_REGISTER_GSM_PMON		PINJ
		#define PIN_GSM_PMON				PJ3

		// GSM_SWREADY Pin Definition
		#define DDR_GSM_SWREADY				DDRJ
		#define PORT_GSM_SWREADY			PORTJ
		#define PIN_REGISTER_GSM_SWREADY	PINJ
		#define PIN_GSM_SWREADY				PJ4

		// GSM_COMM_EN Pin Definition
		#define DDR_GSM_COMM_EN				DDRJ
		#define PORT_GSM_COMM_EN			PORTJ
		#define PIN_REGISTER_GSM_COMM_EN	PINJ
		#define PIN_GSM_COMM_EN				PJ5

		// GSM_ONOFF Pin Definition
		#define DDR_GSM_ONOFF				DDRJ
		#define PORT_GSM_ONOFF				PORTJ
		#define PIN_REGISTER_GSM_ONOFF		PINJ
		#define PIN_GSM_ONOFF				PJ6

		// GSM_SDOWN Pin Definition
		#define DDR_GSM_SDOWN				DDRJ
		#define PORT_GSM_SDOWN				PORTJ
		#define PIN_REGISTER_GSM_SDOWN		PINJ
		#define PIN_GSM_SDOWN				PJ7

		// 3V3_Sense_1 Pin Definition
		#define DDR_3V3_Sense_1				DDRK
		#define PORT_3V3_Sense_1			PORTK
		#define PIN_REGISTER_3V3_Sense_1	PINK
		#define PIN_3V3_Sense_1				PK0

		// 3V3_Sense_2 Pin Definition
		#define DDR_3V3_Sense_2				DDRK
		#define PORT_3V3_Sense_2			PORTK
		#define PIN_REGISTER_3V3_Sense_2	PINK
		#define PIN_3V3_Sense_2				PK1

		// 3V3_Sense_3 Pin Definition
		#define DDR_3V3_Sense_3				DDRK
		#define PORT_3V3_Sense_3			PORTK
		#define PIN_REGISTER_3V3_Sense_3	PINK
		#define PIN_3V3_Sense_3				PK2

		// 3V3_Sense_4 Pin Definition
		#define DDR_3V3_Sense_4				DDRK
		#define PORT_3V3_Sense_4			PORTK
		#define PIN_REGISTER_3V3_Sense_4	PINK
		#define PIN_3V3_Sense_4				PK3

		// 3V3_Sense_5 Pin Definition
		#define DDR_3V3_Sense_5				DDRK
		#define PORT_3V3_Sense_5			PORTK
		#define PIN_REGISTER_3V3_Sense_5	PINK
		#define PIN_3V3_Sense_5				PK4

		// 3V3_Sense_6 Pin Definition
		#define DDR_3V3_Sense_6				DDRK
		#define PORT_3V3_Sense_6			PORTK
		#define PIN_REGISTER_3V3_Sense_6	PINK
		#define PIN_3V3_Sense_6				PK5

		// 3V3_Sense_7 Pin Definition
		#define DDR_3V3_Sense_7				DDRK
		#define PORT_3V3_Sense_7			PORTK
		#define PIN_REGISTER_3V3_Sense_7	PINK
		#define PIN_3V3_Sense_7				PK6

		// 3V3_Sense_8 Pin Definition
		#define DDR_3V3_Sense_8				DDRK
		#define PORT_3V3_Sense_8			PORTK
		#define PIN_REGISTER_3V3_Sense_8	PINK
		#define PIN_3V3_Sense_8				PK7

		// MCU_LED_RED Pin Definition
		#define DDR_MCU_LED_RED				DDRL
		#define PORT_MCU_LED_RED			PORTL
		#define PIN_REGISTER_MCU_LED_RED	PINL
		#define PIN_MCU_LED_RED				PL0

		// MCU_LED_GREEN Pin Definition
		#define DDR_MCU_LED_GREEN			DDRL
		#define PORT_MCU_LED_GREEN			PORTL
		#define PIN_REGISTER_MCU_LED_GREEN	PINL
		#define PIN_MCU_LED_GREEN			PL1

		// MCU_LED_BLUE Pin Definition
		#define DDR_MCU_LED_BLUE			DDRL
		#define PORT_MCU_LED_BLUE			PORTL
		#define PIN_REGISTER_MCU_LED_BLUE	PINL
		#define PIN_MCU_LED_BLUE			PL2

		// HEARTBEAT Pin Definition
		#define DDR_HEARTBEAT				DDRL
		#define PORT_HEARTBEAT				PORTL
		#define PIN_REGISTER_HEARTBEAT		PINL
		#define PIN_HEARTBEAT				PL3

    // B108AA Module
    #elif defined(B108AA)

        // GSM_POWER_EN Pin Definition
        #define DDR_GSM_POWER_EN			DDRA
        #define PORT_GSM_POWER_EN			PORTA
        #define PIN_REGISTER_GSM_POWER_EN	PINA
        #define PIN_GSM_POWER_EN			PA0

        // GSM_COMM_EN Pin Definition
        #define DDR_GSM_COMM_EN				DDRA
        #define PORT_GSM_COMM_EN			PORTA
        #define PIN_REGISTER_GSM_COMM_EN	PINA
        #define PIN_GSM_COMM_EN				PA1

        // GSM_ONOFF Pin Definition
        #define DDR_GSM_ONOFF				DDRA
        #define PORT_GSM_ONOFF				PORTA
        #define PIN_REGISTER_GSM_ONOFF		PINA
        #define PIN_GSM_ONOFF				PA2

        // GSM_SDOWN Pin Definition
        #define DDR_GSM_SDOWN				DDRA
        #define PORT_GSM_SDOWN				PORTA
        #define PIN_REGISTER_GSM_SDOWN		PINA
        #define PIN_GSM_SDOWN				PA3

        // GSM_GNSS_EN Pin Definition
        #define DDR_GSM_GNSS_EN				DDRA
        #define PORT_GSM_GNSS_EN			PORTA
        #define PIN_REGISTER_GSM_GNSS_EN	PINA
        #define PIN_GSM_GNSS_EN				PA4

        // RS485_EN_4 Pin Definition
        #define DDR_RS485_EN_4				DDRC
        #define PORT_RS485_EN_4				PORTC
        #define PIN_REGISTER_RS485_EN_4		PINC
        #define PIN_RS485_EN_4				PC0

        // RS485_EN_2 Pin Definition
        #define DDR_RS485_EN_2				DDRC
        #define PORT_RS485_EN_2				PORTC
        #define PIN_REGISTER_RS485_EN_2		PINC
        #define PIN_RS485_EN_2				PC1

        // RS485_DIR_2 Pin Definition
        #define DDR_RS485_DIR_2				DDRC
        #define PORT_RS485_DIR_2			PORTC
        #define PIN_REGISTER_RS485_DIR_2	PINC
        #define PIN_RS485_DIR_2				PC2

        // RS485_DIR_4 Pin Definition
        #define DDR_RS485_DIR_4				DDRC
        #define PORT_RS485_DIR_4			PORTC
        #define PIN_REGISTER_RS485_DIR_4	PINC
        #define PIN_RS485_DIR_4				PC3

        // RS485_DIR_3 Pin Definition
        #define DDR_RS485_DIR_3				DDRC
        #define PORT_RS485_DIR_3			PORTC
        #define PIN_REGISTER_RS485_DIR_3	PINC
        #define PIN_RS485_DIR_3				PC4

        // RS485_DIR_1 Pin Definition
        #define DDR_RS485_DIR_1				DDRC
        #define PORT_RS485_DIR_1			PORTC
        #define PIN_REGISTER_RS485_DIR_1	PINC
        #define PIN_RS485_DIR_1				PC5

        // RS485_EN_1 Pin Definition
        #define DDR_RS485_EN_1				DDRC
        #define PORT_RS485_EN_1				PORTC
        #define PIN_REGISTER_RS485_EN_1		PINC
        #define PIN_RS485_EN_1				PC6

        // RS485_EN_3 Pin Definition
        #define DDR_RS485_EN_3				DDRC
        #define PORT_RS485_EN_3				PORTC
        #define PIN_REGISTER_RS485_EN_3		PINC
        #define PIN_RS485_EN_3				PC7

        // RS485_MUX_B Pin Definition
        #define DDR_RS485_MUX_B				DDRD
        #define PORT_RS485_MUX_B			PORTD
        #define PIN_REGISTER_RS485_MUX_B	PIND
        #define PIN_RS485_MUX_B				PD5

        // RS485_MUX_A Pin Definition
        #define DDR_RS485_MUX_A				DDRD
        #define PORT_RS485_MUX_A			PORTD
        #define PIN_REGISTER_RS485_MUX_A	PIND
        #define PIN_RS485_MUX_A				PD6

        // RS485_MUX_EN Pin Definition
        #define DDR_RS485_MUX_EN			DDRD
        #define PORT_RS485_MUX_EN			PORTD
        #define PIN_REGISTER_RS485_MUX_EN	PIND
        #define PIN_RS485_MUX_EN			PD7

        // DONE Pin Definition
        #define DDR_DONE					DDRG
        #define PORT_DONE					PORTG
        #define PIN_REGISTER_DONE			PING
        #define PIN_DONE					PG0

        // GSM_PMON Pin Definition
        #define DDR_GSM_PWMon				DDRJ
        #define PORT_GSM_PWMon				PORTJ
        #define PIN_REGISTER_GSM_PWMon		PINJ
        #define PIN_GSM_PWMon				PJ2

        // GSM_RING Pin Definition
        #define DDR_GSM_RING				DDRJ
        #define PORT_GSM_RING				PORTJ
        #define PIN_REGISTER_GSM_RING		PINJ
        #define PIN_GSM_RING				PJ3

        // GSM_SWREADY Pin Definition
        #define DDR_GSM_SWReady				DDRJ
        #define PORT_GSM_SWReady			PORTJ
        #define PIN_REGISTER_GSM_SWReady	PINJ
        #define PIN_GSM_SWReady				PJ4

        // MCU LED RED Pin Definition
        #define DDR_MCU_LED_RED				DDRK
        #define PORT_MCU_LED_RED			PORTK
        #define PIN_MCU_LED_RED				PK0

        // MCU LED GREEN Pin Definition
        #define DDR_MCU_LED_GREEN			DDRK
        #define PORT_MCU_LED_GREEN			PORTK
        #define PIN_MCU_LED_GREEN			PK1

        // MCU LED BLUE Pin Definition
        #define DDR_MCU_LED_BLUE			DDRK
        #define PORT_MCU_LED_BLUE			PORTK
        #define PIN_MCU_LED_BLUE			PK2

        // RS485_SENSE_1 Pin Definition
        #define DDR_RS485_SENSE_1			DDRL
        #define PORT_RS485_SENSE_1			PORTL
        #define PIN_RS485_SENSE_1			PL0

        // RS485_SENSE_2 Pin Definition
        #define DDR_RS485_SENSE_2			DDRL
        #define PORT_RS485_SENSE_2			PORTL
        #define PIN_RS485_SENSE_2			PL1

        // RS485_SENSE_3 Pin Definition
        #define DDR_RS485_SENSE_3			DDRL
        #define PORT_RS485_SENSE_3			PORTL
        #define PIN_RS485_SENSE_3			PL2

        // RS485_SENSE_4 Pin Definition
        #define DDR_RS485_SENSE_4			DDRL
        #define PORT_RS485_SENSE_4			PORTL
        #define PIN_RS485_SENSE_4			PL3

    #endif

#endif