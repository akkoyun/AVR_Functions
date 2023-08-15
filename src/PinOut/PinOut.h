#ifndef __PinOut__
#define __PinOut__

    // B107AA Module
    #if defined(B107AA)


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