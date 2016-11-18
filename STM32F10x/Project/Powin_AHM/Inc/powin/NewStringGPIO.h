/*
 *??:GPIO.H
 *??:??IO???????
 *??:Alan
 *??:2015/09/21
 */

#ifndef __GPIO_H
#define __GPIO_H

#include <stm32f10x.h>


//LED
#define RUN_LED_PORT            GPIOB
#define RUN_LED_PIN             GPIO_Pin_7
//?????
#define FRONT_LED_PORT          GPIOB
#define FRONT_LED_GRN_PORT      GPIOB
#define FRONT_LED_GRN_PIN       GPIO_Pin_9
#define FRONT_LED_RED_PORT      GPIOB
#define FRONT_LED_RED_PIN       GPIO_Pin_8

#define SysIndicator      	  RUN_LED_PORT,RUN_LED_PIN
#define GreenIndictor         FRONT_LED_GRN_PORT,FRONT_LED_GRN_PIN
#define RedIndictor           FRONT_LED_RED_PORT,FRONT_LED_RED_PIN
//?????
#define ADC_Current_PORT 		GPIOC
#define ADC_Current1_PORT       GPIOC
#define ADC_Current1_PIN        GPIO_Pin_2
#define ADC_Current2_PORT       GPIOC
#define ADC_Current2_PIN        GPIO_Pin_3
//?????
#define ADC_Voltage_PORT        GPIOA
#define ADC_Voltage1_PORT       GPIOA
#define ADC_Voltage1_PIN        GPIO_Pin_0
#define ADC_Voltage2_PORT       GPIOA
#define ADC_Voltage2_PIN        GPIO_Pin_1
#define ADC_Voltage3_PORT       GPIOA
#define ADC_Voltage3_PIN        GPIO_Pin_2
//????
#define Temp1_PORT              GPIOC
#define Temp1_PIN               GPIO_Pin_1
#define Temp2_PORT              GPIOA
#define Temp2_PIN               GPIO_Pin_3
//???
#define WatchDog_PORT           GPIOC
#define WatchDog_PIN            GPIO_Pin_0
//????
#define Switch_PORT             GPIOC
#define SwitchA0_PORT           GPIOC
#define SwitchA0_PIN            GPIO_Pin_6
#define SwitchA1_PORT           GPIOC
#define SwitchA1_PIN            GPIO_Pin_7
#define SwitchA2_PORT           GPIOC
#define SwitchA2_PIN            GPIO_Pin_8
#define SwitchA3_PORT           GPIOC
#define SwitchA3_PIN            GPIO_Pin_9
//I2C
#define I2C_PORT                GPIOB
#define I2C_SCL_PORT            GPIOB
#define I2C_SCL_PIN             GPIO_Pin_10
#define I2C_SDA_PORT            GPIOB
#define I2C_SDA_PIN             GPIO_Pin_11
//CAN????
#define CAN_TX_CTRL_PORT        GPIOB
#define CAN_TX_CTRL_PIN         GPIO_Pin_13
#define CAN_RX_CTRL_PORT        GPIOB
#define CAN_RX_CTRL_PIN         GPIO_Pin_12
#define CAN_TX_STRING_PORT      GPIOA
#define CAN_TX_STRING_PIN       GPIO_Pin_12
#define CAN_RX_STRING_PORT      GPIOA
#define CAN_RX_STRING_PIN       GPIO_Pin_11
//CAN????LED
#define CAN_CTRL_LED_PORT       GPIOB
#define CAN_CTRL_TX_LED_PORT    GPIOB
#define CAN_CTRL_TX_LED_PIN     GPIO_Pin_15
#define CAN_CTRL_RX_LED_PORT    GPIOB
#define CAN_CTRL_RX_LED_PIN     GPIO_Pin_14
#define CAN_STRING_LED_PORT     GPIOA
#define CAN_STRING_TX_LED_PORT  GPIOA
#define CAN_STRING_TX_LED_PIN   GPIO_Pin_10
#define CAN_STRING_RX_LED_PORT  GPIOA
#define CAN_STRING_RX_LED_PIN   GPIO_Pin_9
//Contact????
#define Contact_Ctrl_PORT       GPIOA
#define Contact1_Ctrl_PORT      GPIOA
#define Contact1_Ctrl_PIN       GPIO_Pin_4
#define Contact2_Ctrl_PORT      GPIOA
#define Contact2_Ctrl_PIN       GPIO_Pin_6
//Contact????
#define Contact_Sense_PORT      GPIOA
#define Contact1_Sense_PORT     GPIOA
#define Contact1_Sense_PIN      GPIO_Pin_5
#define Contact2_Sense_PORT     GPIOA
#define Contact2_Sense_PIN      GPIO_Pin_7
//Relays????
#define Relay1_Ctrl_PORT      GPIOC
#define Relay1_Ctrl_PIN       GPIO_Pin_4
#define Relay2_Ctrl_PORT      GPIOB
#define Relay2_Ctrl_PIN       GPIO_Pin_0
//Relays????
#define Relay1_Sense_PORT     GPIOC
#define Relay1_Sense_PIN      GPIO_Pin_5
#define Relay2_Sense_PORT     GPIOB
#define Relay2_Sense_PIN      GPIO_Pin_1
//RS485?? (UART4)
#define RS485_RE_PORT           GPIOA
#define RS485_RE_PIN            GPIO_Pin_8
#define RS485_TX_PORT           GPIOC
#define RS485_TX_PIN            GPIO_Pin_10
#define RS485_RX_PORT           GPIOC
#define RS485_RX_PIN            GPIO_Pin_11
//RS485???
#define RS485_LED_PORT          GPIOB
#define RS485_TX_LED_PORT       GPIOB
#define RS485_TX_LED_PIN        GPIO_Pin_5
#define RS485_RX_LED_PORT       GPIOB
#define RS485_RX_LED_PIN        GPIO_Pin_6
//?????? (UART5)
#define TESTUART_TX_PORT        GPIOC
#define TESTUART_TX_PIN         GPIO_Pin_12
#define TESTUART_RX_PORT        GPIOD
#define TESTUART_RX_PIN         GPIO_Pin_2



#define CONTACT1    0
#define CONTACT2    1
#define CONTACT_ON  1
#define CONTACT_OFF 0
enum Contact_State
{
    CONTACT_STATE_CLOSE=0,
    CONTACT_STATE_OPEN=1
};

#define RELAY1      0
#define RELAY2      1
#define RELAY_ON    1
#define RELAY_OFF   0
enum Relay_State
{
    RELAY_STATE_CLOSE=0,
    RELAY_STATE_OPEN=1
};

#define FRONT_LED_RED   0
#define FRONT_LED_GRN   1
#define FRONT_LED_ON    0
#define FRONT_LED_OFF   1

void GPIO_Configurature(void);
void WDT_State(FunctionalState WDGState);
void WDT_SW(void);
void RunLed_ON(void);
void RunLed_OFF(void);
void CommLed_ON(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void CommLed_OFF(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LED_Flash(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void FrontLED_Set(u8 led,u8 OnOff);
void Contact_Set(u8 Contactx, u8 OnOff);
enum Contact_State Contact_ReadState(u8 Contactx);
void Relay_Set(u8 Relayx, u8 OnOff);
enum Relay_State Relay_ReadState(u8 Relayx);
u8 CAN_GetAddr(void);
void RS485_Init(void);






#endif /* __GPIO_H */



