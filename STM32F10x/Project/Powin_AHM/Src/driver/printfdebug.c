/**
  ******************************************************************************
  * @file    USART/HyperTerminal_Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
 
/* Includes ------------------------------------------------------------------*/
#include "printfdebug.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

uint8_t TxBuffer[] = "\n\rUSART Hyperterminal Interrupts Example: USART-Hyperterminal\
 communication using Interrupt\n\r";
//uint8_t RxBuffer[DEBUG_UART_RxBufferSize];
//uint8_t NbrOfDataToRead = DEBUG_UART_RxBufferSize;
//uint16_t RxCounter = 0; 
//uint16_t RxCounterProcess = 0; 
static QueueHandle_t xRxedChars;

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			            ( ( TickType_t ) 0xffff )
#define mainNORMAL_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
/* Private function prototypes -----------------------------------------------*/


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
    #error
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#if(USER_FREERTOS)
static void vDebugUartTask( void *pvParameters )
{
    //vTaskDelay(2000); 
    signed char cByteRxed;
    printf("vDebugUartTask Going...\r\n");        
    for(;;)
    {
        //printf("|");  
        //vTaskDelay(1000); 
        if( xQueueReceive( xRxedChars, &cByteRxed, comRX_BLOCK_TIME) )
        {
            printf("<%c>\n", cByteRxed);
        }
        else
        {
            printf("vDebugUartTask xQueueReceive err\n");
        }
    }
}

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void PrintfInit(void)
{    
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    #if (DEBUG_TARGET_UART == DEBUG_UART_1)
    #if(1)
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 


    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    GPIO_PinRemapConfig(GPIO_Remap_USART1 , ENABLE);
    #else
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 


    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);    
    #endif    
    #elif (DEBUG_TARGET_UART == DEBUG_UART_3)
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 


    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    #elif (DEBUG_TARGET_UART == DEBUG_UART_5)
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); 


    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);   
    #endif
    
    /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* USART configuration */
    USART_Init(DEBUG_UART, &USART_InitStructure);
#if(USER_FREERTOS)
    xRxedChars = xQueueCreate( DEBUG_UART_RxBufferSize, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
    xTaskCreate( vDebugUartTask, "vDebugUartTask", 128, NULL, mainNORMAL_TASK_PRIORITY, NULL );
    
    /* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the 
    EVAL_COM1 receive data register is not empty */
    USART_ITConfig(DEBUG_UART, USART_IT_RXNE, ENABLE);
    
    NVIC_Configuration();
#endif    
    /* Enable USART */
    USART_Cmd(DEBUG_UART, ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    #if(0)   
    return ITM_SendChar(ch);
    #else
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(DEBUG_UART, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC) == RESET)
    {}
    return ch;
    #endif
}


/**
  * @brief  This function handles USARTx global interrupt request.
  * @param  None
  * @retval None
  */
void DEBUG_USARTx_IRQHandler(void)
{
    char cChar;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    //printf("^^");
    if(USART_GetITStatus(DEBUG_UART, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
        cChar = (USART_ReceiveData(DEBUG_UART) & 0x7F);
        xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
    }
}
