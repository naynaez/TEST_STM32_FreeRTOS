# TEST_STM32_FreeRTOS  
Testing FreeRTOS with analog sensor and UART communication on STM32F401RE Nucleo by separate into 2 tasks.  

### Configuration	of TEST_STM32_FreeRTOS
STM32CubeMX 	4.20.0  
MCU	STM32F401RETx  


**PERIPHERALS , MODES , FUNCTIONS , PINS**  
{ADC1 , IN0 , ADC1_IN0 , PA0-WKUP}  
{TIM2 , Internal Clock , TIM2_VS_ClockSourceINT , VP_TIM2_VS_ClockSourceINT}  
{USART2 , Asynchronous , USART2_RX , PA3}  
{USART2	, Asynchronous , USART2_TX , PA2}  


**PinNb , PINs , FUNCTIONs , LABELs**  
{2 , PC13-ANTI_TAMP , GPIO_EXTI13 , BLUE_BUTTON}  
{14 , PA0-WKUP , ADC1_IN0 , ANALOG_SENSOR_IN}  
{16 , PA2 , USART2_TX , , }  
{17 , PA3 , USART2_RX , , }  
{21 , PA5 , GPIO_Output , LED2 }  
