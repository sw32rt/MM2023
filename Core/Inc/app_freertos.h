#ifndef APP_FREERTOS_H_
#define APP_FREERTOS_H_

/* thread handles */
extern osThreadId_t soundTaskHandle;
extern osThreadId_t tofRangingTaskHandle;
extern osThreadId_t logTaskHandle;
extern osThreadId_t uartRecvTaskHandle;
extern osThreadId_t uartSendTaskHandle;
extern osThreadId_t uartCommandTaskHandle;

/* queue handles */
extern osMessageQueueId_t soundQueueHandle;
extern osMessageQueueId_t logQueueHandle;
extern osMessageQueueId_t uartRecvQueueHandle;
extern osMessageQueueId_t uartSendQueueHandle;

/* timer handles */
extern osTimerId_t Timer1kHzHandle;

/* semaphore handles */
extern osSemaphoreId_t measurementTriggerSemaphoreHandle;
extern osSemaphoreId_t i2cTxCompleteSemaphoreHandle;
extern osSemaphoreId_t i2cRxCompleteSemaphoreHandle;
extern osSemaphoreId_t logTriggerSemaphoreHandle;
extern osSemaphoreId_t uartRecvCompleteSemaphoreHandle;
extern osSemaphoreId_t uartSendCompleteSemaphoreHandle;


#endif /* APP_FREERTOS_H_ */
