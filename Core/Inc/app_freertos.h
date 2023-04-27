#ifndef APP_FREERTOS_H_
#define APP_FREERTOS_H_

extern osThreadId_t soundTaskHandle;
extern osThreadId_t tofRangingTaskHandle;
extern osThreadId_t logTaskHandle;
extern osMessageQueueId_t soundQueueHandle;
extern osSemaphoreId_t measurementTriggerSemaphoreHandle;
extern osSemaphoreId_t i2cTxCompleteSemaphoreHandle;
extern osSemaphoreId_t i2cRxCompleteSemaphoreHandle;
extern osSemaphoreId_t logTriggerSemaphoreHandle;


#endif /* APP_FREERTOS_H_ */
