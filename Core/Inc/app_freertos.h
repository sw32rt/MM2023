#ifndef APP_FREERTOS_H_
#define APP_FREERTOS_H_

extern osMessageQueueId_t soundQueueHandle;
extern osSemaphoreId_t measurementTriggerSemaphoreHandle;
extern osSemaphoreId_t i2cTxCompleteSemaphoreHandle;
extern osSemaphoreId_t i2cRxCompleteSemaphoreHandle;


#endif /* APP_FREERTOS_H_ */
