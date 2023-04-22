#include "tof.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"
#include "sensor.h"
#include "app_freertos.h" 
#include "custom_ranging_sensor.h"

#include "debug.h"

static void TofInit(void);


    uint32_t diff;
void g_TofRangingTask(void *argument)
{
    VL53L4CD_Result_t result = {0};

    uint32_t start;
    uint32_t stop;

    stopWatchInit();
    TofInit();

    while(1)
    {
        osSemaphoreAcquire(measurementTriggerSemaphoreHandle, osWaitForever);
        // osDelay(10);

start = getClock();
        CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_L, &result);
        g_sensor.range_l = result.ZoneResult[0].Distance[0];
        CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_C, &result);
        g_sensor.range_f = result.ZoneResult[0].Distance[0];
        CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_R, &result);
        g_sensor.range_r = result.ZoneResult[0].Distance[0];
stop = getClock();
diff = getTimeUs(start, stop);
(void)diff;
    }
}


static void TofInit(void)
{
    VL53L4CD_ProfileConfig_t tofConfig = {0};
    
    tofConfig.RangingProfile = VL53L4CD_PROFILE_CONTINUOUS;
    tofConfig.TimingBudget   = 10;
    tofConfig.Frequency      = 0;
    tofConfig.EnableAmbient  = 0;
    tofConfig.EnableSignal   = 0;

    CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_L);
    CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_C);
    CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_R);
    CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_L, &tofConfig);
    CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_C, &tofConfig);
    CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_R, &tofConfig);
    CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_L, VL53L4CD_MODE_ASYNC_CONTINUOUS);
    CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_C, VL53L4CD_MODE_ASYNC_CONTINUOUS);
    CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_R, VL53L4CD_MODE_ASYNC_CONTINUOUS);

}
