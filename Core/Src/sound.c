
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "app_freertos.h"
#include "sound.h"
#include "stdint.h"
#include "tim.h"

/* Private define ------------------------------------------------------------*/
#define     NOTE_A0    (27.500  )    /* ラ0     */
#define     NOTE_As0   (29.135  )    /* ラ#0    */
#define     NOTE_B0    (30.868  )    /* シ0     */
#define     NOTE_C1    (32.703  )    /* ド1     */
#define     NOTE_Cs1   (34.648  )    /* ド#1    */
#define     NOTE_D1    (36.708  )    /* レ1     */
#define     NOTE_Ds1   (38.891  )    /* レ#1    */
#define     NOTE_E1    (41.203  )    /* ミ1     */
#define     NOTE_F1    (43.654  )    /* ファ1   */
#define     NOTE_Fs1   (46.249  )    /* ファ#1  */
#define     NOTE_G1    (48.999  )    /* ソ1     */
#define     NOTE_Gs1   (51.913  )    /* ソ#1    */
#define     NOTE_A1    (55.000  )    /* ラ1     */
#define     NOTE_As1   (58.270  )    /* ラ#1    */
#define     NOTE_B1    (61.735  )    /* シ1     */
#define     NOTE_C2    (65.406  )    /* ド2     */
#define     NOTE_Cs2   (69.296  )    /* ド#2    */
#define     NOTE_D2    (73.416  )    /* レ2     */
#define     NOTE_Ds2   (77.782  )    /* レ#2    */
#define     NOTE_E2    (82.407  )    /* ミ2     */
#define     NOTE_F2    (87.307  )    /* ファ2   */
#define     NOTE_Fs2   (92.499  )    /* ファ#2  */
#define     NOTE_G2    (97.999  )    /* ソ2     */
#define     NOTE_Gs2   (103.826 )    /* ソ#2    */
#define     NOTE_A2    (110.000 )    /* ラ2     */
#define     NOTE_As2   (116.541 )    /* ラ#2    */
#define     NOTE_B2    (123.471 )    /* シ2     */
#define     NOTE_C3    (130.813 )    /* ド3     */
#define     NOTE_Cs3   (138.591 )    /* ド#3    */
#define     NOTE_D3    (146.832 )    /* レ3     */
#define     NOTE_Ds3   (155.563 )    /* レ#3    */
#define     NOTE_E3    (164.814 )    /* ミ3     */
#define     NOTE_F3    (174.614 )    /* ファ3   */
#define     NOTE_Fs3   (184.997 )    /* ファ#3  */
#define     NOTE_G3    (195.998 )    /* ソ3     */
#define     NOTE_Gs3   (207.652 )    /* ソ#3    */
#define     NOTE_A3    (220.000 )    /* ラ3     */
#define     NOTE_As3   (233.082 )    /* ラ#3    */
#define     NOTE_B3    (246.942 )    /* シ3     */
#define     NOTE_C4    (261.626 )    /* ド4     */
#define     NOTE_Cs4   (277.183 )    /* ド#4    */
#define     NOTE_D4    (293.665 )    /* レ4     */
#define     NOTE_Ds4   (311.127 )    /* レ#4    */
#define     NOTE_E4    (329.628 )    /* ミ4     */
#define     NOTE_F4    (349.228 )    /* ファ4   */
#define     NOTE_Fs4   (369.994 )    /* ファ#4  */
#define     NOTE_G4    (391.995 )    /* ソ4     */
#define     NOTE_Gs4   (415.305 )    /* ソ#4    */
#define     NOTE_A4    (440.000 )    /* ラ4     */
#define     NOTE_As4   (466.164 )    /* ラ#4    */
#define     NOTE_B4    (493.883 )    /* シ4     */
#define     NOTE_C5    (523.251 )    /* ド5     */
#define     NOTE_Cs5   (554.365 )    /* ド#5    */
#define     NOTE_D5    (587.330 )    /* レ5     */
#define     NOTE_Ds5   (622.254 )    /* レ#5    */
#define     NOTE_E5    (659.255 )    /* ミ5     */
#define     NOTE_F5    (698.456 )    /* ファ5   */
#define     NOTE_Fs5   (739.989 )    /* ファ#5  */
#define     NOTE_G5    (783.991 )    /* ソ5     */
#define     NOTE_Gs5   (830.609 )    /* ソ#5    */
#define     NOTE_A5    (880.000 )    /* ラ5     */
#define     NOTE_As5   (932.328 )    /* ラ#5    */
#define     NOTE_B5    (987.767 )    /* シ5     */
#define     NOTE_C6    (1046.502)    /* ド6     */
#define     NOTE_Cs6   (1108.731)    /* ド#6    */
#define     NOTE_D6    (1174.659)    /* レ6     */
#define     NOTE_Ds6   (1244.508)    /* レ#6    */
#define     NOTE_E6    (1318.510)    /* ミ6     */
#define     NOTE_F6    (1396.913)    /* ファ6   */
#define     NOTE_Fs6   (1479.978)    /* ファ#6  */
#define     NOTE_G6    (1567.982)    /* ソ6     */
#define     NOTE_Gs6   (1661.219)    /* ソ#6    */
#define     NOTE_A6    (1760.000)    /* ラ6     */
#define     NOTE_As6   (1864.655)    /* ラ#6    */
#define     NOTE_B6    (1975.533)    /* シ6     */
#define     NOTE_C7    (2093.005)    /* ド7     */
#define     NOTE_Cs7   (2217.461)    /* ド#7    */
#define     NOTE_D7    (2349.318)    /* レ7     */
#define     NOTE_Ds7   (2489.016)    /* レ#7    */
#define     NOTE_E7    (2637.020)    /* ミ7     */
#define     NOTE_F7    (2793.826)    /* ファ7   */
#define     NOTE_Fs7   (2959.955)    /* ファ#7  */
#define     NOTE_G7    (3135.963)    /* ソ7     */
#define     NOTE_Gs7   (3322.438)    /* ソ#7    */
#define     NOTE_A7    (3520.000)    /* ラ7     */
#define     NOTE_As7   (3729.310)    /* ラ#7    */
#define     NOTE_B7    (3951.066)    /* シ7     */
#define     NOTE_C8    (4186.009)    /* ド8     */

#define COUNTER_PRESCALER (8)
#define COUNTER_INPUT_Hz (170000000 / (COUNTER_PRESCALER + 1))

#define A0   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A0 ) + 1)       /* ラ0     */
#define As0  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As0) + 1)       /* ラ#0    */
#define B0   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B0 ) + 1)       /* シ0     */
#define C1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C1 ) + 1)       /* ド1     */
#define Cs1  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs1) + 1)       /* ド#1    */
#define D1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D1 ) + 1)       /* レ1     */
#define Ds1  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds1) + 1)       /* レ#1    */
#define E1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E1 ) + 1)       /* ミ1     */
#define F1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F1 ) + 1)       /* ファ1   */
#define Fs1  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs1) + 1)       /* ファ#1  */
#define G1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G1 ) + 1)       /* ソ1     */
#define Gs1  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs1) + 1)       /* ソ#1    */
#define A1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A1 ) + 1)       /* ラ1     */
#define As1  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As1) + 1)       /* ラ#1    */
#define B1   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B1 ) + 1)       /* シ1     */
#define C2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C2 ) + 1)       /* ド2     */
#define Cs2  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs2) + 1)       /* ド#2    */
#define D2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D2 ) + 1)       /* レ2     */
#define Ds2  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds2) + 1)       /* レ#2    */
#define E2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E2 ) + 1)       /* ミ2     */
#define F2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F2 ) + 1)       /* ファ2   */
#define Fs2  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs2) + 1)       /* ファ#2  */
#define G2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G2 ) + 1)       /* ソ2     */
#define Gs2  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs2) + 1)       /* ソ#2    */
#define A2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A2 ) + 1)       /* ラ2     */
#define As2  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As2) + 1)       /* ラ#2    */
#define B2   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B2 ) + 1)       /* シ2     */
#define C3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C3 ) + 1)       /* ド3     */
#define Cs3  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs3) + 1)       /* ド#3    */
#define D3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D3 ) + 1)       /* レ3     */
#define Ds3  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds3) + 1)       /* レ#3    */
#define E3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E3 ) + 1)       /* ミ3     */
#define F3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F3 ) + 1)       /* ファ3   */
#define Fs3  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs3) + 1)       /* ファ#3  */
#define G3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G3 ) + 1)       /* ソ3     */
#define Gs3  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs3) + 1)       /* ソ#3    */
#define A3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A3 ) + 1)       /* ラ3     */
#define As3  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As3) + 1)       /* ラ#3    */
#define B3   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B3 ) + 1)       /* シ3     */
#define C4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C4 ) + 1)       /* ド4     */
#define Cs4  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs4) + 1)       /* ド#4    */
#define D4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D4 ) + 1)       /* レ4     */
#define Ds4  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds4) + 1)       /* レ#4    */
#define E4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E4 ) + 1)       /* ミ4     */
#define F4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F4 ) + 1)       /* ファ4   */
#define Fs4  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs4) + 1)       /* ファ#4  */
#define G4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G4 ) + 1)       /* ソ4     */
#define Gs4  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs4) + 1)       /* ソ#4    */
#define A4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A4 ) + 1)       /* ラ4     */
#define As4  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As4) + 1)       /* ラ#4    */
#define B4   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B4 ) + 1)       /* シ4     */
#define C5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C5 ) + 1)       /* ド5     */
#define Cs5  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs5) + 1)       /* ド#5    */
#define D5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D5 ) + 1)       /* レ5     */
#define Ds5  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds5) + 1)       /* レ#5    */
#define E5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E5 ) + 1)       /* ミ5     */
#define F5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F5 ) + 1)       /* ファ5   */
#define Fs5  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs5) + 1)       /* ファ#5  */
#define G5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G5 ) + 1)       /* ソ5     */
#define Gs5  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs5) + 1)       /* ソ#5    */
#define A5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A5 ) + 1)       /* ラ5     */
#define As5  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As5) + 1)       /* ラ#5    */
#define B5   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B5 ) + 1)       /* シ5     */
#define C6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C6 ) + 1)       /* ド6     */
#define Cs6  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs6) + 1)       /* ド#6    */
#define D6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D6 ) + 1)       /* レ6     */
#define Ds6  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds6) + 1)       /* レ#6    */
#define E6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E6 ) + 1)       /* ミ6     */
#define F6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F6 ) + 1)       /* ファ6   */
#define Fs6  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs6) + 1)       /* ファ#6  */
#define G6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G6 ) + 1)       /* ソ6     */
#define Gs6  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs6) + 1)       /* ソ#6    */
#define A6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A6 ) + 1)       /* ラ6     */
#define As6  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As6) + 1)       /* ラ#6    */
#define B6   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B6 ) + 1)       /* シ6     */
#define C7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C7 ) + 1)       /* ド7     */
#define Cs7  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Cs7) + 1)       /* ド#7    */
#define D7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_D7 ) + 1)       /* レ7     */
#define Ds7  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Ds7) + 1)       /* レ#7    */
#define E7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_E7 ) + 1)       /* ミ7     */
#define F7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_F7 ) + 1)       /* ファ7   */
#define Fs7  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Fs7) + 1)       /* ファ#7  */
#define G7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_G7 ) + 1)       /* ソ7     */
#define Gs7  (uint16_t)((COUNTER_INPUT_Hz / NOTE_Gs7) + 1)       /* ソ#7    */
#define A7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_A7 ) + 1)       /* ラ7     */
#define As7  (uint16_t)((COUNTER_INPUT_Hz / NOTE_As7) + 1)       /* ラ#7    */
#define B7   (uint16_t)((COUNTER_INPUT_Hz / NOTE_B7 ) + 1)       /* シ7     */
#define C8   (uint16_t)((COUNTER_INPUT_Hz / NOTE_C8 ) + 1)       /* ド8     */
#define REST (uint16_t)(0)

#define SCORE_MEASURE_RESOLUTION 4
#define SCORE_MEASURE 4
#define SCORE_MEASURE_SETS 8 
#define BPM (180) * SCORE_MEASURE_RESOLUTION
#define UPDATE_RETE (1000 / (BPM / 60))

/* Private variables ---------------------------------------------------------*/

const uint16_t music1[][SCORE_MEASURE][SCORE_MEASURE_RESOLUTION] = 
{
    {
        {
            A4,
            A4,
            A4,
            REST,
        },
        {
            C5,
            C5,
            C5,
            REST,
        },
        {
            Ds5,
            Ds5,
            Ds5,
            E5,
        },
        {
            E5,
            E5,
            Gs4,
            REST,
        },
    },
    {
        {
            A4,
            A4,
            C5,
            C5,
        },
        {
            D5,
            D5,
            C5,
            C5,
        },
        {
            Ds5,
            REST,
            D5,
            E5,
        },
        {
            E5,
            E5,
            E5,
            REST,
        },
    },
    {
        {
            G5,
            G5,
            Fs5,
            Fs5,
        },
        {
            F5,
            F5,
            E5,
            E5,
        },
        {
            D5,
            D5,
            C5,
            C5,
        },
        {
            B4,
            B4,
            As4,
            As4,
        },
    },
    {
        {
            B4,
            B4,
            C5,
            C5,
        },
        {
            D5,
            D5,
            Ds5,
            Ds5,
        },
        {
            E5,
            E5,
            E5,
            E5,
        },
        {
            E4,
            E4,
            E4,
            E4,
        },
    },

    {
        {
            A4,
            A4,
            A4,
            REST,
        },
        {
            C5,
            C5,
            C5,
            REST,
        },
        {
            Ds5,
            Ds5,
            Ds5,
            E5,
        },
        {
            E5,
            E5,
            Gs4,
            REST,
        },
    },
    {
        {
            A4,
            A4,
            C5,
            C5,
        },
        {
            D5,
            D5,
            C5,
            C5,
        },
        {
            Ds5,
            REST,
            D5,
            E5,
        },
        {
            E5,
            E5,
            E5,
            REST,
        },
    },
    {
        {
            G5,
            G5,
            Fs5,
            Fs5,
        },
        {
            F5,
            F5,
            E5,
            REST,
        },
        {
            E5,
            REST,
            F5,
            F5,
        },
        {
            Fs5,
            Fs5,
            G5,
            G5,
        },
    },
    {
        {
            Gs5,
            REST,
            G5,
            Gs5,
        },
        {
            A5,
            REST,
            A5,
            REST,
        },
        {
            B5,
            B5,
            B5,
            B5,
        },
        {
            B5,
            B5,
            B5,
            B5,
        },
    },
};

const score_t score[] = 
{
    [SCORE_INDEX_0] = 
    {
        .pScore = (uint16_t*)music1,
        .length = (sizeof(music1) / sizeof(uint16_t))
    },
};

/* Private function prototypes -----------------------------------------------*/
void soundOut(uint16_t period);


/* Public function code --------------------------------------------------*/
/**
  * @brief  sound play thread.
  * @param  argument: Not used
  * @retval None
  */
void g_SoundTask(void *argument)
{
    osStatus_t err = 0;
    scoreIndex Index;

    while(1)
    {
        err = osMessageQueueGet(soundQueueHandle, &Index, NULL, osWaitForever);
        if (err == osOK)
        {
            for(int iterator = 0; iterator < score[Index].length; iterator++)
            {
                soundOut(score[Index].pScore[iterator]);
                osDelay(UPDATE_RETE);
            }
        }
    }
}

/**
  * @brief  sound component initialize.
  * @param  argument: Not used
  * @retval None
  */
void g_SoundInit(void)
{
    __HAL_TIM_SET_AUTORELOAD(&htim8, 42928);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); /* 消音 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

/* Private application code --------------------------------------------------*/
/**
  * @brief  set register, sound out.
  * @param  period: timer period
  * @retval None
  */
void soundOut(uint16_t period)
{
    if(period == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); /* 消音 */
    }
    else
    {
        __HAL_TIM_SET_AUTORELOAD(&htim8, period);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, period/2); /* 50% duty */
    }
}
