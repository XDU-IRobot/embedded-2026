#include "ws2812b.hpp"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness
uint16_t pwmData[(24*MAX_LED)+50];


void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
    LED_Data[LEDnum][0] = LEDnum;
    LED_Data[LEDnum][1] = Green;
    LED_Data[LEDnum][2] = Red;
    LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

    if (brightness > 45) brightness = 45;
    for (int i=0; i<MAX_LED; i++)
    {
        LED_Mod[i][0] = LED_Data[i][0];
        for (int j=1; j<4; j++)
        {
            float angle = 90-brightness;  // in degrees
            angle = angle*3.14159265 / 180;  // in rad
            LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
        }
    }
#endif
}

void WS2812_Send (void)
{
    uint32_t indx=0;
    uint32_t color;


    for (int i= 0; i<MAX_LED; i++)
    {
#if USE_BRIGHTNESS
        color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
        color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

        for (int i=23; i>=0; i--)
        {
            if (color&(1<<i))
            {
                pwmData[indx] = 140;  // 2/3 of 90
            }

            else pwmData[indx] = 70;  // 1/3 of 90

            indx++;
        }

    }

    for (int i=0; i<50; i++)
    {
        pwmData[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == htim1.Instance)
    {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    }
}

