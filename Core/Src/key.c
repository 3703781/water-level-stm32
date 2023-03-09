#include "key.h"

#define KEY0 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)  // KEY0按键PC5
#define KEY1 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) // KEY1按键PA15
#define WK_UP HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) // WKUP按键PA0

void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOA_CLK_ENABLE(); // 开启GPIOA时钟
    __HAL_RCC_GPIOC_CLK_ENABLE(); // 开启GPIOC时钟

    GPIO_Initure.Pin = GPIO_PIN_0;             // PA0
    GPIO_Initure.Mode = GPIO_MODE_INPUT;       // 输入
    GPIO_Initure.Pull = GPIO_PULLDOWN;         // 下拉
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_LOW; // 高速
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    GPIO_Initure.Pin = GPIO_PIN_15;  // PA15
    GPIO_Initure.Pull = GPIO_PULLUP; // 上拉
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    GPIO_Initure.Pin = GPIO_PIN_5;   // PC5
    GPIO_Initure.Pull = GPIO_PULLUP; // 上拉
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);
}

// 按键处理函数
// 返回按键值
// mode:0,不支持连续按;1,支持连续按;
// 0，没有任何按键按下
// 1，WKUP按下 WK_UP
// 注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan(uint8_t mode)
{
    static uint8_t key_up = 1; // 按键松开标志
    if (mode == 1)
        key_up = 1; // 支持连按
    if (key_up && (KEY0 == 0 || KEY1 == 0 || WK_UP == 1))
    {
        HAL_Delay(5);
        key_up = 0;
        if (KEY0 == 0)
            return KEY0_PRES;
        else if (KEY1 == 0)
            return KEY1_PRES;
        else if (WK_UP == 1)
            return WKUP_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && WK_UP == 0)
        key_up = 1;
    return 0; // 无按键按下
}
