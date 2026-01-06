/*
 * x5_lite.c
 *
 *  Created on: Dec 15, 2025
 *      Author: rith
 */

#include "x5_lite.h"
#include <string.h>


/* ================= Private variables ================= */
static SPI_HandleTypeDef *x5_hspi;
static uint8_t rxBuf[sizeof(X5_RawData)];

volatile X5_RawData x5_data;
volatile X5_Button  x5_button;
volatile x5_Maihua Meihua;

/* ================= Functions ================= */

void X5_Init(SPI_HandleTypeDef *hspi)
{
    x5_hspi = hspi;
    HAL_SPI_Receive_IT(x5_hspi, rxBuf, sizeof(rxBuf));
}

void X5_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == x5_hspi)
    {
        memcpy((void *)&x5_data, rxBuf, sizeof(X5_RawData));
        HAL_SPI_Receive_IT(x5_hspi, rxBuf, sizeof(rxBuf));
    }
}

void X5_UpdateButtons(void)
{
    X5_Button tmp = {
        .a      = (x5_data.button & X5_A),
        .b      = (x5_data.button & X5_B),
        .x      = (x5_data.button & X5_X),
        .y      = (x5_data.button & X5_Y),
        .up     = (x5_data.button & X5_UP),
        .down   = (x5_data.button & X5_DOWN),
        .left   = (x5_data.button & X5_LEFT),
        .right  =(x5_data.button & X5_RIGHT),
        .lb     = (x5_data.button & X5_LB),
        .rb     = (x5_data.button & X5_RB),
        .lt     = (x5_data.button & X5_LT),
        .rt     = (x5_data.button & X5_RT),
        .l3     = (x5_data.button & X5_L3),
        .r3     = (x5_data.button & X5_R3),
        .shere  = (x5_data.button & X5_SHERE),
        .chiken = (x5_data.button & X5_CHIKEN),
        .menu   = (x5_data.button & X5_MENU)
    };

    x5_button = tmp;
}
void X5_Meihua(void){
	x5_Maihua data_m ={
			.b0 =!(x5_data.buttonM & B1), .b1 =!(x5_data.buttonM& B2) ,.b2 =!(x5_data.buttonM & B3),  .b3 =!(x5_data.buttonM & B4),
			.b4 =!(x5_data.buttonM & B5), .b5 =!(x5_data.buttonM & B6) ,.b6 =!(x5_data.buttonM & B7),  .b7 =!(x5_data.buttonM & B8),
			.b8 =!(x5_data.buttonM & B9), .b9 =!(x5_data.buttonM & B10), .b10 =!(x5_data.buttonM & B11), .b11 =!(x5_data.buttonM & B12),
			.start_b =!(x5_data.buttonM & Start_b), .retry_b =!(x5_data.buttonM & Retry_b ), .start_r =!(x5_data.buttonM & Start_r),
			.retry_r= !(x5_data.buttonM & Retry_r)
	};
	Meihua = data_m;

}
void X5_Joy(int8_t *rx, int8_t *ry, int8_t *lx, int8_t *ly)
{
    *rx = x5_data.rx;
    *ry = x5_data.ry;
    *lx = x5_data.lx;
    *ly = x5_data.ly;
}

