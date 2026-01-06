/*
 * x5_lite.h
 *
 *  Created on: Dec 15, 2025
 *      Author: rith
 */
#ifndef __X5_LITE_H
#define __X5_LITE_H


#include "main.h"
#include <stdbool.h>
#include <stdint.h>


/* ================= Button bit masks ================= */
#define X5_A      (1<<0)
#define X5_B      (1<<1)
#define X5_X      (1<<2)
#define X5_Y      (1<<3)
#define X5_SHERE  (1<<4)
#define X5_CHIKEN (1<<5)
#define X5_MENU   (1<<6)
#define X5_L3     (1<<7)
#define X5_R3     (1<<8)
#define X5_LB     (1<<9)
#define X5_RB     (1<<10)
#define X5_UP     (1<<11)
#define X5_DOWN   (1<<12)
#define X5_LEFT   (1<<13)
#define X5_RIGHT  (1<<14)
#define X5_LT     (1<<15)
#define X5_RT     (1<<16)

//bits_mask Meihua forest

#define B1 (1<<15)
#define B2 (1<<14)
#define B3 (1<<13)
#define B4 (1<<12)
#define B5 (1<<11)
#define B6 (1<<10)
#define B7 (1<<9)
#define B8 (1<<8)
#define B9 (1<<7)
#define B10 (1<<6)
#define B11 (1<<5)
#define B12 (1<<4)
#define Start_r (1<<3)
#define Retry_r (1<<2)
#define Start_b (1<<1)
#define Retry_b (1<<0)

/* ================= Data structures ================= */
#pragma pack(push,1)
typedef struct {
    int8_t  rx;
    int8_t  ry;
    int8_t  lx;
    int8_t  ly;
    uint32_t button;
    uint16_t buttonM;
} X5_RawData;
#pragma pack(pop)

typedef struct {
    bool a, b, x, y;
    bool shere, chiken, menu;
    bool l3, r3, lb, rb;
    bool up, down, left, right;
    bool lt, rt;
} X5_Button;
typedef struct{
	bool b0,b1,b2,b3;
	bool b4,b5,b6,b7;
	bool b8,b9,b10,b11;
	bool start_b,retry_b,start_r,retry_r;

}x5_Maihua;
typedef struct{
	int8_t  rx;
	int8_t  ry;
	int8_t  lx;
	int8_t  ly;
}X5_JoyStick;

/* ================= API ================= */
void X5_Init(SPI_HandleTypeDef *hspi);
void X5_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void X5_UpdateButtons(void);
void X5_Joy(int8_t *rx, int8_t *ry, int8_t *lx, int8_t *ly);
extern volatile X5_RawData  x5_data;
extern volatile X5_Button  x5_button;
extern volatile x5_Maihua Meihua;



#endif /* __X5_LITE_H */

