#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "image.h"
/************/
#include "cv.h"
#include "highgui.h"
/************/
//#include "SmartCar_MT9V034.h"
//#include "common.h"
//#include"Six_Control.h"


#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define OUT_H	100//120							//逆透视高度
#define OUT_W	140//188							//逆透视宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 85//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数
#define ImgUsed	*ImgPerspect

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t* ImgPerspect[OUT_H][OUT_W];//逆透视后图像数组
extern uint8_t mt9v034_image[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];//中线
extern uint8_t threshold;//阈值

extern uint8_t State;
extern uint8_t State_Pre;
extern uint8_t State_Error;
extern uint8_t Flag_Stop;
extern uint8_t Flag_Zebra;
extern uint8_t Flag_Curve;
extern uint8_t Flag_Strai;
extern uint8_t Flag_Error;

enum Road_State {
	Zebra = 1, CrossFront, CrossEnter, CrossBias,
};//车道各状态

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);

/*************************/
uint8_t OTSU_Get(void);
uint8_t Park_Protect(void);
uint8_t Cross_Judge(void);
void Cross_Line(void);
uint8_t Cross_Bias_Judge(void);
void Cross_Bias_Line(void);
uint8_t Cross_Enter_Judge(void);
void Cross_Enter_Line(void);
uint8_t Zebra_Judge(void);
void Zebra_Line(void);
void Zebra_Stop(void);
void Curve_Judge(void);
void Straight_Judge(void);
void Road_Correct(void);
void Least_Square(uint8_t, uint8_t, uint8_t, uint8_t, int);
void Image_Filter(void);
//void Image_Perspect(void);

#endif //