#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"
#include <math.h>
#include "image.h"

//各种颜色 用于显示和调试///////////////////////////////////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
CvScalar col_white, col_black, col_blue, col_red, col_green, col_gray, col_purple;
////////////////////////////////////////////////////////////////////////////////////
#define change_speed 2								//图片切换速度
#define NUMOFSTART 0								//起始图片标号
#define NUMOFEND 4073								//结束图片标号
IplImage* pImg_1 = NULL;							//原图图像指针
IplImage* pImg_2 = NULL;							//当前图图像指针
char path[1000];									//保存路径的字符数组
int have_paint_midline = 0, pic_num = 0;
uint8_t mt9v034_image[CAMERA_H][CAMERA_W];
uint8_t* fullBuffer;


void col_init(void);
void create_window(void);
void create_pic_now_background(void);
void load_pic(void);
void show_pic(void);
void on_mouse(int event, int x, int y, int flag, void* param);
void cvText(IplImage* image, const char* text, int x, int y, double h, double v, int line_width);
void find_midline(void);

void col_init(void)
{
	col_white.val[0] = 255; col_white.val[1] = 255; col_white.val[2] = 255;
	col_black.val[0] = 0;   col_black.val[1] = 0;   col_black.val[2] = 0;
	col_blue.val[0] = 255;  col_blue.val[1] = 0;    col_blue.val[2] = 0;
	col_green.val[0] = 0;   col_green.val[1] = 255; col_green.val[2] = 0;
	col_purple.val[0] = 255;  col_purple.val[1] = 0; col_purple.val[2] = 255;
	col_red.val[0] = 0;     col_red.val[1] = 0;     col_red.val[2] = 255;
	col_gray.val[0] = 100;     col_gray.val[1] = 100;     col_gray.val[2] = 100;
}

void create_window(void)
{
	cvNamedWindow("原图", 0);//创建窗口
	cvNamedWindow("当前图", 0);//创建窗口
}

void create_pic_now_background(void)
{
	int width = 0;
	int height = 0;
	int i, j;
	CvScalar s;
	width = CAMERA_W + 80;
	height = CAMERA_H + 80;
	pImg_2 = cvCreateImage(cvSize(width, height), 8, 3);
	for (i = 0; i < height; i++)//全屏涂白
	{
		for (j = 0; j < width; j++)
		{
			cvSet2D(pImg_2, i, j, col_white);
		}
	}

	for (i = 5; i < 20; i++)//start
	{
		for (j = CAMERA_W + 10; j < CAMERA_W + 60; j++)
		{
			cvSet2D(pImg_2, i, j, col_black);
		}
	}
	cvText(pImg_2, "start", CAMERA_W + 10, 15, 0.4, 0.4, 1);

	for (i = 25; i < 40; i++)//midline
	{
		for (j = CAMERA_W + 10; j < CAMERA_W + 60; j++)
		{
			cvSet2D(pImg_2, i, j, col_black);
		}
	}
	cvText(pImg_2, "midline", CAMERA_W + 10, 35, 0.4, 0.4, 1);

	for (i = 45; i < 60; i++)//turn to
	{
		for (j = CAMERA_W + 10; j < CAMERA_W + 60; j++)
		{
			cvSet2D(pImg_2, i, j, col_black);
		}
	}
	cvText(pImg_2, "turn_to", CAMERA_W + 10, 55, 0.4, 0.4, 1);

	for (i = 65; i < 80; i++)//draw
	{
		for (j = CAMERA_W + 10; j < CAMERA_W + 60; j++)
		{
			cvSet2D(pImg_2, i, j, col_black);
		}
	}
	cvText(pImg_2, "draw", CAMERA_W + 10, 75, 0.4, 0.4, 1);

	for (i = 85; i < 100; i++)//restart
	{
		for (j = CAMERA_W + 10; j < CAMERA_W + 60; j++)
		{
			cvSet2D(pImg_2, i, j, col_black);
		}
	}
	cvText(pImg_2, "restart", CAMERA_W + 10, 95, 0.4, 0.4, 1);

}


void load_pic(void)
{
	have_paint_midline = 0;

	sprintf(path, "C:\\Users\\user\\Desktop\\Last2\\%d.jpeg", pic_num + 2);//请在此处修改路径！！！如路径为“C:\\桌面\\fiscar\\2.jpg”，修改函数为：sprintf(pos, "C:\\桌面\\fiscar\\%d.jpg", 2);
	//图片如果在工程文件夹中不需要标注路径

	pImg_1 = cvLoadImage(path, 0);
	if (pImg_1 == NULL)
	{
		return;
		printf("can not load picture!");
	}

	cvShowImage("原图", pImg_1);

	CvScalar s;
	int i, j;
	for (i = 0; i < CAMERA_H; i++)
	{
		for (j = 0; j < CAMERA_W; j++)
		{
			s = cvGet2D(pImg_1, i, j);
			mt9v034_image[i][j] = s.val[0];
		}
	}
	fullBuffer = &mt9v034_image[0][0];
	THRE();
	Image_Filter();
	head_clear();
	return;
}


void show_pic(void)
{
	int i, j;
	CvScalar s;
	char my_text[60];
	//显示当前图
	for (i = 0; i < CAMERA_H; i++)
	{
		uint8_t* map = NULL;
		map = &IMG[i][0];
		//显示的图像数组
		for (j = 0; j < CAMERA_W; j++)
		{
			if (j < LEFT_SIDE || j > RIGHT_SIDE)
			{
				s = col_gray;
			}
			else
			{
				if (*(map) == white) s = col_white;
				if (*(map) == black) s = col_black;
				if (*(map) == blue)  s = col_blue;
				if (*(map) == green) s = col_green;
				if (*(map) == red)   s = col_red;
				if (*(map) == purple)s = col_purple;
				if (*(map) == gray) s = col_gray;
			}
			cvSet2D(pImg_2, i, j, s);
			map++;
		}
	}
	//以下为“this is picture x” 的刷新部分/////////////////////////////////
	for (i = CAMERA_H + 1; i < CAMERA_H + 21; i++)
	{
		for (j = 0; j < 100; j++)
		{
			s = col_white;
			cvSet2D(pImg_2, i, j, s);
		}
	}
	sprintf(my_text, "picture %d", pic_num);
	cvText(pImg_2, my_text, 5, CAMERA_H + 20, 0.4, 0.4, 1);
	//以上为“this is picture x” 的刷新部分//////////////////////////////////
	cvShowImage("当前图", pImg_2);
}

int midline_flag = 0;
void on_mouse(int event, int x, int y, int flag, void* param)
{
	static int start_flag = 0;
	static int restart_flag = 0;
	static int turnto_flag = 0;
	static int draw_flag = 0;
	static int rightbutton_flag = 0;
	static int fre_num = 0;

	char my_text[60];
	int i, j;
	CvPoint pt;
	CvScalar s;
	pt.x = x;
	pt.y = y;

	if (draw_flag == 1)
	{
		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
		{
			if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>65 && y < 80)
			{
				draw_flag = 0;
			}
			else    load_pic();
			break;
		}
		case CV_EVENT_RBUTTONUP:
		{
			rightbutton_flag = 0;
			break;
		}
		case CV_EVENT_RBUTTONDOWN:
		{
			rightbutton_flag = 1;
			break;
		}
		case CV_EVENT_MOUSEMOVE:
		{
			char location[15];//*********************************************************坐标显示
			for (i = CAMERA_H + 22; i < CAMERA_H + 50; i++)
			{
				for (j = 0; j < 130; j++)
				{
					s = col_white;
					cvSet2D(pImg_2, i, j, s);
				}
			}
			sprintf(location, "x = %d y= %d", x, y);
			cvText(pImg_2, location, 10, CAMERA_H + 40, 0.4, 0.4, 1);
			//***************************************************************************************
			if (x >= CAMERA_W || y >= CAMERA_H)
			{
				break;
			}
			if (rightbutton_flag == 1)
			{
				IMG[y][x] = blue;
			}
			break;
		}
		}
	}

	else if (start_flag == 0)
	{
		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
		{
			if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>5 && y < 20)
			{
				start_flag = 1;
			}
			else if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>25 && y < 40)
			{
				if (midline_flag == 0)
				{
					midline_flag = 1;
				}
				else
				{
					midline_flag = 0;
				}

			}
			else if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>45 && y < 60)
			{
				printf("Please input the num of picture you want to check:");
				scanf("%d", &pic_num);
				load_pic();
			}
			else if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>65 && y < 80)
			{
				draw_flag = 1;
			}
			else if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>85 && y < 100)
			{
				pic_num = 1;
				load_pic();
			}
			else
			{
				pic_num++;
				if (pic_num >= NUMOFEND)   pic_num = NUMOFSTART;
				load_pic();
			}
			break;
		}
		case CV_EVENT_RBUTTONDOWN:
		{
			if (pic_num <= NUMOFSTART)     pic_num = NUMOFEND;
			pic_num--;
			load_pic();
			break;
		}
		case CV_EVENT_MBUTTONDOWN:
		{
			printf("pt.x = %d,pt.y = %d\n", x, y);
			sprintf(my_text, "(%d,%d)", x, y);
			for (i = CAMERA_H + 20; i < CAMERA_H + 40; i++)
			{
				for (j = 0; j < 80; j++)
				{
					s.val[0] = 255;
					s.val[1] = 255;
					s.val[2] = 255;
					cvSet2D(pImg_2, i, j, s);
				}
			}
			cvText(pImg_2, my_text, 0, CAMERA_H + 40, 0.4, 0.4, 1);
			IMG[y][x] = 80;
			break;
		}
		}
	}

	else if (start_flag == 1)
	{
		if (event == CV_EVENT_LBUTTONDOWN)
		{
			if (x > CAMERA_W + 10 && x < CAMERA_W + 60 && y>5 && y < 20)
				start_flag = 0;
		}
		else
		{
			fre_num++;
			if (fre_num == change_speed)//********************************图片切换频率*********
			{
				pic_num++;
				if (pic_num == NUMOFEND)
				{
					pic_num = NUMOFSTART;
				}
				load_pic();
				fre_num = 1;
			}
		}

	}

	if (midline_flag == 1)
	{
		if (!have_paint_midline)
		{
			have_paint_midline = 1;
			find_midline();
		}
	}
	show_pic();
}

void cvText(IplImage* image, const char* text, int x, int y, double h, double v, int line_width)
{
	CvFont font;
	double hscale = h;
	double vscale = v;
	int linewidth = line_width;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hscale, vscale, 0, 1, linewidth);
	CvScalar textColor;
	textColor.val[0] = 255;
	textColor.val[1] = 0;
	textColor.val[2] = 0;
	CvPoint textPos = cvPoint(x, y);
	cvPutText(image, text, textPos, &font, textColor);
}

void find_midline(void)
{
	//如果点击了middle则在此处开始图像处理
	image_main();
}
int main()
{
	col_init();
	create_window();
	create_pic_now_background();
	load_pic();
	//Image_Perspect();//逆透视处理
	show_pic();

	cvSetMouseCallback("当前图", on_mouse, 0);
	pic_num = NUMOFSTART;

	while (1)
	{
		int key;
		key = cvWaitKey(0);
		if (key == 'a')
		{
			pic_num--;
			if (pic_num == NUMOFSTART - 1)
			{
				pic_num = NUMOFEND;
			}
			load_pic();
			if (midline_flag == 1)
			{
				if (!have_paint_midline)
				{
					have_paint_midline = 1;
					find_midline();
				}
			}
			show_pic();
		}
		if (key == 'd')
		{
			pic_num++;
			if (pic_num == NUMOFEND + 1)
			{
				pic_num = NUMOFSTART;
			}
			load_pic();
			if (midline_flag == 1)
			{
				if (!have_paint_midline)
				{
					have_paint_midline = 1;
					find_midline();
				}
			}
			show_pic();
		}
	}
	return 0;
}