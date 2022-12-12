#include "image.h"
int f[10 * CAMERA_H];//考察连通域联通性
//uint8_t* fullBuffer = &mt9v034_image[0][0];//改

//每个白条子属性
typedef struct {
	uint8_t   left;//左边界
	uint8_t   right;//右边界
	int   connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
	uint8_t   num;//每行白条数量
	range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
	uint8_t   left;//左边界
	uint8_t   right;//右边界
	uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
	uint8_t   white_num;
	road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t* ImgPerspect[OUT_H][OUT_W];//逆透视后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];//赛道中线
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint8_t threshold = 160;//阈值

/****************************/
uint8_t State = 0, State_Pre = 0,State_Error = 0;//状态机
uint8_t lefty[2], righty[2];//正入十字四个拐点
float param_A = 0, param_B = 0;//最小二乘直线方程参数
uint8_t RB = 0, LB = 0, RC = 0, LC = 0, RE = 0, LE = 0;//左右拐点和跳变点
float A_pre_left = 0, B_pre_left = 0, A_pre_right = 0, B_pre_right = 0;
uint8_t Flag_Stop = 0, Flag_Zebra = 0;//斑马线标志
uint8_t Flag_Curve = 0, Flag_Error = 0, Flag_Strai = 0;//弯道标志
uint8_t mid_line_pre[CAMERA_H];//上一帧中线
uint8_t image_perspect[OUT_H][OUT_W];      //逆变换图像
double map_square[CAMERA_H][CAMERA_W][2];//现实映射
int map_int[OUT_H][OUT_W][2];			//图像映射


////////////////////////////////////////////
//功能：大津法二值化
//输入：灰度图片
//输出：二值化阈值
//备注：
///////////////////////////////////////////;
uint8_t OTSU_Get() {
	unsigned short Gram[256];
	uint8_t Threshold = 0;
	unsigned short MinValue, MaxValue;
	unsigned long Amount = 0;
	unsigned long PixelBack = 0;
	unsigned long PixelshortegralBack = 0;
	unsigned long Pixelshortegral = 0;
	signed long PixelshortegralFore = 0;
	signed long PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB = -1, Sigma;
	for (int j = 0; j < 256; j++)
		Gram[j] = 0;			//初始化灰度直方图
	for (int i = 0; i < NEAR_LINE; i++) {
		for (int j = 0; j < RIGHT_SIDE; j++) {
			Gram[mt9v034_image[i][j]]++;
		}
	}
	for (MinValue = 0; MinValue < 256 && Gram[MinValue] == 0; MinValue++);
	for (MaxValue = 255; MaxValue > MinValue && Gram[MinValue] == 0; MaxValue--);
	if (MaxValue == MinValue) return MaxValue;
	if (MinValue + 1 == MaxValue) return MinValue;
	for (int j = MinValue; j <= MaxValue; j++) Amount += Gram[j];
	for (int j = MinValue; j <= MaxValue; j++) {
		Pixelshortegral += Gram[j] * j;
	}
	for (int j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + Gram[j];     //前景像素点数
		PixelFore = Amount - PixelBack;           //背景像素点数
		OmegaBack = (float)PixelBack / Amount;   //前景像素百分比
		OmegaFore = (float)PixelFore / Amount;   //背景像素百分比
		PixelshortegralBack += Gram[j] * j;  //前景灰度值
		PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
		MicroBack = (float)PixelshortegralBack / PixelBack;   //前景灰度百分比
		MicroFore = (float)PixelshortegralFore / PixelFore;   //背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
		if (Sigma > SigmaB) {
			SigmaB = Sigma;
			Threshold = (uint8_t)j;
		}
	}
	return Threshold;
}

////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
	uint8_t* map;
	uint8_t* my_map;
	map = fullBuffer;
	threshold = OTSU_Get();
	for (int i = 0; i < 30; i++)
	{
		my_map = &IMG[i][0];
		for (int j = 0; j < 188; j++)
		{
			if ((*map) > threshold + 5)
				(*my_map) = 1;
			else (*my_map) = 0;
			map++;
			my_map++;
		}
	}
	for (int i = 30; i <= NEAR_LINE; i++)
	{
		my_map = &IMG[i][0];
		for (int j = 0; j < 188; j++)
		{
			if ((*map) > threshold - 2)
				(*my_map) = 1;
			else (*my_map) = 0;
			map++;
			my_map++;
		}
	}
}

////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)//分为两部分进行清理，车头处有线挡着
{
	uint8_t* my_map;
	for (int i = 119; i >= 86; i--)//清除底部
	{
		my_map = &IMG[i][0];
		for (int j = 0; j <= 187; j++)
		{
			*(my_map + j) = black;
		}
	}
	for (int i = 85; i >= 75; i--)//清除线
	{
		my_map = &IMG[i][0];
		for (int j = 65; j <= 125; j++)
		{
			*(my_map + j) = white;
		}
	}
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号//
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
	if (f[node] == node)return node;//找到最古老祖先，return
	f[node] = find_f(f[node]);//向上寻找自己的父节点
	return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
	uint8_t i, j;
	int istart = NEAR_LINE;//处理起始行
	int iend = FAR_LINE;//处理终止行
	int tnum = 0;//当前行白条数
	all_connect_num = 0;//白条编号初始化
	uint8_t* map = NULL;
	for (i = istart; i >= iend; i--)
	{
		map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
		tnum = 0;
		for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
		{
			if ((*map))//遇白条左边界
			{
				tnum++;
				if (tnum >= white_num_MAX)break;
				range* now_white = &white_range[i].area[tnum];
				now_white->left = j;

				//开始向后一个一个像素点找这个白条右边界
				map++;
				j++;

				while ((*map) && j <= RIGHT_SIDE)
				{
					map++;
					j++;
				}
				now_white->right = j - 1;
				now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
			}
		}
		white_range[i].num = tnum;
	}
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
	//f数组初始化
	for (int i = 1; i <= all_connect_num; i++)
		f[i] = i;

	//u为up d为down 即为当前处理的这两行中的上面那行和下面那行
	//u_num：上面行白条数
	//u_left：上面行当前白条左边界
	//u_right：上面行当前白条右边界
	//i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
	int u_num, i_u, u_left, u_right;
	int d_num, i_d, d_left, d_right;
	all_range* u_white = NULL;
	all_range* d_white = NULL;
	for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
	{
		u_num = white_range[i - 1].num;
		d_num = white_range[i].num;
		u_white = &white_range[i - 1];
		d_white = &white_range[i];
		i_u = 1; i_d = 1;

		//循环到当前行或上面行白条子数耗尽为止
		while (i_u <= u_num && i_d <= d_num)
		{
			//变量先保存，避免下面访问写的冗杂且访问效率低
			u_left = u_white->area[i_u].left;
			u_right = u_white->area[i_u].right;
			d_left = d_white->area[i_d].left;
			d_right = d_white->area[i_d].right;

			if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
				f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

			//当前算法规则，手推一下你就知道为啥这样了
			if (d_right > u_right)i_u++;
			if (d_right < u_right)i_d++;
			if (d_right == u_right) { i_u++; i_d++; }
		}
	}
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
	uint8_t istart = NEAR_LINE;
	uint8_t iend = FAR_LINE;
	top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
	int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
	int while_range_num = 0, roud_while_range_num = 0;
	all_range* twhite_range = NULL;
	road* tmy_road = NULL;
	//寻找赛道所在连通域
	// 寻找最中心的白条子
	for (int i = 1; i <= white_range[istart].num; i++)
		if (white_range[istart].area[i].left <= CAMERA_W / 2
			&& white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
			road_f = find_f(white_range[istart].area[i].connect_num);

	if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
	{
		int widthmax = 0, jselect = 1;
		for (int i = 1; i <= white_range[istart].num; i++)
			if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
			{
				widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
				jselect = i;
			}
		road_f = find_f(white_range[istart].area[jselect].connect_num);
	}

	//现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
	for (int i = istart; i >= iend; i--)
	{
		//变量保存，避免之后写的冗杂且低效
		twhite_range = &white_range[i];
		tmy_road = &my_road[i];
		while_range_num = twhite_range->num;
		tmy_road->white_num = 0;
		roud_while_range_num = 0;
		for (int j = 1; j <= while_range_num; j++)
		{
			if (find_f(twhite_range->area[j].connect_num) == road_f)
			{
				top_road = i;
				tmy_road->white_num++; roud_while_range_num++;
				tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
				tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
				tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

			}
		}
	}
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
	uint8_t j_return;
	uint8_t j;
	uint8_t width_max = 0;
	uint8_t width_new = 0;
	uint8_t devia_new = 0, devia_min = 94;								//白条偏差
	uint8_t left = 0;
	uint8_t right = 0;
	uint8_t dright, dleft, uright, uleft;
	
	j_return = MISS;//如果没找到，输出255
	if (j_start > my_road[i_start].white_num)
		return MISS;
	//选一个重叠最大的
	for (j = 1; j <= my_road[i_start - 1].white_num; j++)
	{
		
		dleft = my_road[i_start].connected[j_start].left;
		dright = my_road[i_start].connected[j_start].right;
		uleft = my_road[i_start - 1].connected[j].left;
		uright = my_road[i_start - 1].connected[j].right;
		if (//相连
			dleft < uright
			&&
			dright > uleft
			)
		{
			//计算重叠大小
			if (dleft < uleft) left = uleft;
			else left = dleft;
			if (dright > uright) right = uright;
			else right = dright;
			width_new = right - left + 1;


			if (uleft > 94)  devia_new = uleft - 94;
			else if (uright < 94) devia_new = 94 - uright;
			else devia_new = 0;

			if (width_new > width_max)
			{
				if (devia_new < devia_min) {
					devia_min = devia_new;
					width_max = width_new;
					j_return = j;
				}
			}
			else if (devia_new < devia_min) {
				devia_min = devia_new;
				j_return = j;
			}
		}

	}
	return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
	uint8_t i;
	uint8_t j;
	uint8_t j_continue[CAMERA_H];//第一条连通路径
	uint8_t i_start;
	uint8_t i_end;
	uint8_t j_start = MISS;
	int width_max;

	//寻找起始行最宽的白条子
	i_start = NEAR_LINE;
	i_end = FAR_LINE;
	width_max = 0;
	for (j = 1; j <= my_road[i_start].white_num; j++)
	{
		if (my_road[i_start].connected[j].width > width_max)
		{
			width_max = my_road[i_start].connected[j].width;
			j_start = j;
		}
	}
	j_continue[i_start] = j_start;

	//记录连贯区域编号
	for (i = i_start; i > i_end; i--)
	{
		//如果相连编号大于该行白条数，非正常，从此之后都MISS
		if (j_continue[i] > my_road[i].white_num)
		{
			j_continue[i - 1] = MISS;
		}
		else
		{
			j_continue[i - 1] = find_continue(i, j_continue[i]);
		}
	}

	//全部初始化为MISS
	my_memset(left_line, MISS, CAMERA_H);
	my_memset(right_line, MISS, CAMERA_H);


	for (i = i_start; i > i_end; i--)
	{
		if (j_continue[i] <= my_road[i].white_num)
		{
			left_line[i] = my_road[i].connected[j_continue[i]].left;
			right_line[i] = my_road[i].connected[j_continue[i]].right;
		}
		else
		{
			left_line[i] = MISS;
			right_line[i] = MISS;
		}
	}
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
	uint8_t* p = ptr;
	uint8_t my_num = num;
	uint8_t Size = size;
	for (int i = 0; i < Size; i++, p++)
	{
		*p = my_num;
	}
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
	my_memset(mid_line, MISS, CAMERA_H);
	for (int i = NEAR_LINE; i >= FAR_LINE; i--)
	{
		if (left_line[i] != MISS && right_line[i] != MISS)	//直道
		{
			mid_line[i] = (left_line[i] + right_line[i]) / 2;
		}
		else
		{
			mid_line[i] = MISS;
		}
	}
	if (Flag_Error != 0) {
		if (Flag_Error == 1) {
			for (int i = NEAR_LINE; i >= FAR_LINE; i--) {
				mid_line[i] = mid_line_pre[i] - 1;
			}
		}
		else if (Flag_Error == 2) {
			for (int i = NEAR_LINE; i >= FAR_LINE; i--) {
				mid_line[i] = mid_line_pre[i] + 1;
			}
		}
		Flag_Error = 0;
	}
	else if (State_Pre == 0 && State == 4) {									//弯道和斜入十字间的切换
		for (int i = NEAR_LINE; i >= FAR_LINE; i--) {
			if (mid_line[i] != MISS || mid_line_pre[i] != MISS) {
				mid_line[i] = mid_line[i] * 0.6 + mid_line_pre[i] * 0.4;
			}
		}
	}
	else if (Flag_Curve == 1) {
		uint8_t differ = (mid_line[50] + mid_line[47] + mid_line[44] - left_line[50] - left_line[47] - left_line[44]) / 3;
		for (int i = 50; i >= 0; i--) {
			if (mid_line[i] != MISS || mid_line_pre[i] != MISS) {
				mid_line[i] = (left_line[i] + differ - (14 - 0.2 * i)) * 0.6 + mid_line_pre[i] * 0.4;
				if (mid_line[i] > RIGHT_SIDE) {
					mid_line[i] = MISS;
					break;
				}
			}
		}
	}
	else if (Flag_Curve == 2) {
		uint8_t differ = (right_line[50] + right_line[47] + right_line[44] - mid_line[50] - mid_line[47] - mid_line[44]) / 3;
		for ( int i = 50; i >= 0; i--) {
			if (mid_line[i] != MISS || mid_line_pre[i] != MISS) {
				mid_line[i] = (right_line[i] - differ + (14 - 0.2 * i)) * 0.6 + mid_line_pre[i] * 0.4;
				if (mid_line[i] < 3) {
					mid_line[i] = MISS;
					break;
				}
			}
		}
	}
	else if (State == 1 && mid_line[20] == MISS) {								//斑马线状态下丢线
		Least_Square(NEAR_LINE, NEAR_LINE - 2, NEAR_LINE - 4, NEAR_LINE - 5, 3);
		for (int i = NEAR_LINE; i > 0; i--) {
			mid_line[i] = param_A * i + param_B;
		}
	}
	for (int i = NEAR_LINE; i >= FAR_LINE; i--) {								//储存十字上一帧中线
		mid_line_pre[i] = mid_line[i];
	}
}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
	//Image_Filter();
	//THRE();
	//head_clear();
	search_white_range();
	find_all_connect();
	find_road();
	/*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
	ordinary_two_line();
	/*******赛道状态处理*******/
	State = 0;
	Flag_Curve = 0;
	Flag_Strai = 0;
	Flag_Error = 0;
	if (Zebra_Judge()) State = 1;
	else if (Cross_Judge())	State = 2;
	else if (Cross_Enter_Judge())	State = 3;
	else if (Cross_Bias_Judge()) State = 4;
	Zebra_Stop();
	switch (State) {
	case Zebra:
		printf("1 ");
		Zebra_Line();
		break;
	case CrossFront:
		printf("2 ");
		Cross_Line();
		break;
	case CrossEnter:
		printf("3 ");
		Cross_Enter_Line();
		break;
	case CrossBias:
		printf("4 ");
		Cross_Bias_Line();
		break;
	default:
		printf("0 ");
		Road_Correct();
		Curve_Judge();
		break;
	}
	get_mid_line();
	Straight_Judge();
	State_Pre = State;
	printf("\n");
	/*************************/
	for (int i = NEAR_LINE; i > 0; i--) {
		if (mid_line[i] != MISS)
		{
			IMG[i][left_line[i]] = blue;
			IMG[i][right_line[i]] = green;
			IMG[i][mid_line[i]] = red;
		}
	}
	IMG[35][mid_line[35]] = 0;
	IMG[35][mid_line[35] - 1] = 0;
	IMG[35][mid_line[35] + 1] = 0;
}

/*************************XJB改区*******************************/
//斑马线判断 *状态1
uint8_t Zebra_Judge() {
	uint8_t count_jump = 0, count_zebra = 0;
	for (int i = 60; i >= 25; i--) {
		if (my_road[i].white_num > 4) count_zebra++;
		if (count_zebra > 6) return 1;
		}
	return 0;
}

//斑马线补线
void Zebra_Line() {
	Least_Square(30, 40, 50, 60, 5);
	for (int i = 50; i > 0; i--) {
		right_line[i] = param_A * i + param_B;
	}
	Least_Square(30, 40, 50, 60, 4);
	for (int i = 50; i > 0; i--) {
		left_line[i] = param_A * i + param_B;
	}
}

//斑马线判断停车
void Zebra_Stop() {
	if (State == 1) {
		Flag_Zebra = 1;
	}
	else if (State != 1 && Flag_Zebra == 1) {
		Flag_Zebra = 0;
		Flag_Stop++;
	}
}

//正入十字判断 *状态2
uint8_t Cross_Judge() {
	uint8_t counter = 0, upline = 0, downline = 0;
	uint8_t left_down_flag = 0, left_up_flag = 0, right_down_flag = 0, right_up_flag = 0;
	for (int i = NEAR_LINE; i > 1; i--) {															//左下跳变点
		if (abs(left_line[i] - left_line[i - 1]) < 5 && abs(left_line[i - 1] - left_line[i - 2]) < 5 &&
			left_line[i - 3] - left_line[i - 2] < -3 && i - 2 < 70 && left_line[i - 2] > 30) {
			lefty[0] = (uint8_t)i - 2;
			left_down_flag = 1;
			break;
		}
	}
	for (int i = NEAR_LINE; i > 1; i--) {															//右下跳变点
		if (abs(right_line[i] - right_line[i - 1]) < 5 && abs(right_line[i - 1] - right_line[i - 2]) < 5 &&
			right_line[i - 3] - right_line[i - 2] > 3 && i - 2 < 70 && right_line[i - 2] < RIGHT_SIDE - 30) {
			righty[0] = (uint8_t)i - 2;
			right_down_flag = 1;
			break;
		}
	}
	for (int i = 0; i < NEAR_LINE; i++) {															//左上跳变点
		if (abs(left_line[i] - left_line[i + 1]) < 5 && abs(left_line[i + 1] - left_line[i + 2]) < 5 &&
			left_line[i + 2] - left_line[i + 3] > 3 && left_line[i + 2] < 100 && i + 2 > 10 && left_line[i + 2] > 30) {
			lefty[1] = (uint8_t)i + 2;
			left_up_flag = 1;
			break;
		}
	}
	for (int i = 0; i < NEAR_LINE; i++) {															//右上跳变点
		if (abs(right_line[i] - right_line[i + 1]) < 5 && abs(right_line[i + 1] - right_line[i + 2]) < 5 &&
			right_line[i + 2] - right_line[i + 3] < -3 && right_line[i + 2] > 80 && i + 2 > 10 && right_line[i + 2] < RIGHT_SIDE - 30) {
			righty[1] = (uint8_t)i + 2;
			right_up_flag = 1;
			break;
		}
	}
	if (left_down_flag && left_up_flag && right_down_flag && right_up_flag && righty[0] - righty[1] > 10 && lefty[0] - lefty[1] > 10) {
		downline = lefty[0] > righty[0] ? righty[0] : lefty[0];
		upline = lefty[1] > righty[1] ? lefty[1] : righty[1];
		for (int i = downline; i > upline; i--) {
			for (int j = 0; j < RIGHT_SIDE; j++) {
				if (IMG[i][j] == 1) counter++;
				if (counter > 5 * (downline - upline)) {
					return 1;
				}
			}
		}
	}
	return 0;
}

// 入十字前补线
void Cross_Line() {
	if (State_Pre != 2 && State_Pre != 3 && State_Pre != 4) {
		Least_Square(lefty[0] + 2, lefty[0], lefty[1], lefty[1] - 2, 1);
		for (int i = lefty[0]; i > lefty[1]; i--) {
			left_line[i] = param_A * i + param_B;
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
		Least_Square(righty[0] + 2, righty[0], righty[1], righty[1] - 2, 2);
		for (int i = righty[0]; i > righty[1]; i--) {
			right_line[i] = param_A * i + param_B;
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
	}
	else {
		Least_Square(lefty[0] + 2, lefty[0], lefty[1], lefty[1] - 2, 1);
		for (int i = lefty[0]; i > lefty[1]; i--) {
			left_line[i] = (param_A * 0.65 + A_pre_left * 0.35) * i + (param_B * 0.65 + B_pre_left * 0.35);
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
		Least_Square(righty[0] + 2, righty[0], righty[1], righty[1] - 2, 2);
		for (int i = righty[0]; i > righty[1]; i--) {
			right_line[i] = (param_A * 0.65 + A_pre_right * 0.35) * i + (param_B * 0.65 + B_pre_right * 0.35);
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
	}
	param_A = 0;
	param_B = 0;
}

//进入十字判断  *状态3
uint8_t Cross_Enter_Judge() {
	uint8_t count_blank = 0, flag_blank = 0, flag_LE = 0, flag_RE = 0;
	for (int i = NEAR_LINE; i > 0; i--) {														//检测左右边缘线
		if (right_line[i] - left_line[i] > 160 && right_line[i] - left_line[i] < 200) {
			count_blank++;
		}
		else count_blank = 0;
		if (count_blank > 15) {
			flag_blank = 1;
			break;
		}
	}
	if (flag_blank) {
		for (int i = 1; i < NEAR_LINE - 20; i++) {
			if (abs(right_line[i + 1] - right_line[i]) < 5 && abs(right_line[i + 2] - right_line[i + 1]) < 5 && right_line[i + 3] - right_line[i + 2] > 3
				&& right_line[i + 2] > 80 && i + 2 > 15 && right_line[i + 2] < RIGHT_SIDE - 30 && IMG[i + 10][right_line[i + 2] + 2] == 1) {							//右上跳变点
				RE = (uint8_t)i + 2;
				flag_RE = 1;
				printf("RE=%d ", RE);
				break;
			}
		}
		for (int i = 1; i < NEAR_LINE - 20; i++) {
			if (abs(left_line[i + 1] - left_line[i]) < 5 && abs(left_line[i + 2] - left_line[i + 1]) < 5 && left_line[i + 3] - left_line[i + 2] < -3
				&& left_line[i + 2] < 100 && i + 2 > 15 && left_line[i + 2] > 30 && IMG[i + 10][left_line[i + 2] - 2] == 1) {							//左上跳变点
				LE = (uint8_t)i + 2;
				flag_LE = 1;
				printf("LE=%d ", LE);
				break;
			}
		}
	}
	if (flag_LE && flag_RE) return 1;
	else return 0;
}

//进入十字补线
void Cross_Enter_Line() {
	if (State_Pre != 2 && State_Pre != 3 && State_Pre != 4) {
		Least_Square(RE, RE - 1, RE - 2, RE - 4, 2);
		for (int i = RE; i < NEAR_LINE; i++) {
			right_line[i] = param_A * i + param_B;
			if (right_line[i - 1] == RIGHT_SIDE - 1 || right_line[i] > RIGHT_SIDE - 1) {
				right_line[i] = RIGHT_SIDE - 1;
			}
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
		Least_Square(LE, LE - 1, LE - 2, LE - 4, 1);
		for (int i = LE; i < NEAR_LINE; i++) {
			left_line[i] = param_A * i + param_B;
			if (abs(left_line[i] - left_line[i - 1]) > 50 || left_line[i - 1] == 0) {
				left_line[i] = 0;
			}
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
	}
	else {
		Least_Square(RE, RE - 1, RE - 2, RE - 4, 2);
		for (int i = RE; i < NEAR_LINE; i++) {
			right_line[i] = (param_A * 0.65 + A_pre_right * 0.35) * i + (param_B * 0.65 + B_pre_right * 0.35);
			if (right_line[i - 1] == RIGHT_SIDE - 1 || right_line[i] > RIGHT_SIDE - 1) {
				right_line[i] = RIGHT_SIDE - 1;
			}
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
		Least_Square(LE, LE - 1, LE - 2, LE - 4, 1);
		for (int i = LE; i < NEAR_LINE; i++) {
			left_line[i] = (param_A * 0.65 + A_pre_left * 0.35) * i + (param_B * 0.65 + B_pre_left * 0.35);
			if (abs(left_line[i] - left_line[i - 1]) > 50 || left_line[i - 1] == 0) {
				left_line[i] = 0;
			}
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
	}
	param_A = 0;
	param_B = 0;
}

//斜出入十字 *状态4
uint8_t Cross_Bias_Judge() {
	uint8_t RB_flag = 0, LB_flag = 0, count_left = 0, count_right = 0;
	uint8_t flag_left = 1, flag_right = 1;
	uint8_t mid_mark = 0;
	int temp = 0;
	for (int i = 5; i < 15; i += 2) {
		temp += (left_line[NEAR_LINE - i] + right_line[NEAR_LINE - i]) / 2;
		mid_mark = temp / 5;
	}
	for (int i = NEAR_LINE - 1; i > 0; i--) {
		if (left_line[i] > mid_mark && left_line[i] < 200) count_left++;
		if (right_line[i] < mid_mark)  count_right++;
		if (count_left > 6) {
			flag_left = 0;
			break;
		}
		if (count_right > 6) {
			flag_right = 0;
			break;
		}
	}
	if (!flag_left || !flag_right) {
		for (int i = NEAR_LINE / 2; i > 5; i--) {
			if (!flag_left && abs(left_line[i] - left_line[i + 1]) > 30 && abs(left_line[i] - left_line[i + 1]) < 200) {
				flag_left = 1;
				break;
			}
			if (!flag_right && abs(right_line[i] - right_line[i + 1]) > 30 && abs(right_line[i] - right_line[i + 1]) < 200) {
				flag_right = 1;
				break;
			}
		}
	}
	if (flag_left && flag_right) {
		for (int i = NEAR_LINE - 5; i > 20; i--) {
			if (left_line[NEAR_LINE - 2] - left_line[i - 2] < 0 && left_line[i] - left_line[i - 1] <= 0 && left_line[i - 1] - left_line[i - 2] <= 0 && left_line[i - 2] - left_line[i - 3] > 0
				&& left_line[i - 3] - left_line[i - 5] >= 0 && left_line[i - 2] - left_line[i - 3] < 50 && i - 2 < 80 && IMG[i - 10][left_line[i - 2] - 2] == 1) {
				LB = (uint8_t)i - 2;														//左拐点
				LB_flag = 1;
				printf("LB=%d ", LB);
				break;
			}
		}
		for (int i = NEAR_LINE - 5; i > 20; i--) {
			if (right_line[NEAR_LINE - 2] - right_line[i - 2] > 0 && right_line[i] - right_line[i - 1] >= 0 && right_line[i - 1] - right_line[i - 2] >= 0 && right_line[i - 2] - right_line[i - 3] < 0
				&& right_line[i - 3] - right_line[i - 5] <= 0 && right_line[i - 2] - right_line[i - 3] > -50 && i - 2 < 80 && IMG[i - 10][right_line[i - 2] + 2] == 1) {
				RB = (uint8_t)i - 2;														//右拐点
				RB_flag = 1;
				printf("RB=%d ", RB);
				break;
			}
		}
	}
	if (LB_flag && RB_flag && right_line[RB] - left_line[LB] > 20) return 1;
	else return 0;
}

//斜出入十字补线
void Cross_Bias_Line() {
	if (State_Pre != 2 && State_Pre != 3 && State_Pre != 4) {
		Least_Square(RB, RB + 3, RB + 6, RB + 9, 2);
		for (int i = RB; i > 0; i--) {
			right_line[i] = param_A * i + param_B;
			if (right_line[i] >= RIGHT_SIDE - 1 || right_line[i + 1] == RIGHT_SIDE - 1) {
				right_line[i] = RIGHT_SIDE - 1;
			}
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
		Least_Square(LB, LB + 3, LB + 6, LB + 9, 1);
		for (int i = LB; i > 0; i--) {
			left_line[i] = param_A * i + param_B;
			if (abs(left_line[i] - left_line[i + 1]) > 50 || left_line[i + 1] == 0) {
				left_line[i] = 0;
			}
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
	}
	else {
		Least_Square(RB, RB + 3, RB + 6, RB + 9, 2);
		for (int i = RB; i > 0; i--) {
			right_line[i] = (param_A * 0.65 + A_pre_right * 0.35) * i + (param_B * 0.65 + B_pre_right * 0.35);
			if (right_line[i] >= RIGHT_SIDE - 1 || right_line[i + 1] == RIGHT_SIDE - 1) {
				right_line[i] = RIGHT_SIDE - 1;
			}
		}
		A_pre_right = param_A;
		B_pre_right = param_B;
		Least_Square(LB, LB + 3, LB + 6, LB + 9, 1);
		for (int i = LB; i > 0; i--) {
			left_line[i] = (param_A * 0.65 + A_pre_left * 0.35) * i + (param_B * 0.65 + B_pre_left * 0.35);
			if (abs(left_line[i] - left_line[i + 1]) > 50 || left_line[i + 1] == 0) {
				left_line[i] = 0;
			}
		}
		A_pre_left = param_A;
		B_pre_left = param_B;
	}
	param_A = 0;
	param_B = 0;
}

//弯道识别
void Curve_Judge() {
	uint8_t count_left = 0, count_right = 0;
	if (top_road > 8) {
		for (int i = NEAR_LINE - 20; i > 10; i--) {
			if (left_line[i] < 5 && left_line[NEAR_LINE - 1] > 3) count_left++;
			else count_left = 0;
			if (right_line[i] > RIGHT_SIDE - 5 && right_line[i] < RIGHT_SIDE + 1 && right_line[NEAR_LINE - 1] < RIGHT_SIDE - 3) count_right++;
			else count_right = 0;
			if (count_left > 5) {
				Flag_Curve = 2;
				break;
			}
			if (count_right > 5) {
				Flag_Curve = 1;
				break;
			}
		}
	}
	/*for (int i = NEAR_LINE / 2; i > 5; i--) {
		if (abs(left_line[i] - left_line[i + 1]) > 30 && left_line[i] != MISS) {
			Flag_Curve = 0;
			break;
		}
		if (abs(right_line[i] - right_line[i + 1]) > 30 && right_line[i] != MISS) {
			Flag_Curve = 0;
			break;
		}
	}*/
}

//直道识别
void Straight_Judge() {
	uint8_t mid_mark = 0;
	int error = 0;
	int temp = 0;
	for (int i = 6; i < 15; i += 2) {
		temp += (left_line[NEAR_LINE - i] + right_line[NEAR_LINE - i]) / 2;
		mid_mark = temp / 5;
	}
	if (Flag_Curve == 0) {
		for (int i = NEAR_LINE / 2; i > 15; i--) {
			error += pow(abs(mid_mark - mid_line[i]), 2);
		}
		if (error < 120) Flag_Strai = 1;
		else Flag_Strai = 0;
	}
}

//漫水误判纠正
void Road_Correct() {
	int count = 0;
	uint8_t flag = 0;
	if (State_Pre == 2 || State_Pre == 3 || State_Pre == 4 || State_Error == 1) {
		for (int i = NEAR_LINE / 2; i > 0; i--) {
			if (right_line[i] - left_line[i] < 20 && right_line[i] - left_line[i] > 0) count++;
			if (count > 6) {
				flag = 1;
				break;
			}
		}
	}
	if (flag == 1) {
		for (int i = NEAR_LINE / 2; i > 0; i--) {
			if (left_line[i] - left_line[i + 1] > 30 && left_line[i] - left_line[i + 1] < 150) {
				Flag_Error = 1;
				State_Error = 1;
				break;
			}
			else if (right_line[i + 1] - right_line[i] > 30 && right_line[i + 1] - right_line[i] < 150) {
				Flag_Error = 2;
				State_Error = 1;
				break;
			}
		}
	}
	if (Flag_Error == 0) State_Error = 0;
}

//最小二乘法
void Least_Square(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4, int type) {
	uint8_t y1 = 0, y2 = 0, y3 = 0, y4 = 0;
	if (type == 1) {				//左边线拟合
		y1 = left_line[x1];
		y2 = left_line[x2];
		y3 = left_line[x3];
		y4 = left_line[x4];
	}
	else if (type == 2) {			//右边线拟合
		y1 = right_line[x1];
		y2 = right_line[x2];
		y3 = right_line[x3];
		y4 = right_line[x4];
	}
	else if (type == 3) {			//中线拟合
		y1 = mid_line[x1];
		y2 = mid_line[x2];
		y3 = mid_line[x3];
		y4 = mid_line[x4];
	}
	else if (type == 4) {
		y1 = my_road[x1].connected[1].left;
		y2 = my_road[x2].connected[1].left;
		y3 = my_road[x3].connected[1].left;
		y4 = my_road[x4].connected[1].left;
	}
	else if (type == 5) {
		y1 = my_road[x1].connected[my_road[x1].white_num].right;
		y2 = my_road[x2].connected[my_road[x2].white_num].right;
		y3 = my_road[x3].connected[my_road[x3].white_num].right;
		y4 = my_road[x4].connected[my_road[x4].white_num].right;
	}
	float sum_x2, sum_x, sum_y, sum_xy;
	sum_x2 = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
	sum_x = x1 + x2 + x3 + x4;
	sum_y = y1 + y2 + y3 + y4;
	sum_xy = x1 * y1 + x2 * y2 + x3 * y3 + x4 * y4;
	param_A = (4 * sum_xy - sum_x * sum_y) / (4 * sum_x2 - sum_x * sum_x);
	param_B = (sum_x2 * sum_y - sum_x * sum_xy) / (4 * sum_x2 - sum_x * sum_x);
}

//停车保护
uint8_t Park_Protect() {
	int count = 0;
	for (int i = 60; i > 30; i--) {
		for (int j = LEFT_SIDE; j < RIGHT_SIDE; j++) {
			if (IMG[i][j] == 0) count++;
			if (count > (60 - 30) * 170) return 1;
		}
	}
	return 0;
}

//图像降噪
void Image_Filter() {
	for (int i = 1; i < NEAR_LINE - 1; i++) {
		for (int j = 1; j < RIGHT_SIDE - 1; j++) {
			if (IMG[i][j] == black && (IMG[i - 1][j] + IMG[i + 1][j] + IMG[i][j - 1] + IMG[i][j + 1] > 2)) {
				IMG[i][j] = white;
			}
			if (IMG[i][j] == white && (IMG[i - 1][j] + IMG[i + 1][j] + IMG[i][j - 1] + IMG[i][j + 1] < 2)) {
				IMG[i][j] = black;
			}
		}
	}
}

//逆透视变换
void Image_Perspect() {
	static uint8_t Black = 0;
	double change_un_Mat[3][3] = { {0.413302,-0.401727,7.131702},{0.010244,0.049443,7.229940},{0.000126,-0.004416,0.492404} };
	for (int i = 0; i < OUT_W; i++) {
		for (int j = 0; j < OUT_H; j++) {
			int local_x = (int)((change_un_Mat[0][0] * i
				+ change_un_Mat[0][1] * j + change_un_Mat[0][2])
				/ (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
				+ change_un_Mat[2][2]));
			int local_y = (int)((change_un_Mat[1][0] * i
				+ change_un_Mat[1][1] * j + change_un_Mat[1][2])
				/ (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
				+ change_un_Mat[2][2]));
			if (local_x >= 0 && local_y >= 0 && local_y < CAMERA_H && local_x < CAMERA_W) {
				ImgPerspect[j][i] = &IMG[local_y][local_x];
			}
			else {
				ImgPerspect[j][i] = &Black;
			}
		}
	}
}
