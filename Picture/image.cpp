#include "image.h"
int f[10 * CAMERA_H];//������ͨ����ͨ��
//uint8_t* fullBuffer = &mt9v034_image[0][0];//��

//ÿ������������
typedef struct {
	uint8_t   left;//��߽�
	uint8_t   right;//�ұ߽�
	int   connect_num;//��ͨ���
}range;

//ÿ�е����а�����
typedef struct {
	uint8_t   num;//ÿ�а�������
	range   area[white_num_MAX];//���и���������
}all_range;

//����������ÿ������������
typedef struct {
	uint8_t   left;//��߽�
	uint8_t   right;//�ұ߽�
	uint8_t   width;//���
}road_range;

//ÿ������������ÿ��������
typedef struct {
	uint8_t   white_num;
	road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//���а�����
road my_road[CAMERA_H];//����
uint8_t IMG[CAMERA_H][CAMERA_W];//��ֵ����ͼ������
uint8_t* ImgPerspect[OUT_H][OUT_W];//��͸�Ӻ�ͼ������
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//���������ұ߽�
uint8_t mid_line[CAMERA_H];//��������
int all_connect_num = 0;//���а�������
uint8_t top_road;//������ߴ���������
uint8_t threshold = 160;//��ֵ

/****************************/
uint8_t State = 0, State_Pre = 0,State_Error = 0;//״̬��
uint8_t lefty[2], righty[2];//����ʮ���ĸ��յ�
float param_A = 0, param_B = 0;//��С����ֱ�߷��̲���
uint8_t RB = 0, LB = 0, RC = 0, LC = 0, RE = 0, LE = 0;//���ҹյ�������
float A_pre_left = 0, B_pre_left = 0, A_pre_right = 0, B_pre_right = 0;
uint8_t Flag_Stop = 0, Flag_Zebra = 0;//�����߱�־
uint8_t Flag_Curve = 0, Flag_Error = 0, Flag_Strai = 0;//�����־
uint8_t mid_line_pre[CAMERA_H];//��һ֡����
uint8_t image_perspect[OUT_H][OUT_W];      //��任ͼ��
double map_square[CAMERA_H][CAMERA_W][2];//��ʵӳ��
int map_int[OUT_H][OUT_W][2];			//ͼ��ӳ��


////////////////////////////////////////////
//���ܣ���򷨶�ֵ��
//���룺�Ҷ�ͼƬ
//�������ֵ����ֵ
//��ע��
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
		Gram[j] = 0;			//��ʼ���Ҷ�ֱ��ͼ
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
		PixelBack = PixelBack + Gram[j];     //ǰ�����ص���
		PixelFore = Amount - PixelBack;           //�������ص���
		OmegaBack = (float)PixelBack / Amount;   //ǰ�����ذٷֱ�
		OmegaFore = (float)PixelFore / Amount;   //�������ذٷֱ�
		PixelshortegralBack += Gram[j] * j;  //ǰ���Ҷ�ֵ
		PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
		MicroBack = (float)PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
		MicroFore = (float)PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
		if (Sigma > SigmaB) {
			SigmaB = Sigma;
			Threshold = (uint8_t)j;
		}
	}
	return Threshold;
}

////////////////////////////////////////////
//���ܣ���ֵ��
//���룺�Ҷ�ͼƬ
//�������ֵ��ͼƬ
//��ע��
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
//���ܣ�������峵ͷ
//���룺
//�����
//��ע��Ҫ�����Լ���ͷ�Ĵ�С�����޸�
///////////////////////////////////////////
void head_clear(void)//��Ϊ�����ֽ���������ͷ�����ߵ���
{
	uint8_t* my_map;
	for (int i = 119; i >= 86; i--)//����ײ�
	{
		my_map = &IMG[i][0];
		for (int j = 0; j <= 187; j++)
		{
			*(my_map + j) = black;
		}
	}
	for (int i = 85; i >= 75; i--)//�����
	{
		my_map = &IMG[i][0];
		for (int j = 65; j <= 125; j++)
		{
			*(my_map + j) = white;
		}
	}
}

////////////////////////////////////////////
//���ܣ����Ҹ��ڵ�
//���룺�ڵ���//
//�������������
//��ע����·��ѹ��
///////////////////////////////////////////
int find_f(int node)
{
	if (f[node] == node)return node;//�ҵ���������ȣ�return
	f[node] = find_f(f[node]);//����Ѱ���Լ��ĸ��ڵ�
	return f[node];
}

////////////////////////////////////////////
//���ܣ���ȡ������ ����ȫ�������ӱ��
//���룺IMG[120][188]
//�����white_range[120]
//��ע��ָ������
///////////////////////////////////////////
void search_white_range()
{
	uint8_t i, j;
	int istart = NEAR_LINE;//������ʼ��
	int iend = FAR_LINE;//������ֹ��
	int tnum = 0;//��ǰ�а�����
	all_connect_num = 0;//������ų�ʼ��
	uint8_t* map = NULL;
	for (i = istart; i >= iend; i--)
	{
		map = &IMG[i][LEFT_SIDE];//ָ�����߼ӿ�����ٶ�
		tnum = 0;
		for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
		{
			if ((*map))//��������߽�
			{
				tnum++;
				if (tnum >= white_num_MAX)break;
				range* now_white = &white_range[i].area[tnum];
				now_white->left = j;

				//��ʼ���һ��һ�����ص�����������ұ߽�
				map++;
				j++;

				while ((*map) && j <= RIGHT_SIDE)
				{
					map++;
					j++;
				}
				now_white->right = j - 1;
				now_white->connect_num = ++all_connect_num;//��������һ��������������
			}
		}
		white_range[i].num = tnum;
	}
}

////////////////////////////////////////////
//���ܣ�Ѱ�Ұ�������ͨ�ԣ���ȫ����ͨ�����ӵĽڵ���ˢ����������ȵĽڵ���
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_all_connect()
{
	//f�����ʼ��
	for (int i = 1; i <= all_connect_num; i++)
		f[i] = i;

	//uΪup dΪdown ��Ϊ��ǰ������������е��������к���������
	//u_num�������а�����
	//u_left�������е�ǰ������߽�
	//u_right�������е�ǰ�����ұ߽�
	//i_u����ǰ�������������ǵ�ǰ���У������У������еĵ�i_u��
	int u_num, i_u, u_left, u_right;
	int d_num, i_d, d_left, d_right;
	all_range* u_white = NULL;
	all_range* d_white = NULL;
	for (int i = NEAR_LINE; i > FAR_LINE; i--)//��Ϊÿ����ÿ���бȽ� ����ѭ����FAR_LINE+1
	{
		u_num = white_range[i - 1].num;
		d_num = white_range[i].num;
		u_white = &white_range[i - 1];
		d_white = &white_range[i];
		i_u = 1; i_d = 1;

		//ѭ������ǰ�л������а��������ľ�Ϊֹ
		while (i_u <= u_num && i_d <= d_num)
		{
			//�����ȱ��棬�����������д�������ҷ���Ч�ʵ�
			u_left = u_white->area[i_u].left;
			u_right = u_white->area[i_u].right;
			d_left = d_white->area[i_d].left;
			d_right = d_white->area[i_d].right;

			if (u_left <= d_right && u_right >= d_left)//�������������ͨ
				f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//���ڵ�������

			//��ǰ�㷨��������һ�����֪��Ϊɶ������
			if (d_right > u_right)i_u++;
			if (d_right < u_right)i_d++;
			if (d_right == u_right) { i_u++; i_d++; }
		}
	}
}

////////////////////////////////////////////
//���ܣ�Ѱ������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void find_road()
{
	uint8_t istart = NEAR_LINE;
	uint8_t iend = FAR_LINE;
	top_road = NEAR_LINE;//������ߴ������������ȳ�ʼ����Ϊ��ʹ�
	int road_f = -1;//����������ͨ�򸸽ڵ��ţ��ȳ�ʼ��Ϊ-1�����ж��Ƿ��ҵ�����
	int while_range_num = 0, roud_while_range_num = 0;
	all_range* twhite_range = NULL;
	road* tmy_road = NULL;
	//Ѱ������������ͨ��
	// Ѱ�������ĵİ�����
	for (int i = 1; i <= white_range[istart].num; i++)
		if (white_range[istart].area[i].left <= CAMERA_W / 2
			&& white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
			road_f = find_f(white_range[istart].area[i].connect_num);

	if (road_f == -1)//������û���м䣬��113��ѡһ�������Ϊ���������
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

	//���������Ѿ��õ�������������ͨ�򸸽ڵ��ţ������������и��ڵ�����road_f�����а������ӽ��������������
	for (int i = istart; i >= iend; i--)
	{
		//�������棬����֮��д�������ҵ�Ч
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
//���ܣ�����������һ�а����ӱ��
//���룺i_start��ʼ��  j_start�������
//������������
//��ע����Ϊ��һ���뱾�������ص����ֶԶ�İ���Ϊѡ������
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
	uint8_t j_return;
	uint8_t j;
	uint8_t width_max = 0;
	uint8_t width_new = 0;
	uint8_t devia_new = 0, devia_min = 94;								//����ƫ��
	uint8_t left = 0;
	uint8_t right = 0;
	uint8_t dright, dleft, uright, uleft;
	
	j_return = MISS;//���û�ҵ������255
	if (j_start > my_road[i_start].white_num)
		return MISS;
	//ѡһ���ص�����
	for (j = 1; j <= my_road[i_start - 1].white_num; j++)
	{
		
		dleft = my_road[i_start].connected[j_start].left;
		dright = my_road[i_start].connected[j_start].right;
		uleft = my_road[i_start - 1].connected[j].left;
		uright = my_road[i_start - 1].connected[j].right;
		if (//����
			dleft < uright
			&&
			dright > uleft
			)
		{
			//�����ص���С
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
//���ܣ�ͨ�þ���˫��
//���룺
//�����
//��ע��
///////////////////////////////////////////
void ordinary_two_line(void)
{
	uint8_t i;
	uint8_t j;
	uint8_t j_continue[CAMERA_H];//��һ����ͨ·��
	uint8_t i_start;
	uint8_t i_end;
	uint8_t j_start = MISS;
	int width_max;

	//Ѱ����ʼ�����İ�����
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

	//��¼����������
	for (i = i_start; i > i_end; i--)
	{
		//���������Ŵ��ڸ��а����������������Ӵ�֮��MISS
		if (j_continue[i] > my_road[i].white_num)
		{
			j_continue[i - 1] = MISS;
		}
		else
		{
			j_continue[i - 1] = find_continue(i, j_continue[i]);
		}
	}

	//ȫ����ʼ��ΪMISS
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
//���ܣ������ʼ��
//���룺uint8_t* ptr �����׵�ַ, uint8_t num��ʼ����ֵ, uint8_t size�����С
//�����
//��ע����Ϊk66������Ϊmemset��������ȫ�������޷�ʹ�ã������Ҫ�Լ�дһ��my_memset
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
//���ܣ����ߺϳ�
//���룺���ұ߽�
//���������
//��ע��
///////////////////////////////////////////
void get_mid_line(void)
{
	my_memset(mid_line, MISS, CAMERA_H);
	for (int i = NEAR_LINE; i >= FAR_LINE; i--)
	{
		if (left_line[i] != MISS && right_line[i] != MISS)	//ֱ��
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
	else if (State_Pre == 0 && State == 4) {									//�����б��ʮ�ּ���л�
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
	else if (State == 1 && mid_line[20] == MISS) {								//������״̬�¶���
		Least_Square(NEAR_LINE, NEAR_LINE - 2, NEAR_LINE - 4, NEAR_LINE - 5, 3);
		for (int i = NEAR_LINE; i > 0; i--) {
			mid_line[i] = param_A * i + param_B;
		}
	}
	for (int i = NEAR_LINE; i >= FAR_LINE; i--) {								//����ʮ����һ֡����
		mid_line_pre[i] = mid_line[i];
	}
}
////////////////////////////////////////////
//���ܣ�ͼ����������
//���룺
//�����
//��ע��
///////////////////////////////////////////
void image_main()
{
	//Image_Filter();
	//THRE();
	//head_clear();
	search_white_range();
	find_all_connect();
	find_road();
	/*���˴�Ϊֹ�������Ѿ��õ������������Ľṹ������my_road[CAMERA_H]*/
	ordinary_two_line();
	/*******����״̬����*******/
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

/*************************XJB����*******************************/
//�������ж� *״̬1
uint8_t Zebra_Judge() {
	uint8_t count_jump = 0, count_zebra = 0;
	for (int i = 60; i >= 25; i--) {
		if (my_road[i].white_num > 4) count_zebra++;
		if (count_zebra > 6) return 1;
		}
	return 0;
}

//�����߲���
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

//�������ж�ͣ��
void Zebra_Stop() {
	if (State == 1) {
		Flag_Zebra = 1;
	}
	else if (State != 1 && Flag_Zebra == 1) {
		Flag_Zebra = 0;
		Flag_Stop++;
	}
}

//����ʮ���ж� *״̬2
uint8_t Cross_Judge() {
	uint8_t counter = 0, upline = 0, downline = 0;
	uint8_t left_down_flag = 0, left_up_flag = 0, right_down_flag = 0, right_up_flag = 0;
	for (int i = NEAR_LINE; i > 1; i--) {															//���������
		if (abs(left_line[i] - left_line[i - 1]) < 5 && abs(left_line[i - 1] - left_line[i - 2]) < 5 &&
			left_line[i - 3] - left_line[i - 2] < -3 && i - 2 < 70 && left_line[i - 2] > 30) {
			lefty[0] = (uint8_t)i - 2;
			left_down_flag = 1;
			break;
		}
	}
	for (int i = NEAR_LINE; i > 1; i--) {															//���������
		if (abs(right_line[i] - right_line[i - 1]) < 5 && abs(right_line[i - 1] - right_line[i - 2]) < 5 &&
			right_line[i - 3] - right_line[i - 2] > 3 && i - 2 < 70 && right_line[i - 2] < RIGHT_SIDE - 30) {
			righty[0] = (uint8_t)i - 2;
			right_down_flag = 1;
			break;
		}
	}
	for (int i = 0; i < NEAR_LINE; i++) {															//���������
		if (abs(left_line[i] - left_line[i + 1]) < 5 && abs(left_line[i + 1] - left_line[i + 2]) < 5 &&
			left_line[i + 2] - left_line[i + 3] > 3 && left_line[i + 2] < 100 && i + 2 > 10 && left_line[i + 2] > 30) {
			lefty[1] = (uint8_t)i + 2;
			left_up_flag = 1;
			break;
		}
	}
	for (int i = 0; i < NEAR_LINE; i++) {															//���������
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

// ��ʮ��ǰ����
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

//����ʮ���ж�  *״̬3
uint8_t Cross_Enter_Judge() {
	uint8_t count_blank = 0, flag_blank = 0, flag_LE = 0, flag_RE = 0;
	for (int i = NEAR_LINE; i > 0; i--) {														//������ұ�Ե��
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
				&& right_line[i + 2] > 80 && i + 2 > 15 && right_line[i + 2] < RIGHT_SIDE - 30 && IMG[i + 10][right_line[i + 2] + 2] == 1) {							//���������
				RE = (uint8_t)i + 2;
				flag_RE = 1;
				printf("RE=%d ", RE);
				break;
			}
		}
		for (int i = 1; i < NEAR_LINE - 20; i++) {
			if (abs(left_line[i + 1] - left_line[i]) < 5 && abs(left_line[i + 2] - left_line[i + 1]) < 5 && left_line[i + 3] - left_line[i + 2] < -3
				&& left_line[i + 2] < 100 && i + 2 > 15 && left_line[i + 2] > 30 && IMG[i + 10][left_line[i + 2] - 2] == 1) {							//���������
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

//����ʮ�ֲ���
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

//б����ʮ�� *״̬4
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
				LB = (uint8_t)i - 2;														//��յ�
				LB_flag = 1;
				printf("LB=%d ", LB);
				break;
			}
		}
		for (int i = NEAR_LINE - 5; i > 20; i--) {
			if (right_line[NEAR_LINE - 2] - right_line[i - 2] > 0 && right_line[i] - right_line[i - 1] >= 0 && right_line[i - 1] - right_line[i - 2] >= 0 && right_line[i - 2] - right_line[i - 3] < 0
				&& right_line[i - 3] - right_line[i - 5] <= 0 && right_line[i - 2] - right_line[i - 3] > -50 && i - 2 < 80 && IMG[i - 10][right_line[i - 2] + 2] == 1) {
				RB = (uint8_t)i - 2;														//�ҹյ�
				RB_flag = 1;
				printf("RB=%d ", RB);
				break;
			}
		}
	}
	if (LB_flag && RB_flag && right_line[RB] - left_line[LB] > 20) return 1;
	else return 0;
}

//б����ʮ�ֲ���
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

//���ʶ��
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

//ֱ��ʶ��
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

//��ˮ���о���
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

//��С���˷�
void Least_Square(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4, int type) {
	uint8_t y1 = 0, y2 = 0, y3 = 0, y4 = 0;
	if (type == 1) {				//��������
		y1 = left_line[x1];
		y2 = left_line[x2];
		y3 = left_line[x3];
		y4 = left_line[x4];
	}
	else if (type == 2) {			//�ұ������
		y1 = right_line[x1];
		y2 = right_line[x2];
		y3 = right_line[x3];
		y4 = right_line[x4];
	}
	else if (type == 3) {			//�������
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

//ͣ������
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

//ͼ����
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

//��͸�ӱ任
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
