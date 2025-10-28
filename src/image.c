//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "morph_binary_bitpacked.h"
#include "global_image_buffer.h"
#include "dynamic_log.h"
#include "kalman.h"

// ---- Kalman Filter for firstcorner_pos ----
static KalmanFilter kf_firstcorner;
static int kf_firstcorner_initialized = 0;
static int kf_firstcorner_loss_count = 0; // 连续丢失帧数计数器
#define KALMAN_MAX_LOSS_FRAMES 5 // 禁用卡尔曼滤波器的连续丢失帧数阈值
uint8_t firstcorner_pos[2]={0,0};
uint8_t firstcorner_pos_filtered[2] = {0, 0}; // 滤波后的位置 [x, y]
uint8_t secondcorner_pos[2]={0,0};
uint8_t secondcorner_pos_filtered[2] = {0, 0}; // 滤波后的位置 [x, y]
// -----------------------------------------

// --- IMO 数组颜色映射说明 ---
// imo 数组中的特定值在 GUI 中会被渲染成不同的颜色，用于可视化。
// 0: 黑色 (Black)
// 1: 红色 (Red) - 用于左边界点
// 2: 橙色 (Orange) - 用于右边界点
// 3: 黄色 (Yellow) - 用于中线
// 4: 绿色 (Green) - 用于左边界
// 5: 青色 (Cyan) - 用于右边界
// 6: 洋红色 (Magenta)
// 7: 蓝色 (Blue)
// 8: 紫色 (Purple)
// 9: 粉色 (Pink)
// 255: 白色 (White) - 默认背景或赛道内部
// -----------------------------------------

// ---- 桌面环境桩：移除嵌入式依赖，提供最小可编译实现 ----
// 若在嵌入式环境下已有这些外部符号，可在编译时定义以下宏以禁用本文件的桩：
//  - HAVE_EXTERNAL_GRAYSCALE
//  - HAVE_EXTERNAL_IMO
//  - HAVE_EXTERNAL_LCD_SHOW

// 使用 global_image_buffer.h 中的全局数组

#ifndef HAVE_EXTERNAL_LCD_SHOW
// LCD 显示函数空实现（避免链接错误）
void show_ov2640_image_int8(int start_x, int start_y,
							uint8_t *data,
							int w, int h,
							int stride_w, int stride_h)
{
	(void)start_x; (void)start_y; (void)data; (void)w; (void)h; (void)stride_w; (void)stride_h;
}
#endif

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16_t limit_a_b(int16_t x, int16_t a, int16_t b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16_t limit(int16_t x, int16_t y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16_t limit1(int16_t x, int16_t y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}

//二值化后bin_image用Grayscale取代

/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8_t start_point_l[2] = { 0 };//左边起点的x，y值
uint8_t start_point_r[2] = { 0 };//右边起点的x，y值
uint8_t get_start_point(uint8_t start_row)
{
	uint16_t i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = image_w / 2; i >= border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (imo[start_row][i] == 255 && imo[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i <= border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (imo[start_row][i] == 255 && imo[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[image_w]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16_t points_l[(uint16_t)USE_num][2] = { {  0 } };//左线
uint16_t points_r[(uint16_t)USE_num][2] = { {  0 } };//右线
uint16_t dir_r[(uint16_t)USE_num] = { 0 };//用来存储右边生长方向
uint16_t dir_l[(uint16_t)USE_num] = { 0 };//用来存储左边生长方向
uint16_t data_stastics_l = 0;//统计左边找到点的个数
uint16_t data_stastics_r = 0;//统计右边找到点的个数
uint8_t hightest = 0;//最高点
void search_l_r(uint16_t break_flag, uint8_t(*image)[image_w], uint16_t *l_stastic, uint16_t *r_stastic, uint8_t l_start_x, uint8_t l_start_y, uint8_t r_start_x, uint8_t r_start_y, uint8_t *hightest)
{

	uint8_t i = 0, j = 0;

	//左边变量
	uint8_t search_filds_l[8][2] = { {  0 } };
	uint8_t index_l = 0;
	uint8_t temp_l[8][2] = { {  0 } };
	uint8_t center_point_l[2] = {  0 };
	uint16_t l_data_statics;//统计左边
	//定义八个邻域（左线顺时针扫描）
	static int8_t seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	// 索引:     0:向下   1:左下   2:向左   3:左上   4:向上   5:右上   6:向右   7:右下
	// 
	// ⚠️ 重要：dir_l[]记录值与实际生长方向存在+1偏移
	// 检测逻辑：if(image[i]==0 && image[i+1]==255) 时记录i，但实际选择i+1
	// 
	// dir_l[]记录值 → 实际生长方向对应关系：
	//   记录0 → 实际生长seeds_l[1]={-1,1}  → 左下
	//   记录1 → 实际生长seeds_l[2]={-1,0}  → 向左
	//   记录2 → 实际生长seeds_l[3]={-1,-1} → 左上
	//   记录3 → 实际生长seeds_l[4]={0,-1}  → 向上（主要生长方向）
	//   记录4 → 实际生长seeds_l[5]={1,-1}  → 右上
	//   记录5 → 实际生长seeds_l[6]={1,0}   → 向右
	//   记录6 → 实际生长seeds_l[7]={1,1}   → 右下
	//   记录7 → 实际生长seeds_l[0]={0,1}   → 向下
	//
	// 常见模式：
	//   3,3,3... → 持续向上爬升
	//   4,4,4... → 持续右上爬升
	//   2,1,0... → 左转（左上→向左→左下）

	//右边变量
	uint8_t search_filds_r[8][2] = { {  0 } };
	uint8_t center_point_r[2] = { 0 };//中心坐标点
	uint8_t index_r = 0;//索引下标
	uint8_t temp_r[8][2] = { {  0 } };
	uint16_t r_data_statics;//统计右边
	//定义八个邻域（右线逆时针扫描）
	static int8_t seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	// 索引:     0:向下   1:右下   2:向右   3:右上   4:向上   5:左上   6:向左   7:左下
	//
	// ⚠️ 重要：dir_r[]记录值与实际生长方向存在+1偏移
	// 检测逻辑：if(image[i]==0 && image[i+1]==255) 时记录i，但实际选择i+1
	//
	// dir_r[]记录值 → 实际生长方向对应关系：
	//   记录0 → 实际生长seeds_r[1]={1,1}   → 右下
	//   记录1 → 实际生长seeds_r[2]={1,0}   → 向右
	//   记录2 → 实际生长seeds_r[3]={1,-1}  → 右上
	//   记录3 → 实际生长seeds_r[4]={0,-1}  → 向上（主要生长方向）
	//   记录4 → 实际生长seeds_r[5]={-1,-1} → 左上
	//   记录5 → 实际生长seeds_r[6]={-1,0}  → 向左
	//   记录6 → 实际生长seeds_r[7]={-1,1}  → 左下
	//   记录7 → 实际生长seeds_r[0]={0,1}   → 向下
	//
	// 常见模式：
	//   3,3,3... → 持续向上爬升
	//   4,4,4... → 持续左上爬升
	//   2,1,0... → 右转（右上→向右→右下）

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			// 边界检测：i位置是黑(赛道外) 且 i+1位置是白(赛道内) → 找到黑白边界
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				// 实际选择i+1的坐标（白色区域的边缘点）
				temp_l[index_l][0] = search_filds_l[(i + 1) & 7][0];
				temp_l[index_l][1] = search_filds_l[(i + 1) & 7][1];
				index_l++;
				// 记录i（表示在i方向检测到黑色边界，实际生长方向是i+1）
				// 例：记录3表示在左上(3)检测到黑色，实际向上(4)生长
				dir_l[l_data_statics - 1] = (i);
			}
		}

		// 决策逻辑移到循环外部
		if (index_l)
		{
			//更新坐标点
			center_point_l[0] = temp_l[0][0];//x
			center_point_l[1] = temp_l[0][1];//y
			for (j = 0; j < index_l; j++)
			{
				if (center_point_l[1] > temp_l[j][1])
				{
					center_point_l[0] = temp_l[j][0];//x
					center_point_l[1] = temp_l[j][1];//y
				}
			}
		}
		if ((r_data_statics >= 2 && points_r[r_data_statics][0] == points_r[r_data_statics-1][0] && points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            || (l_data_statics >= 3 && points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2
			)
		{
			//printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			//printf("\n如果左边比右边高了，左边等待右边\n");	
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			// dir_l==7 表示记录了7，实际生长方向是seeds_l[0]={0,1}即向下
			// 左线开始向下说明可能遇到十字路口或环岛，等待右边
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			// 边界检测：i位置是黑(赛道外) 且 i+1位置是白(赛道内) → 找到黑白边界
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				// 实际选择i+1的坐标（白色区域的边缘点）
				temp_r[index_r][0] = search_filds_r[(i + 1) & 7][0];
				temp_r[index_r][1] = search_filds_r[(i + 1) & 7][1];
				index_r++;//索引加一
				// 记录i（表示在i方向检测到黑色边界，实际生长方向是i+1）
				// 例：记录3表示在右上(3)检测到黑色，实际向上(4)生长
				dir_r[r_data_statics - 1] = (i);
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
		}

		// 决策逻辑移到循环外部
		if (index_r)
		{
			//更新坐标点
			center_point_r[0] = temp_r[0][0];//x
			center_point_r[1] = temp_r[0][1];//y
			for (j = 0; j < index_r; j++)
			{
				if (center_point_r[1] > temp_r[j][1])
				{
					center_point_r[0] = temp_r[j][0];//x
					center_point_r[1] = temp_r[j][1];//y
				}
			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
 */
uint8_t l_border[image_h];//左线数组
uint8_t r_border[image_h];//右线数组
uint8_t center_line[image_h];//中线数组
uint8_t left_lost[image_h];//左线丢失标志数组
uint8_t right_lost[image_h];//右线丢失标志数组
uint8_t last_left_lost_down=0;//记录左边下方最后一次左线丢失的位置 注意1这是由于局限的 这里主要是为了后续直线判断
uint8_t last_right_lost_down=0;//记录右边下方最后一次右线丢失的位置  注意2这是索引 实际丢线行数值要再+1
uint8_t last_left_lost_midstart=0;//记录中间段丢线开始位置
uint8_t last_right_lost_midstart=0;//记录中间段丢线开始位置
uint8_t last_left_lost_midend=0;//记录中间段丢线结束位置
uint8_t last_right_lost_midend=0;//记录中间段丢线结束位置
uint8_t last_left_lost_up=image_h-1;//记录左边上方最后一次左线丢失的位置
uint8_t last_right_lost_up=image_h-1;//记录右边上方最后一次右线丢失的位置 注意3这是索引 实际丢线行数为image_h - last_right_lost_up
uint8_t left_lost_num=image_h;//左线丢失总行数
uint8_t right_lost_num=image_h;//右线丢失总行数

#define max(a, b) ((a) > (b) ? (a) : (b))

void get_left(uint16_t total_L)
{
	uint16_t j;
	left_lost_num=image_h;
	last_left_lost_midstart=0;last_left_lost_midend=0;
	//初始化左边界为最小值（丢线状态）
	for (j = 0; j < image_h; j++)
	{
		l_border[j] = border_min;
		left_lost[j] = 1;  // 丢线标志
	}

	// 遍历所有找到的点，更新l_border数组（过程中反转行号）
	for (j = 0; j < total_L; j++)
	{
		uint16_t row = image_h - 1 - points_l[j][1]; // 反转行号
		uint16_t col = points_l[j][0];
		if (row < image_h) // 确保行号在范围内 其实没必要
		{	
			// 若非丢线 取该行最右边的左边界点
			if (col > l_border[row])
			{
				// 只在第一次找到该行边界时减少丢线计数
				if (left_lost[row] == 1) {
					left_lost_num--;
				}
				l_border[row] = col;
				left_lost[row] = 0;
			}
		}
	}

	//从下往上找丢线
	for(uint8_t row=0;row<image_h;row++){
		if(left_lost[row]==1&&left_lost[row+1]==0/*&&left_lost[row+2]==0*/)
		{
			last_left_lost_down=row;
			break;
		}
	}
	//从上往下找丢线
	for(int8_t row=image_h-1;row>=0;row--){
		if(left_lost[row]==1&&left_lost[row-1]==0/*&&left_lost[row-2]==0*/)
		{
			last_left_lost_up=row;
			break;
		}
	}
	uint8_t temp_lost_num=image_h - last_left_lost_up+last_left_lost_down+1;//这是上加下丢线总行数 总是小于等于left_lost_num 中间段不丢线时取等
	//判断是否还有一段丢线
	if(left_lost_num-temp_lost_num>=3)// 这里3为了抗噪
	{
		//如果有中间段 从上往下看 因为下面两个角落经常糊
		for(uint8_t row=last_left_lost_up-2;row>=last_left_lost_down+2;row--)
		{
			if(left_lost[row]==1&&left_lost[row+1]==0&&/*left_lost[row+2]==0&&*/last_left_lost_midend==0)
			{
				last_left_lost_midend=row;
			}
			if(left_lost[row]==1&&left_lost[row-1]==0&&/*left_lost[row-2]==0&&*/last_left_lost_midstart==0)
			{
				last_left_lost_midstart=row;
				break;
			}
		}
	}
    // 角落糊了还是没办法 手动补一下
	if(last_left_lost_midstart<=10&&last_left_lost_midstart!=0)
	{
		last_left_lost_midstart=0;
		last_left_lost_down=last_left_lost_midend;
		last_left_lost_midend=0;
	}
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16_t total_R)
{
	uint16_t j;
	right_lost_num=image_h;
	last_right_lost_midstart=0;last_right_lost_midend=0;
	//初始化右边界为最大值（丢线状态）
	for (j = 0; j < image_h; j++)
	{
		r_border[j] = border_max;
		right_lost[j] = 1;  // 丢线标志
	}

	// 遍历所有找到的点，更新r_border数组（过程中反转行号）
	for (j = 0; j < total_R; j++)
	{
		uint16_t row = image_h - 1 - points_r[j][1]; // 反转行号
		uint16_t col = points_r[j][0];
		if (row < image_h) // 确保行号在范围内
		{
			// 若非丢线 取该行最左边的右边界点
			if (col < r_border[row])
			{
				// 只在第一次找到该行边界时减少丢线计数
				if (right_lost[row] == 1) {
					right_lost_num--;
				}
				r_border[row] = col;
				right_lost[row] = 0;
			}
		}
	}
	//从下往上找丢线
	for(uint8_t row=0;row<image_h;row++){
		if(right_lost[row]==1&&right_lost[row+1]==0/*&&right_lost[row+2]==0*/)
		{
			last_right_lost_down=row;
			break;
		}
	}
	//从上往下找丢线
	for(uint8_t row=image_h-1;row>=0;row--){
		if(right_lost[row]==1&&right_lost[row-1]==0/*&&right_lost[row-2]==0*/)
		{
			last_right_lost_up=row;
			break;
		}
	}
	uint8_t temp_lost_num=image_h - last_right_lost_up+last_right_lost_down+1;//这是上加下丢线总行数 总是小于等于right_lost_num 中间段不丢线时取等
	//判断是否还有一段丢线
	if(right_lost_num-temp_lost_num>=3)// 这里3为了抗噪
	{
		//如果有中间段 从上往下找 因为下面两个角落经常糊
		for(uint8_t row=last_right_lost_up-2;row>=last_right_lost_down+2;row--)
		{
			if(right_lost[row]==1&&right_lost[row+1]==0&&/*right_lost[row+2]==0&&*/last_right_lost_midend==0)
			{
				last_right_lost_midend=row;
			}
			if(right_lost[row]==1&&right_lost[row-1]==0&&/*right_lost[row-2]==0&&*/last_right_lost_midstart==0)
			{
				last_right_lost_midstart=row;
				break;
			}
		}
	}
	// 角落糊了还是没办法 手动补一下
	if(last_right_lost_midstart<=10&&last_right_lost_midstart!=0)
	{
		last_right_lost_midstart=0;
		last_right_lost_down=last_right_lost_midend;
		last_right_lost_midend=0;
	}
}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框（1像素宽）
参数说明：uint8(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8_t(*image)[image_w])
{
	uint8_t i = 0;
	
	// 左右边框：各1列
	for (i = 0; i < image_h; i++)
	{
		image[i][0] = 0;           // 最左边
		image[i][image_w - 1] = 0; // 最右边
	}
	
	// 上边框：1行
	for (i = 0; i < image_w; i++)
	{
		image[0][i] = 0; // 最上面
	}
}

/*绘制边界线(横向去重)
void draw_edge()
{
	int row=0;
	for(row=0;row<120;row++)
    {
		imo[119-row][l_border[row]]=1;
		imo[119-row][r_border[row]]=2;
		imo[119-row][center_line[row]]=3;
	}
}
*/

//绘制边界线(完全体)
void draw_edge()
{
    // 显示左边界
    for (int i = 0; i < data_stastics_l; i++) {
        int row = points_l[i][1];
        int col = points_l[i][0];
        imo[row][col] = 1; // 左边界点标记为1
    }
    // 显示右边界
    for (int i = 0; i < data_stastics_r; i++) {
        int row = points_r[i][1];
        int col = points_r[i][0];
        imo[row][col] = 2; // 右边界点标记为2
    }
    // 显示中线
    for (int row = 0; row < image_h; row++) {
		// 这里y索引要颠倒 因为最终左、右、中线是从底部向上 而imo是从顶部向下
        imo[image_h-1-row][center_line[row]] = 3;
		imo[image_h-1-row][l_border[row]] = 4;
		imo[image_h-1-row][r_border[row]] = 5;

    }
	// 显示拐点及其周围的3x3区域
	for(int i=0;i<9;i++)
	{
		imo[image_h-firstcorner_pos[1]][firstcorner_pos[0]] = 6;
		imo[image_h-1-firstcorner_pos[1]][firstcorner_pos[0]] = 6;
		imo[image_h-2-firstcorner_pos[1]][firstcorner_pos[0]] = 6;
	    imo[image_h-firstcorner_pos[1]][firstcorner_pos[0]+1] = 6;
		imo[image_h-1-firstcorner_pos[1]][firstcorner_pos[0]+1] = 6;
		imo[image_h-2-firstcorner_pos[1]][firstcorner_pos[0]+1] = 6;
        imo[image_h-firstcorner_pos[1]][firstcorner_pos[0]-1] = 6;
		imo[image_h-1-firstcorner_pos[1]][firstcorner_pos[0]-1] = 6;
		imo[image_h-2-firstcorner_pos[1]][firstcorner_pos[0]-1] = 6;
	    imo[image_h-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]] = 7;
		imo[image_h-1-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]] = 7;
		imo[image_h-2-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]] = 7;
	    imo[image_h-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]+1] = 7;
		imo[image_h-1-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]+1] = 7;
		imo[image_h-2-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]+1] = 7;
        imo[image_h-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]-1] = 7;
		imo[image_h-1-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]-1] = 7;
		imo[image_h-2-firstcorner_pos_filtered[1]][firstcorner_pos_filtered[0]-1] = 7;
	}
}


/**
 * @brief 整数序列匹配函数
 *
 * 功能：
 * 1. 在 'input' 序列中查找 'pattern' 序列。
 * 2. 严格匹配 'pattern' 中的每一个元素, 包括重复。
 * 3. 允许 'pattern' 中相邻元素之间存在最多 'max_gap' 个 "噪声" 元素。
 *
 * @param input        输入整数序列
 * @param input_len    输入长度
 * @param pattern      目标模式序列
 * @param pattern_len  模式长度 (必须 > 0)
 * @param max_gap      允许的最大单段间隔 (应 >= 0)
 * @param start_pos    可选：从输入序列的第几个位置开始匹配，默认传0从头开始
 *
 * @return match_result_t 结构体, 包含匹配状态和置信度
 * @note result.start 和 result.end 返回的是匹配在整个input数组中的起始和结束索引（不是相对于start_pos） 永远有start<end 不论匹配顺序
 */
match_result match_strict_sequence_with_gaps(
    const uint16_t* input,     // 输入序列
    size_t         input_len,
    const uint16_t* pattern,    //目标模式序列
    size_t         pattern_len,
    uint16_t        max_gap,      // 允许的最大单段间隔
    size_t         start_pos,     // 起始匹配位置（新增参数）
    int8_t         direction       // 1=正向, -1=反向
) {
    match_result result = {0, 0, 0, 0, 0.0f};
    if (!input || !pattern || pattern_len == 0 || input_len == 0) {
        return result;
    }
    if (start_pos >= input_len) {
        return result;
    }

    size_t pat_idx = 0;
    uint16_t current_gap = 0;
    uint16_t total_gap = 0;
    size_t match_start = 0;

    if (direction == 1) {
        // 正向匹配：pattern[0] -> pattern[pattern_len-1]
        // 使用步长为1的滑动窗口，确保尝试每个位置作为起点
        for (size_t window_start = start_pos; window_start < input_len; window_start++) {
            // 检查剩余长度是否足够
            if (input_len - window_start < pattern_len) break;
            
            // 重置匹配状态，从新的窗口起点开始
            pat_idx = 0;
            current_gap = 0;
            total_gap = 0;
            match_start = 0;
            
            // 从 window_start 开始向后匹配
            for (size_t i = window_start; i < input_len; i++) {
                if (input[i] == pattern[pat_idx]) {
                    if (pat_idx == 0) match_start = i;
                    if (pat_idx > 0) total_gap += current_gap;
                    pat_idx++;
                    current_gap = 0;

                    if (pat_idx == pattern_len) {
                        // 匹配成功！
                        result.matched = 1;
                        result.total_gap = total_gap;
                        result.start = (uint16_t)match_start;
                        result.end = (uint16_t)i;
                        uint16_t max_possible_gap = (uint16_t)(pattern_len - 1) * max_gap;
                        if (max_possible_gap == 0) {
                            result.confidence = (total_gap == 0) ? 1.0f : 0.0f;
                        } else {
                            result.confidence = 1.0f - (float)total_gap / (float)max_possible_gap;
                        }
                        return result;
                    }
                } else {
                    if (pat_idx > 0) {
                        current_gap++;
                        if (current_gap > max_gap) {
                            // gap超限，当前窗口匹配失败，跳到下一个窗口
                            break;
                        }
                    }
                }
            }
        }
    } else if (direction == -1) {
        // 反向匹配：按 pattern[pattern_len-1] -> pattern[0] 顺序匹配
        // 使用步长为1的滑动窗口，确保尝试每个位置作为起点
        int start = (int)start_pos >= (int)input_len ? (int)input_len - 1 : (int)start_pos;
        
        for (int window_start = start; window_start >= 0; window_start--) {
            // 检查剩余长度是否足够
            if (window_start + 1 < (int)pattern_len) break;
            
            // 重置匹配状态，从新的窗口起点开始
            pat_idx = 0;
            current_gap = 0;
            total_gap = 0;
            match_start = 0;
            
            // 从 window_start 开始向前匹配
            for (int i = window_start; i >= 0; i--) {
                size_t reverse_pat_idx = pattern_len - 1 - pat_idx;
                
                if (input[i] == pattern[reverse_pat_idx]) {
                    if (pat_idx == 0) match_start = i;
                    if (pat_idx > 0) total_gap += current_gap;
                    pat_idx++;
                    current_gap = 0;
                    
                    if (pat_idx == pattern_len) {
                        // 匹配成功！
                        result.matched = 1;
                        result.total_gap = total_gap;
                        result.start = (uint16_t)i;
                        result.end = (uint16_t)match_start;
                        uint16_t max_possible_gap = (uint16_t)(pattern_len - 1) * max_gap;
                        if (max_possible_gap == 0) {
                            result.confidence = (total_gap == 0) ? 1.0f : 0.0f;
                        } else {
                            result.confidence = 1.0f - (float)total_gap / (float)max_possible_gap;
                        }
                        return result;
                    }
                } else {
                    if (pat_idx > 0) {
                        current_gap++;
                        if (current_gap > max_gap) {
                            // gap超限，当前窗口匹配失败，跳到下一个窗口
                            break;
                        }
                    }
                }
            }
        }
    }

    return result;
}

//byd八位和十六位整数指针不兼容
match_result match_strict_sequence_with_gaps_u8(
    const uint8_t* input,
    size_t         input_len,
    const uint8_t* pattern,
    size_t         pattern_len,
    uint8_t        max_gap,
    size_t         start_pos,
    int8_t            direction
) {
    match_result result = {0, 0, 0, 0, 0.0f};
    if (!input || !pattern || pattern_len == 0 || input_len == 0) {
        return result;
    }
    if (start_pos >= input_len) {
        return result;
    }

    size_t pat_idx = 0;
    uint8_t current_gap = 0;
    uint8_t total_gap = 0;
    size_t match_start = 0;

    if (direction == 1) {
        // 正向匹配：从 pattern[0] 到 pattern[pattern_len-1]
        // 使用步长为1的滑动窗口，确保尝试每个位置作为起点
        for (size_t window_start = start_pos; window_start < input_len; window_start++) {
            // 检查剩余长度是否足够
            if (input_len - window_start < pattern_len) break;
            
            // 重置匹配状态，从新的窗口起点开始
            pat_idx = 0;
            current_gap = 0;
            total_gap = 0;
            match_start = 0;
            
            // 从 window_start 开始向后匹配
            for (size_t i = window_start; i < input_len; i++) {
                if (input[i] == pattern[pat_idx]) {
                    if (pat_idx == 0) match_start = i;
                    if (pat_idx > 0) total_gap += current_gap;
                    pat_idx++;
                    current_gap = 0;
                    
                    if (pat_idx == pattern_len) {
                        // 匹配成功！
                        result.matched = 1;
                        result.start = (uint8_t)match_start;
                        result.end = (uint8_t)i;
                        result.total_gap = total_gap;
                        uint8_t max_possible_gap = (uint8_t)(pattern_len - 1) * max_gap;
                        result.confidence = (max_possible_gap == 0) ? ((total_gap == 0) ? 1.0f : 0.0f)
                            : 1.0f - (float)total_gap / (float)max_possible_gap;
                        return result;
                    }
                } else {
                    if (pat_idx > 0) {
                        current_gap++;
                        if (current_gap > max_gap) {
                            // gap超限，当前窗口匹配失败，跳到下一个窗口
                            break;
                        }
                    }
                }
            }
        }
    } else if (direction == -1) {
        // 反向匹配：从 pattern[pattern_len-1] 倒着到 pattern[0]
        // 使用步长为1的滑动窗口，确保尝试每个位置作为起点
        int start = (int)start_pos >= (int)input_len ? (int)input_len - 1 : (int)start_pos;
        
        for (int window_start = start; window_start >= 0; window_start--) {
            // 检查剩余长度是否足够
            if (window_start + 1 < (int)pattern_len) break;
            
            // 重置匹配状态，从新的窗口起点开始
            pat_idx = 0;
            current_gap = 0;
            total_gap = 0;
            match_start = 0;
            
            // 从 window_start 开始向前匹配
            for (int i = window_start; i >= 0; i--) {
                // 关键：反向匹配时，从 pattern 末尾开始
                size_t reverse_pat_idx = pattern_len - 1 - pat_idx;
                
                if (input[i] == pattern[reverse_pat_idx]) {
                    if (pat_idx == 0) match_start = i;
                    if (pat_idx > 0) total_gap += current_gap;
                    pat_idx++;
                    current_gap = 0;
                    
                    if (pat_idx == pattern_len) {
                        // 匹配成功！
                        result.matched = 1;
                        result.start = (uint8_t)i;           // 反向时end在前
                        result.end = (uint8_t)match_start;   // start在后
                        result.total_gap = total_gap;
                        uint8_t max_possible_gap = (uint8_t)(pattern_len - 1) * max_gap;
                        result.confidence = (max_possible_gap == 0) ? ((total_gap == 0) ? 1.0f : 0.0f)
                            : 1.0f - (float)total_gap / (float)max_possible_gap;
                        return result;
                    }
                } else {
                    if (pat_idx > 0) {
                        current_gap++;
                        if (current_gap > max_gap) {
                            // gap超限，当前窗口匹配失败，跳到下一个窗口
                            break;
                        }
                    }
                }
            }
        }
    }
    return result;
}


// 创建匹配序列（用于元素识别）
//  注意：这些序列值是dir_l/dir_r的记录值，不是实际生长方向 但是后续判断就用这个
// 实际生长方向 = seeds[(记录值+1) & 7]
growth_array arr = {
    .up = {3,3,3,3,3,3},           
    .outer = {1,1,1,1,1,1},
	.up_inner={4,4,4,4,4,4},
    .inner = {5,5,5,5,5,5},       
    .corner1 = {4,3,2,1,1,1}
};

/** 
* @brief 计算边界拟合方差（最小二乘法）
* @param begin					输入起点
* @param end					输入终点
* @param border				输入需要计算的边界数组首地址
*  @see CTest		float variance = calculate_border_variance(start, end, border);
* @return 返回拟合方差值，值越小说明边界越接近直线
*     -<em>-1.0f</em> 计算失败（参数错误或数据点不足）
*     -<em>>=0</em> 拟合方差值
* @note 方差表示边界点与拟合直线的平均偏离程度，可用于判断直线质量
*/
float calculate_border_variance(uint8_t begin, uint8_t end, uint8_t *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	uint16_t i;
	uint16_t num = 0;
	float x_average, y_average;
	float slope, intercept;
	static float slope_last = 0.0f;
	
	// 参数检查
	if (end <= begin || border == NULL) {
		return -1.0f;
	}
	
	// 累加计算
	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * border[i];
		x2sum += i * i;
		num++;
	}
	
	// 检查数据点数量
	if (num == 0) {
		return -1.0f;
	}
	
	// 计算平均值
	x_average = xsum / num;
	y_average = ysum / num;
	
	// 计算斜率（最小二乘法）
	float denominator = num * x2sum - xsum * xsum;
	if (denominator != 0) {
		slope = (num * xysum - xsum * ysum) / denominator;
		slope_last = slope;
	} else {
		// 除数为零时使用上次结果
		slope = slope_last;
	}
	
	// 计算截距
	intercept = y_average - slope * x_average;
	
	// 计算方差（拟合残差的平方和除以样本数）
	float variance = 0;
	for (i = begin; i < end; i++)
	{
		float fitted_y = slope * i + intercept;
		float residual = border[i] - fitted_y;
		variance += residual * residual;
	}
	variance /= num;
	
	return variance;
}

/** 
十字补线函数
 */
uint8_t cross_flag=0;
void cross_detect(uint16_t total_num_l, uint16_t total_num_r,uint16_t *dir_l, uint16_t *dir_r, uint16_t(*points_l)[2], uint16_t(*points_r)[2])
{
	int temp1=0,temp2=0;
	cross_flag=0;
	// 左边匹配检测
	match_result result_l1 = match_strict_sequence_with_gaps(dir_l, total_num_l, arr.up, 6, 0, 0, 1);
	if(result_l1.matched){
		match_result result_l2 =match_strict_sequence_with_gaps(dir_l, total_num_l, arr.inner, 6, 0, result_l1.end, 1);
		if(result_l2.matched){
			temp1=image_h-1-points_l[result_l2.end][1];//这段是cross上边缘 记录行数
			match_result result_l3 = match_strict_sequence_with_gaps(dir_l, total_num_l, arr.up_inner, 6, 2, result_l2.end, 1);
			if(!result_l3.matched){ 
				return;
			}
		}
		else{
			return;
		}
	}
	else{
		return;
	}
	match_result result_r1 = match_strict_sequence_with_gaps(dir_r, total_num_r, arr.up, 6, 0, 0, 1);
	if(result_r1.matched){
		match_result result_r2 = match_strict_sequence_with_gaps(dir_r, total_num_r, arr.inner, 6, 0, result_r1.end, 1);
		if(result_r2.matched){
			temp2=image_h-1-points_r[result_r2.end][1];//这段是cross上边缘 记录行数
			match_result result_r3 = match_strict_sequence_with_gaps(dir_r, total_num_r, arr.up_inner, 6, 2, result_r2.end, 1);
			if(!result_r3.matched){ 
				return;
			}

		}
		else{
			return;
		}
	}
	else{
		return;
	}
	//上面的检察全过才能到这里 所以直接赋1 补线流程集成在一块 flag主要是为了日志记录
	cross_flag=1;
	/*补线
	log_add_int16("temp1", temp1, -1);
	log_add_int16("temp2", temp2, -1);
	uint8_t templ=l_border[temp1+3],tempr=r_border[temp2+3];
	for(uint8_t i=0;(i<=temp1)||(i<=temp2);i++)
	{
		//补竖线得了
		l_border[i]=templ;
		r_border[i]=tempr;
	}*/
	
}



//直线检测函数
uint8_t left_straight=0,right_straight=0,straight=0;
void straight_detect(uint8_t *l, uint8_t *r,uint16_t start_l,uint16_t start_r,uint16_t end_l,uint16_t end_r)
{
	// 先清零
	right_straight=0;
	left_straight=0;
	straight=0;
	float left_variance = calculate_border_variance(start_l, end_l, l);
	float right_variance = calculate_border_variance(start_r, end_r, r);
	log_add_float("left_variance", left_variance, -1);
	log_add_float("right_variance", right_variance, -1);

	// 这里留了一个不那么严格的直线判断标准 值为2
	left_straight = (left_variance < 10.0f)?1u:(left_variance < 50.0f?2u:0u);
	right_straight = (right_variance < 10.0f)?1u:(right_variance < 50.0f?2u:0u);
	straight = left_straight && right_straight;
}

/*

环岛检测函数群

*/
uint8_t island_flag=0;
uint8_t first_corner=0;
uint8_t second_corner=0;
uint8_t count_down=0;
//uint8_t firstcorner_pos[2]={0,0};//记录第一个角点位置 行列 写在顶部了
void firstcorner_detect(uint16_t total_num_l, uint16_t total_num_r,uint16_t *dir_l, uint16_t *dir_r, uint16_t(*points_l)[2], uint16_t(*points_r)[2])
{
	uint16_t match_start_l=total_num_l-1,match_start_r=total_num_r-1;
	match_result result_cl={0,0,0,0,0.0f};
	match_result result_cr={0,0,0,0,0.0f};
	first_corner=0;
	/* 见到环岛第一个角点时 环的部分会有丢线 也就是last_left/right_lost_midstart非零 我们从midstart行开始反向匹配第一角点序列
	   但midstart是从下往上数的行数 我们要找出它对应的dir_l/dir_r索引位置

	   一般来说 在从下往上数的第y行上 一个点满足image_h-1-points_l/r[i][1]==y 其中i就是我们要找的dir_l/r的索引
	   且i>=y 所以我们就可以在一个相对缩小的范围内 根据y搜出i
	*/

	//如果没有中间丢线 直接返回
	if(last_left_lost_midstart==0&&last_right_lost_midstart==0)
	    //return;
		;
	//中部左丢右不丢 检测左第一角点
    else if(last_left_lost_midstart!=0&&last_right_lost_midstart==0)
	{
	// 将 l_border 索引转换为原始图像 y 坐标
	uint16_t target_y = image_h - 1 - last_left_lost_midstart;
	// 从 last_left_lost_midstart 开始搜索，必能找到或越过目标行
	for(uint16_t i=last_left_lost_midstart; i<total_num_l; i++)
	{
		if(points_l[i][1] == target_y)
		{
			match_start_l=i;
			break;
		}
		// 提前终止：如果 y 坐标已经比 target_y 小（更靠近顶部），说明已越过目标行，但一般不会
		if(points_l[i][1] <= target_y)
			break;
	}
	result_cl = match_strict_sequence_with_gaps(dir_l, total_num_l, arr.corner1, 6, 4, match_start_l, -1);
    }

	//中部右丢左不丢 检测右第一角点
	else if(last_left_lost_midstart==0&&last_right_lost_midstart!=0)
	{
	// 将 r_border 索引转换为原始图像 y 坐标
	uint16_t target_y = image_h - 1 - last_right_lost_midstart;
	// 从 last_right_lost_midstart 开始搜索，必能找到或越过目标行
	for(uint16_t i=last_right_lost_midstart; i<total_num_r; i++)
	{
		if(points_r[i][1] == target_y)
		{
			match_start_r=i;
			break;
		}
		// 提前终止：如果 y 坐标已经比 target_y 小（更靠近顶部），说明已越过目标行，但一般不会
		if(points_r[i][1] <= target_y)
			break;
	}
	result_cr = match_strict_sequence_with_gaps(dir_r, total_num_r, arr.corner1, 6, 4, match_start_r, -1);
    }

	if((left_straight!=0)&&result_cr.matched)
	{
		first_corner=1;
		count_down=8;//见到第一角点就重置计数器
		firstcorner_pos[0]=points_r[result_cr.start][0];// x
		firstcorner_pos[1]=image_h-1-points_r[result_cr.start][1];// y
		// 右环
	}
	else if((right_straight!=0)&&result_cl.matched)
	{
		first_corner=2;
		count_down=8;//见到第一角点就重置计数器
	    firstcorner_pos[0]=points_l[result_cl.start][0];// x
		firstcorner_pos[1]=image_h-1-points_l[result_cl.start][1];// y
		// 左环
	}
	else
	{
		if(count_down>0)
		count_down--;
		first_corner=0;
		firstcorner_pos[0]=0;
		firstcorner_pos[1]=0;
	}
	log_add_uint8("横firstcorner_pos", firstcorner_pos[0], -1);
	log_add_uint8("纵firstcorner_pos", firstcorner_pos[1], -1);


	/*
    // --- 卡尔曼滤波器集成 ---
    if (first_corner != 0) { // 仅当检测到角点时更新卡尔曼滤波器
        float current_measurement[2] = {(float)firstcorner_pos[0], (float)firstcorner_pos[1]};

        if (!kf_firstcorner_initialized) {
            // 使用第一个有效测量值初始化卡尔曼滤波器
            // 过程噪声 (Q) 和测量噪声 (R) 是可调参数。
            // dt 暂时设为 1.0，表示一个帧间隔。
            // 如果帧率可变，dt 应动态计算。
            kalman_init(&kf_firstcorner, current_measurement[0], current_measurement[1], 1.0f, 20.0f);//r20
            kf_firstcorner_initialized = 1;
        } else {
            kalman_predict(&kf_firstcorner, 1.0f); // dt = 1.0f (一个帧间隔)
            kalman_update(&kf_firstcorner, current_measurement);
        }
        // 存储滤波后的位置
        firstcorner_pos_filtered[0] = (uint8_t)kf_firstcorner.x[0];
        firstcorner_pos_filtered[1] = (uint8_t)kf_firstcorner.x[1];
        kf_firstcorner_loss_count = 0; // 成功检测到时重置丢失计数
    } else { // 未检测到角点
        if (kf_firstcorner_initialized) {
            kf_firstcorner_loss_count++;
            if (kf_firstcorner_loss_count < KALMAN_MAX_LOSS_FRAMES) {
                // 在丢失阈值内继续预测
                kalman_predict(&kf_firstcorner, 1.0f);
                firstcorner_pos_filtered[0] = (uint8_t)kf_firstcorner.x[0];
                firstcorner_pos_filtered[1] = (uint8_t)kf_firstcorner.x[1];
            } else {
                // 超过丢失阈值，禁用滤波器
                kf_firstcorner_initialized = 0;
                kf_firstcorner_loss_count = 0; // 重置计数
                firstcorner_pos_filtered[0] = 0u; // 重置滤波后的位置
                firstcorner_pos_filtered[1] = 0u;
            }
        } else {
            // 滤波器未初始化或已禁用，重置滤波后的位置
            firstcorner_pos_filtered[0] = 0u;
            firstcorner_pos_filtered[1] = 0u;
        }
    }
    // --- 卡尔曼滤波器集成结束 ---

	log_add_uint8("横firstcorner_pos_filtered", firstcorner_pos_filtered[0], -1);
	log_add_uint8("纵firstcorner_pos_filtered", firstcorner_pos_filtered[1], -1);
	*/

	log_add_uint8("count_down", count_down, -1);
    log_add_uint8("result_cl.matched",result_cl.matched,-1);
	log_add_float("result_cl.confidence",result_cl.confidence,-1);
	log_add_uint8("result_cr.matched",result_cr.matched,-1);
	log_add_float("result_cr.confidence",result_cr.confidence,-1);
	log_add_uint8("first_corner", first_corner, -1);
}


void secondcorner_detect(uint16_t total_num_l, uint16_t total_num_r,uint16_t *dir_l, uint16_t *dir_r, uint16_t(*points_l)[2], uint16_t(*points_r)[2])
//第一角点检测逻辑内创建了一个计数器count_down 倒计N帧 第二角点检测函数检测到计数值非零才能进入检测逻辑
//注意这个倒计时每次见到第一角点都会重置 
{
;
}



/*
日志记录函数 日志统一写在这里
*/
void userlog()
{
	//log_add_uint8_array("左 丢left_lost", left_lost, image_h,-1);
	//log_add_uint8_array("右 丢right_lost", right_lost, image_h,-1);
	//log_add_uint8_array("左边 最终l_border", l_border, image_h,-1);
	//log_add_uint8_array("右边 最终r_border", r_border, image_h,-1);
	log_add_uint8("左直left_straight", left_straight, -1);
	log_add_uint8("右直right_straight", right_straight, -1);
	//log_add_uint8("环 1右2左island_flag", island_flag, -1);
	//log_add_uint8("十字路口cross_flag", cross_flag, -1);
	//log_add_uint8("左 上 丢last_left_lost_up", last_left_lost_up, -1);
	//log_add_uint8("左 下 丢last_left_lost_down", last_left_lost_down, -1);
	log_add_uint8("左 中 丢last_left_lost_midstart", last_left_lost_midstart, -1);
	//log_add_uint8("左 中 丢last_left_lost_midend", last_left_lost_midend, -1);
	//log_add_uint8("右 上 丢last_right_lost_up", last_right_lost_up, -1);
	//log_add_uint8("右 下 丢last_right_lost_down", last_right_lost_down, -1);
	log_add_uint8("右 中 丢last_right_lost_midstart", last_right_lost_midstart, -1);
	//log_add_uint8("右 中 丢last_right_lost_midend", last_right_lost_midend, -1);
	log_add_uint16_array("左 生长dir_l", dir_l, data_stastics_l,-1);
	log_add_uint16_array("右 生长dir_r", dir_r, data_stastics_r,-1);
}


/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
	uint16_t i;
	uint8_t Hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小

//滤波（形态学处理）
morph_clean_u8_binary_adapter(Grayscale[0], image_w, image_h, imo[0]);
image_draw_rectan(imo);//填黑框
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 3)||get_start_point(image_h - 5)||get_start_point(image_h - 7))//找到起点了，再执行八领域，没找到就一直找
{
	//printf("正在开始八领域\n");
	search_l_r((uint16_t)USE_num, imo, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	//printf("八邻域已结束\n");
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
	//处理函数放这里 不要放到if外面
    cross_detect(data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);//十字检测
	straight_detect(l_border, r_border, last_left_lost_down, last_right_lost_down, last_left_lost_up-3, last_right_lost_up-3);//直线检测 这里去掉顶部三行 因为有时左右线在右侧相交 左border会异常
	firstcorner_detect(data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
}
    //求中线
	for (i = Hightest; i < image_h; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
	}
    //显示边线
	draw_edge();

	userlog();
}


