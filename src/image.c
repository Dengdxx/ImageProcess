//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "morph_binary_bitpacked.h"
#include "global_image_buffer.h"
#include "dynamic_log.h"

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
void get_left(uint16_t total_L)
{
	uint16_t j;
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
			// 取该行最右边的左边界点
			if (col > l_border[row])
			{
				l_border[row] = col;
				left_lost[row] = 0;
			}
			else{//这里也没必要写
				left_lost[row] = 1;  // 丢线标志
			}
		}
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
			// 找到该行的任何有效点，都清除丢线标志
			right_lost[row] = 0;
			
			// 找到真实边界点：取该行最左边的右边界点
			if (col < r_border[row])
			{
				r_border[row] = col;
				right_lost[row] = 0;
			}
			else{
				right_lost[row] = 1;  // 丢线标志
			}
		}
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
        imo[image_h-row][center_line[row]] = 3;
		imo[image_h-row][l_border[row]] = 4;
		imo[image_h-row][r_border[row]] = 5;

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
 *
 * @return match_result_t 结构体, 包含匹配状态和置信度
 */
match_result match_strict_sequence_with_gaps(
    const uint16_t* input,     // 输入序列
    size_t         input_len,
    const uint16_t* pattern,    //目标模式序列
    size_t         pattern_len,
    uint16_t        max_gap       // 允许的最大单段间隔
) {
    // 默认结果
    match_result result = {0, 0, 0, 0.0f}; 
    
    // 1. 鲁棒性检查
    if (!input || !pattern || pattern_len == 0 || input_len == 0 || max_gap < 0) {
        return result;
    }

    size_t  pat_idx = 0;           // 模式索引 (使用 size_t)
    uint16_t current_gap = 0;     // 标准化
    uint16_t total_gap = 0;         // 标准化

    for (size_t i = 0; i < input_len; i++) {
        
        // 2. 提前退出剪枝
        if (input_len - i < pattern_len - pat_idx) {
            break; 
        }

        if (input[i] == pattern[pat_idx]) {
            // 3. 找到匹配项
            if (pat_idx > 0) {
                total_gap += current_gap;
            }
            pat_idx++;
            current_gap = 0; 

            // 4. 检查是否完全匹配
            if (pat_idx == pattern_len) {
                result.matched = 1; 
                result.total_gap = total_gap;
                result.end = (uint8_t)i; // 记录最后匹配位置的行号
                
                // (pattern_len - 1) 可能会溢出如果 pattern_len 是 0,
                // 但我们已在开头检查过 pattern_len > 0, 所以这里是安全的。
                uint16_t max_possible_gap = (uint16_t)(pattern_len - 1) * max_gap; // 标准化
                
                if (max_possible_gap == 0) {
                    result.confidence = (total_gap == 0) ? 1.0f : 0.0f;
                } else {
                    // 强制类型转换为 float 以进行浮点数除法
                    result.confidence = 1.0f - (float)total_gap / (float)max_possible_gap;
                }
                return result; 
            }
        } else {
            // 5. 不匹配
            if (pat_idx > 0) {
                // 计入间隔
                current_gap++;
                
                if (current_gap > max_gap) {
                    // 间隔超限, 重置状态
                    pat_idx = 0;
                    current_gap = 0;
                    total_gap = 0;

                    // 检查当前这个 input[i] 是否是 pattern[0]
                    if (input[i] == pattern[0]) {
                        pat_idx = 1;
                    }
                }
            }
            // else (pat_idx == 0), 继续寻找 pattern[0]
        }
    }

    // 循环结束仍未匹配
    return result;
}

// 创建匹配序列（用于元素识别）
//  注意：这些序列值是dir_l/dir_r的记录值，不是实际生长方向 但是后续判断就用这个
// 实际生长方向 = seeds[(记录值+1) & 7]
growth_array arr = {
    .outer_up = {1,1,1,3,3,3},       
    .inner_up = {5,5,5,4,4,4},       
    .up_outer = {4,4,4,1,1,1},       
    .up_inner = {3,3,3,5,5,5},       
    .up_outerdownarc = {4,4,1,1,2,3,3,3}, 
    .outer_uparc = {2,3,3,3,3,3,3,4} 
};

/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8_t begin, uint8_t end, uint8_t *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16_t i = 0;
	float result = 0;
	static float resultlast=0.0f;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8_t start, uint8_t end, uint8_t *border, float *slope_rate, float *intercept)
{
	uint16_t i, num = 0;
	uint16_t xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)xsum / (float)num;
		y_average = (float)ysum / (float)num;

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

/** 
* @brief 十字补线函数
* @param uint8(*image)[image_w]		输入二值图像
* @param uint8 *l_border			输入左边界首地址
* @param uint8 *r_border			输入右边界首地址
* @param uint16 total_num_l			输入左边循环总次数
* @param uint16 total_num_r			输入右边循环总次数
* @param uint16 *dir_l				输入左边生长方向首地址
* @param uint16 *dir_r				输入右边生长方向首地址
* @param uint16(*points_l)[2]		输入左边轮廓首地址
* @param uint16(*points_r)[2]		输入右边轮廓首地址
*  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8_t(*image)[image_w], uint8_t *l_border, uint8_t *r_border, uint16_t total_num_l, uint16_t total_num_r,
										 uint16_t *dir_l, uint16_t *dir_r, uint16_t(*points_l)[2], uint16_t(*points_r)[2])
{
	uint16_t i;
	uint8_t break_num_l = 0;
	uint8_t break_num_r = 0;
	uint8_t start, end;
	float slope_l_rate = 0, intercept_l = 0;
	
	// 左边匹配检测
	match_result result_l = match_strict_sequence_with_gaps(dir_l, total_num_l, arr.up_inner, 6, 3);
	if (result_l.matched)
	{
		break_num_l = result_l.end; // 使用匹配结束位置
		break_num_l = points_l[break_num_l][1]; // 转换为y坐标
	}
	
	// 右边匹配检测
	match_result result_r = match_strict_sequence_with_gaps(dir_r, total_num_r, arr.up_inner, 6, 3);
	if (result_r.matched)
	{
		break_num_r = result_r.end; // 使用匹配结束位置
		break_num_r = points_r[break_num_r][1]; // 转换为y坐标
	}

	if (result_l.matched && result_r.matched) // 两边生长方向都符合条件
	{
		for(i=0;(i<break_num_l)||(i<break_num_r);i++)
		{
			// 左边补线
			if(i<break_num_l)
			{
				l_border[i] = l_border[break_num_l+5];
			}
			// 右边补线
			if(i<break_num_r)
			{
				r_border[i] = r_border[break_num_r+5];
			}
		}



	}

}

/*
日志记录函数 日志统一写在这里
*/
void userlog()
{
	log_add_uint16_array("dir_l", dir_l, data_stastics_l,-1);
	log_add_uint16_array("dir_r", dir_r, data_stastics_r,-1);
	log_add_uint8_array("right_lost", right_lost, image_h,-1);
	log_add_uint8_array("left_lost", left_lost, image_h,-1);
	log_add_uint8_array("l_border", l_border, image_h,-1);
	log_add_uint8_array("r_border", r_border, image_h,-1);

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
//image_draw_rectan(imo);//填黑框
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
    cross_fill(imo, l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);//十字补线
}
    //求中线
	for (i = Hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
	}
    //显示边线
	draw_edge();

	userlog();
}


