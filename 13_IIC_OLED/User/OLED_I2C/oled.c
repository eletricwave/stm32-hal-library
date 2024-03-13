#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "i2c.h"

uint8_t OLED_GRAM[128][8];

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0 + y, OLED_CMD);
	OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD); // 设置列高地址
	OLED_WR_Byte(x & 0x0f, OLED_CMD);				  // 设置列低地址
}

//反显函数
void OLED_ColorTurn(uint8_t i)
{
	if (i == 0)
	{
		OLED_WR_Byte(0xA6, OLED_CMD); //正常显示
	}
	if (i == 1)
	{
		OLED_WR_Byte(0xA7, OLED_CMD); //反色显示
	}
}

//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i)
{
	if (i == 0)
	{
		OLED_WR_Byte(0xD3, OLED_CMD); /*set display offset*/
		OLED_WR_Byte(0x60, OLED_CMD);
		OLED_WR_Byte(0xC0, OLED_CMD); //反转显示
		OLED_WR_Byte(0xA0, OLED_CMD);
	}
	if (i == 1)
	{
		OLED_WR_Byte(0xD3, OLED_CMD); /*set display offset*/
		OLED_WR_Byte(0x20, OLED_CMD);
		OLED_WR_Byte(0xC8, OLED_CMD); //正常显示
		OLED_WR_Byte(0xA1, OLED_CMD);
	}
}

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
	if (cmd == OLED_DATA)
		HAL_I2C_Mem_Write(&hi2c1, OLEDAddress, 0x40, I2C_MEMADD_SIZE_8BIT, &dat, 1, 1000);
	else
		HAL_I2C_Mem_Write(&hi2c1, OLEDAddress, 0x00, I2C_MEMADD_SIZE_8BIT, &dat, 1, 1000);
}

//开启OLED显示
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD); //电荷泵使能
	OLED_WR_Byte(0x14, OLED_CMD); //开启电荷泵
	OLED_WR_Byte(0xAF, OLED_CMD); //点亮屏幕
}

//关闭OLED显示
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD); //电荷泵使能
	OLED_WR_Byte(0x10, OLED_CMD); //关闭电荷泵
	OLED_WR_Byte(0xAE, OLED_CMD); //关闭屏幕
}

//更新显存到OLED
void OLED_Refresh(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置行起始地址
		OLED_WR_Byte(0x00, OLED_CMD);	  //设置低列起始地址
		OLED_WR_Byte(0x10, OLED_CMD);	  //设置高列起始地址
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
	}
}
//清屏函数
void OLED_Clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		for (n = 0; n < 128; n++)
		{
			OLED_GRAM[n][i] = 0; //清除所有数据
		}
	}
	OLED_Refresh(); //更新显示
}

//画点
//x:0~127
//y:0~63
//t:1 填充 0,清空
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
	uint8_t pos, bx, temp = 0;

	if (x > 127 || y > 63)
		return; //超出范围了.
	pos = 7 - y / 8;
	bx = y % 8;
	temp = 1 << (7 - bx);
	if (t)
		OLED_GRAM[x][pos] |= temp;
	else
		OLED_GRAM[x][pos] &= ~temp;
}

//画线
//x1,y1:起点坐标
//x2,y2:结束坐标
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
	uint16_t t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; //计算坐标增量
	delta_y = y2 - y1;
	uRow = x1; //画线起点坐标
	uCol = y1;
	if (delta_x > 0)
		incx = 1; //设置单步方向
	else if (delta_x == 0)
		incx = 0; //垂直线
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0)
		incy = 1;
	else if (delta_y == 0)
		incy = 0; //水平线
	else
	{
		incy = -1;
		delta_y = -delta_x;
	}
	if (delta_x > delta_y)
		distance = delta_x; //选取基本增量坐标轴
	else
		distance = delta_y;
	for (t = 0; t < distance + 1; t++)
	{
		OLED_DrawPoint(uRow, uCol, mode); //画点
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}
//x,y:圆心坐标
//r:圆的半径
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
	int a, b, num;
	a = 0;
	b = r;
	while (2 * b * b >= r * r)
	{
		OLED_DrawPoint(x + a, y - b, 1);
		OLED_DrawPoint(x - a, y - b, 1);
		OLED_DrawPoint(x - a, y + b, 1);
		OLED_DrawPoint(x + a, y + b, 1);

		OLED_DrawPoint(x + b, y + a, 1);
		OLED_DrawPoint(x + b, y - a, 1);
		OLED_DrawPoint(x - b, y - a, 1);
		OLED_DrawPoint(x - b, y + a, 1);

		a++;
		num = (a * a + b * b) - r * r; //计算画的点离圆心的距离
		if (num > 0)
		{
			b--;
			a--;
		}
	}
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//size1:选择字体 6x8/6x12/8x16/12x24
//mode:0,反色显示;1,正常显示
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
	uint8_t i, m, temp, size2, chr1;
	uint8_t x0 = x, y0 = y;
	if (size1 == 8)
		size2 = 6;
	else
		size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2); //得到字体一个字符对应点阵集所占的字节数
	chr1 = chr - ' ';											   //计算偏移后的值
	for (i = 0; i < size2; i++)
	{
		if (size1 == 8)
		{
			temp = asc2_0806[chr1][i];
		} //调用0806字体
		else if (size1 == 12)
		{
			temp = asc2_1206[chr1][i];
		} //调用1206字体
		else if (size1 == 16)
		{
			temp = asc2_1608[chr1][i];
		} //调用1608字体
		else if (size1 == 24)
		{
			temp = asc2_2412[chr1][i];
		} //调用2412字体
		else
			return;
		for (m = 0; m < 8; m++)
		{
			if (temp & 0x01)
				OLED_DrawPoint(x, y, mode);
			else
				OLED_DrawPoint(x, y, !mode);
			temp >>= 1;
			y++;
		}
		x++;
		if ((size1 != 8) && ((x - x0) == size1 / 2))
		{
			x = x0;
			y0 = y0 + 8;
		}
		y = y0;
	}
}

//显示字符串
//x,y:起点坐标
//size1:字体大小
//*chr:字符串起始地址
//mode:0,反色显示;1,正常显示
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
	while ((*chr >= ' ') && (*chr <= '~')) //判断是不是非法字符!
	{
		OLED_ShowChar(x, y, *chr, size1, mode);
		if (size1 == 8)
			x += 6;
		else
			x += size1 / 2;
		chr++;
	}
}

//m^n
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
	{
		result *= m;
	}
	return result;
}

//显示数字
//x,y :起点坐标
//num :要显示的数字
//len :数字的位数
//size:字体大小
//mode:0,反色显示;1,正常显示
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
	uint8_t t, temp, m = 0;
	if (size1 == 8)
		m = 2;
	for (t = 0; t < len; t++)
	{
		temp = (num / OLED_Pow(10, len - t - 1)) % 10;
		if (temp == 0)
		{
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
		}
		else
		{
			OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
		}
	}
}

//显示一个8x8汉字
void OLED_ShowOneChinese8(uint8_t x, uint8_t y, uint8_t *str, uint8_t mode)
{
	uint8_t m, temp;
	uint8_t x0 = x, y0 = y;

	uint16_t HZnum = sizeof(tfont08) / sizeof(FNT_GB08_S);

	for (uint8_t k = 0; k < HZnum; k++)
	{
		if ((tfont08[k].Index[0] == *(str)) && (tfont08[k].Index[1] == *(str + 1)))
		{
			for (uint8_t i = 0; i < 8; i++)
			{
				temp = tfont08[k].buff[i];
				for (m = 0; m < 8; m++)
				{
					if (temp & 0x01)
						OLED_DrawPoint(x, y, mode);
					else
						OLED_DrawPoint(x, y, !mode);
					temp >>= 1;
					y++;
				}
				x++;
				if ((x - x0) == 16)
				{
					x = x0;
					y0 = y0 + 8;
				}
				y = y0;
			}
		}
	}
}

//显示一个16x16汉字
void OLED_ShowOneChinese16(uint8_t x, uint8_t y, uint8_t *str, uint8_t mode)
{
	uint8_t m, temp;
	uint8_t x0 = x, y0 = y;

	uint16_t HZnum = sizeof(tfont16) / sizeof(FNT_GB16_S);

	for (uint8_t k = 0; k < HZnum; k++)
	{
		if ((tfont16[k].Index[0] == *(str)) && (tfont16[k].Index[1] == *(str + 1)))
		{
			for (uint8_t i = 0; i < 32; i++)
			{
				temp = tfont16[k].buff[i];
				for (m = 0; m < 8; m++)
				{
					if (temp & 0x01)
						OLED_DrawPoint(x, y, mode);
					else
						OLED_DrawPoint(x, y, !mode);
					temp >>= 1;
					y++;
				}
				x++;
				if ((x - x0) == 16)
				{
					x = x0;
					y0 = y0 + 8;
				}
				y = y0;
			}
		}
	}
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t *str, uint8_t size, uint8_t mode)
{
	while (*str != 0)
	{
		if (x > (X_MAX - size) || y > (Y_MAX - size))
			return;
		OLED_ShowOneChinese16(x, y, str, mode);
		str += 2;
		x += size; //下一个汉字偏移
	}
}

//num 显示汉字的个数
//space 每一遍显示的间隔
//mode:0,反色显示;1,正常显示
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
	uint8_t i, n, t = 0, m = 0, r;
	while (1)
	{
		if (m == 0)
		{
			//	    OLED_ShowChinese(64,56,t,16,mode); //写入一个汉字保存在OLED_GRAM[][]数组中
			t++;
		}
		if (t == num)
		{
			for (r = 0; r < 16 * space; r++) //显示间隔
			{
				for (i = 1; i < 80; i++)
				{
					for (n = 0; n < 16; n++)
					{
						OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
					}
				}
				OLED_Refresh();
			}
			t = 0;
		}
		m++;
		if (m == 16)
		{
			m = 0;
		}
		for (i = 1; i < 80; i++) //实现左移
		{
			for (n = 0; n < 16; n++)
			{
				OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
	}
}

//x,y：起点坐标
//sizex,sizey,图片长宽
//BMP[]：要写入的图片数组
//mode:0,反色显示;1,正常显示
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
	uint16_t j = 0;
	uint8_t i, n, temp, m;
	uint8_t x0 = x, y0 = y;
	sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
	for (n = 0; n < sizey; n++)
	{
		for (i = 0; i < sizex; i++)
		{
			temp = BMP[j];
			j++;
			for (m = 0; m < 8; m++)
			{
				if (temp & 0x01)
					OLED_DrawPoint(x, y, mode);
				else
					OLED_DrawPoint(x, y, !mode);
				temp >>= 1;
				y++;
			}
			x++;
			if ((x - x0) == sizex)
			{
				x = x0;
				y0 = y0 + 8;
			}
			y = y0;
		}
	}
}
//OLED的初始化
void OLED_Init(void)
{
	OLED_WR_Byte(0xAE, OLED_CMD); //关闭显示
	OLED_WR_Byte(0x02, OLED_CMD); //---set low column address
	OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
	OLED_WR_Byte(0x40, OLED_CMD); //设置显示开始行[5:0](0x00~0x3F)
	OLED_WR_Byte(0x81, OLED_CMD); //对比度设置
	OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC0, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
	OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
	OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00, OLED_CMD); //-not offset
	OLED_WR_Byte(0xd5, OLED_CMD); //设置时钟分频因子，震荡频率
	OLED_WR_Byte(0x80, OLED_CMD); //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
	OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
	OLED_WR_Byte(0x12, OLED_CMD);
	OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
	OLED_WR_Byte(0x40, OLED_CMD); //Set VCOM Deselect Level
	OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02, OLED_CMD); //
	OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
	OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
	OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
	OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel

	OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
	OLED_Clear();
	OLED_Set_Pos(0, 0);
}
