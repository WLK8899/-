/*
 * OLED.c
 *
 *  Created on: Jul 3, 2023
 *      Author: ???_code
 */


#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "gpio.h"

u8 OLED_GRAM[128][8];

void OLED_Refresh_Gram(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //?????(0~7)
		OLED_WR_Byte (0x00,OLED_CMD);      //??????�????
		OLED_WR_Byte (0x10,OLED_CMD);      //??????�????
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

//?OLED???????
//dat:??????/??
//cmd:??/???? 0,????;1,????;
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	u8 i;
	if(cmd)
	  OLED_RS_Set();
	else
	  OLED_RS_Clr();
	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;
	}
	OLED_RS_Set();
}


//??OLED??
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC??
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//??OLED??
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC??
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//????,???,????????!??????!!!
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;
	OLED_Refresh_Gram();//????
}
//??
//x:0~127
//y:0~63
//t:1 ?? 0,??
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//?????.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}

//???????????,??????
//x:0~127
//y:0~63
//mode:0,????;1,????
//size:???? 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//???????
    for(t=0;t<size;t++)
    {
		if(size==12)temp=oled_asc2_1206[chr][t];  //??1206??
		else temp=oled_asc2_1608[chr][t];		 //??1608??
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
}

//m^n??
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}

//??2???
//x,y :????
//len :?????
//size:????
//mode:??	0,????;1,????
//num:??(0~4294967295);
void OLED_ShowNumber(u8 x, u8 y, int num)
{
    char buffer[20];  // ??????????????????????
    snprintf(buffer, sizeof(buffer), "%d", num);  // ?????????
    
    u8 length = strlen(buffer);  // ????????
    for(u8 t = 0; t < length; t++)
    {
        OLED_ShowChar(x + (8 * t), y, buffer[t], 16, 1);  // ??????????????8??
    }
}

//?????
//x,y:????
//*p:???????
//?16??
void OLED_ShowString(u8 x, u8 y, const u8 *p)
{
#define MAX_CHAR_POSX 124  // ????X??
#define MAX_CHAR_POSY 56   // ????Y??

    while (*p != '\0')
    {
        if (x > MAX_CHAR_POSX)
        {
            x = 0;
            y += 16;
        }
        if (y > MAX_CHAR_POSY)
        {
            y = 0;
            x = 0;
            OLED_Clear();
        }
        OLED_ShowChar(x, y, *p, 12, 1);
        x += 8;  // ????
        p++;
    }
}

//??????
//x,y :????
//value :?????
//decimalPlaces,??????,
// ???????
void OLED_ShowFloat(float value, uint8_t decimalPlaces, uint8_t x, uint8_t y)
{
    char buffer[16];

    // ????????????,???????
    snprintf(buffer, sizeof(buffer), "%.*f", decimalPlaces, value);

    // ??????????
    for (uint8_t i = 0; i < strlen(buffer); i++)
    {
        OLED_ShowChar(x + i * 8, y, buffer[i], 16, 1);
    }
}

//???OLED
void OLED_Init(void)
{
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //????
	OLED_WR_Byte(0xD5,OLED_CMD); //????????,????
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],????;[7:4],????
	OLED_WR_Byte(0xA8,OLED_CMD); //??????
	OLED_WR_Byte(0X3F,OLED_CMD); //??0X3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //??????
	OLED_WR_Byte(0X00,OLED_CMD); //???0

	OLED_WR_Byte(0x40,OLED_CMD); //??????? [5:0],??.

	OLED_WR_Byte(0x8D,OLED_CMD); //?????
	OLED_WR_Byte(0x14,OLED_CMD); //bit2,??/??
	OLED_WR_Byte(0x20,OLED_CMD); //????????
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00,?????;01,?????;10,?????;??10;
	OLED_WR_Byte(0xA1,OLED_CMD); //??????,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //??COM????;bit3:0,????;1,????? COM[N-1]->COM0;N:????
	OLED_WR_Byte(0xDA,OLED_CMD); //??COM??????
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]??

	OLED_WR_Byte(0x81,OLED_CMD); //?????
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;??0X7F (????,????)
	OLED_WR_Byte(0xD9,OLED_CMD); //???????
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //??VCOMH ????
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //??????;bit0:1,??;0,??;(??/??)
	OLED_WR_Byte(0xA6,OLED_CMD); //??????;bit0:1,????;0,????
	OLED_WR_Byte(0xAF,OLED_CMD); //????
	OLED_Clear();
}
