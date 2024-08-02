#ifndef UART_5
#define UART_5	
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>  // 添加此行以包含 memset 函数声明
#include "oled.h"
#include "system.h"
#include <math.h>
#include <stdbool.h>
#include <float.h>
#define HEADER_0 165
#define HEADER_1 90
#define Length_  58

#define POINT_PER_PACK 16
#define DATA_SIZE 120

// 全局变量
extern volatile uint8_t uart5_data_buffer[DATA_SIZE];
extern volatile uint8_t uart5_data_index;
extern volatile uint8_t data_ready_flag;
extern u32 data_cnt;
typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

typedef struct PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef Dataprocess[92];      //用于小车避障、跟随、走直线、ELE雷达避障的雷达数据
extern u8 Uart5_Receive_buf[1];          //串口5接收中断数据存放的缓冲区
extern u8 Uart5_Receive;                 //从串口5读取的数据



void processData(void);
void meanFilter(PointDataProcessDef* data, int dataSize, int windowSize, u16 maxDistance, float maxAngle);

/// @brief 特征识别//////


//笛卡尔坐标
typedef struct {
    float x;
    float y;
	float power;
} CartesianPoint;
#define MIN_RADIUS  20
#define MAX_RADIUS  150
#define WINDOW_SIZE 15
//圆
typedef struct {
    CartesianPoint center;
    float radius;
} Circle;
//聚类算法（DBSCAN）簇点
typedef struct {
    int cluster_id;
    CartesianPoint point;
} ClusteredPoint;
//极坐标，用于目标和避障
typedef struct {
    float r;
    float theta;
} PolarCoord;

int HoughArc(CartesianPoint *points, int Cnt, int r, Circle* Arc);
int dbscan(CartesianPoint *points, int num_points, ClusteredPoint *clustered_points);
void polarToCartesian(PointDataProcessDef *polar, CartesianPoint *cartesian, int num_points);
PolarCoord calculateDistanceAndAngle(CartesianPoint center);
#endif