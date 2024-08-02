#include "uart5.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

u8 Uart5_Receive_buf[1]; // 串口5接收中断数据存放的缓冲区
u8 Uart5_Receive;        // 从串口5读取的数据

LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[92]; // 用于特征识别的雷达数据
/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
u32 data_cnt = 0;
void data_process(void) // 数据处理
{
    // println("lib");
    int m;
    u32 distance_sum[16] = {0};                                                                  // 2个点的距离和的数组
    float start_angle = (((u16)Pack_Data.start_angle_h << 8) + Pack_Data.start_angle_l) / 100.0; // 计算16个点的开始角度
    float end_angle = (((u16)Pack_Data.end_angle_h << 8) + Pack_Data.end_angle_l) / 100.0;       // 计算16个点的结束角度
    float area_angle[16] = {0};
    // print ("q");
    // print_float(start_angle);
    // print_float(end_angle);
    // HAL_Delay(1000);

    if (start_angle > end_angle) // 结束角度和开始角度被0度分割的情况
        end_angle += 360;

    for (m = 0; m < 4; m++)
    {
        area_angle[m] = start_angle + (end_angle - start_angle) / 4 * m;

        if (area_angle[m] >=360)
            area_angle[m] = area_angle[m]-360;

        if (area_angle[m] > 270 || area_angle[m] < 90) // 只提取前方范围内的点,一共120度范围
        {
            distance_sum[m] += ((u16)Pack_Data.point[m].distance_h << 8) + Pack_Data.point[m].distance_l; // 数据高低8位合并

            Dataprocess[data_cnt].angle = area_angle[m];
            Dataprocess[data_cnt++].distance = distance_sum[m]; // 一帧数据为16个点
            // print_float(Dataprocess[data_cnt].angle);
            // print_int32_t(Dataprocess[data_cnt].distance);
            if (data_cnt == 90)
                data_cnt = 0;
        }
    }
}
///////////////////////////////////////////////点云数据滤波////////////////////////////
////////////////////均值滤波带距离判读//////////////////////////
// 点云数据数组，数组大小，窗口大小，窗口内距离阈值，角度阈值///////////

// 均值滤波函数
void meanFilter(PointDataProcessDef *data, int dataSize, int windowSize, u16 maxDistance, float maxAngle)
{
    // 创建一个临时数组用于存储滤波后的结果
    PointDataProcessDef *tempData = (PointDataProcessDef *)malloc(dataSize * sizeof(PointDataProcessDef));
    if (tempData == NULL)
    {
        // 处理内存分配失败的情况
        return;
    }

    // 初始化临时数组
    for (int i = 0; i < dataSize; i++)
    {
        tempData[i].distance = data[i].distance;
        tempData[i].angle = data[i].angle;
    }

    // 进行均值滤波
    for (int i = 0; i < dataSize; i++)
    {
        float sumDistance = 0;
        float sumAngle = 0;
        int count = 0;

        for (int j = i - windowSize; j <= i + windowSize; j++)
        {
            if (j >= 0 && j < dataSize && data[j].distance <= maxDistance && fabs(data[j].angle) <= maxAngle)
            {
                sumDistance += data[j].distance;
                sumAngle += data[j].angle;
                count++;
            }
        }

        if (count > 0)
        {
            tempData[i].distance = sumDistance / count;
            tempData[i].angle = sumAngle / count;
        }
    }

    // 将滤波后的结果复制回原始数组
    for (int i = 0; i < dataSize; i++)
    {
        data[i].distance = tempData[i].distance;
        data[i].angle = tempData[i].angle;
    }

    // 释放临时数组的内存
    free(tempData);
}

void processData(void)
{
    static u8 state = 0; // 状态位
    static u8 cnt = 0;   // 用于一帧16个点的计数
    u8 temp_data;

    // 处理缓冲区中的数据
    for (int i = 0; i < DATA_SIZE; i++)
    {
        temp_data = uart5_data_buffer[i];

        if (state > 6)
        {
            if (state < 55)
            {
                if (state % 3 == 1) // 序号7,10,13....52
                {
                    Pack_Data.point[cnt].distance_h = temp_data; // 16个点的距离数据，高字节
                    state++;
                }
                else if (state % 3 == 2) // 序号8,11,14...53
                {
                    Pack_Data.point[cnt].distance_l = temp_data; // 16个点的距离数据，低字节
                    state++;
                }
                else // 序号9,12,15...54
                {
                    Pack_Data.point[cnt].Strong = temp_data; // 16个点的强度数据
                    state++;
                    cnt++; // 开始下一个数据
                }
            }
            else
            {
                switch (state)
                {
                case 55:
                    Pack_Data.end_angle_h = temp_data;
                    state++;
                    break;
                case 56:
                    Pack_Data.end_angle_l = temp_data;
                    state++;
                    break;
                case 57: // 数据接收完毕
                    // 检查最后一位后的三位数据
                    // if (i + 3 < DATA_SIZE && uart5_data_buffer[i + 1] == HEADER_0 && uart5_data_buffer[i + 2] == HEADER_1 && uart5_data_buffer[i + 3] == Length_)
                    // {
                        // 一组串口数据只要一组有效帧数据
                        Pack_Data.crc = temp_data; // 这里已完成数据的接收，Pack_Data是接收到的数据
                        state = 0;
                        cnt = 0;
                        // 数据处理
                        data_process();
                        // 滤波
                        //meanFilter(Dataprocess, 110, 5, 30, 2);
                        return;
                    // }
                    // else
                    // {   
                    //     state = 0;
                    //     // 结束处理
                    //     return;
                    // }
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            switch (state)
            {
            case 0:
                if (temp_data == HEADER_0) // 头固定
                {
                    Pack_Data.header_0 = temp_data;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            case 1:
                if (temp_data == HEADER_1) // 头固定
                     
                {
                    Pack_Data.header_1 = temp_data;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            case 2:
                if (temp_data == Length_) // 字长固定
                {
                    Pack_Data.ver_len = temp_data;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            case 3:
                Pack_Data.speed_h = temp_data;
                state++;
                break;
            case 4:
                Pack_Data.speed_l = temp_data;
                state++;
                break;
            case 5:
                Pack_Data.start_angle_h = temp_data;
                state++;
                break;
            case 6:
                Pack_Data.start_angle_l = temp_data;
                state++;
                break;
            default:
                break;
            }
        }
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {

        // 如果标志位已设置，表示数据正在处理，直接返回
        if (data_ready_flag)
        {
            // 重新开启UART接收中断
            HAL_UART_Receive_IT(&huart5, Uart5_Receive_buf, sizeof(Uart5_Receive_buf));
            return;
        }

        // 将接收到的数据存入缓冲区
        uart5_data_buffer[uart5_data_index] = Uart5_Receive_buf[0];
        uart5_data_index++;

        // 如果缓冲区已满，设置标志位
        if (uart5_data_index >= DATA_SIZE)
        {
            data_ready_flag = 1;
            uart5_data_index = 0; // 重置索引以准备下一次数据接收
        }

        // 重新开启UART接收中断
        HAL_UART_Receive_IT(&huart5, Uart5_Receive_buf, sizeof(Uart5_Receive_buf));
    }
}

///////////////////////////////////////特征识别//////////////////////////////////
// 极坐标转换为直角坐标
void polarToCartesian(PointDataProcessDef *polar, CartesianPoint *cartesian, int num_points)
{
    for (int i = 0; i < num_points; ++i)
    {
        // println("nnnnnnnnnn");
        cartesian[i].x = polar[i].distance * cosf(polar[i].angle);
        cartesian[i].y = polar[i].distance * sinf(polar[i].angle);
    }
}

// DBSCAN聚类算法
#define EPSILON 0.1  // 距离阈值
#define MIN_POINTS 4 // 最小点数
int dbscan(CartesianPoint *points, int num_points, ClusteredPoint *clustered_points)
{
    int cluster_id = 0;

    for (int i = 0; i < num_points; ++i)
    {
        clustered_points[i].point = points[i];
        clustered_points[i].cluster_id = -1;
    }

    for (int i = 0; i < num_points; ++i)
    {
        if (clustered_points[i].cluster_id != -1)
            continue;

        int neighbors[num_points];
        int neighbor_count = 0;

        for (int j = 0; j < num_points; ++j)
        {
            float distance = sqrtf((points[i].x - points[j].x) * (points[i].x - points[j].x) +
                                   (points[i].y - points[j].y) * (points[i].y - points[j].y));
            if (distance < EPSILON)
            {
                neighbors[neighbor_count++] = j;
            }
        }

        if (neighbor_count < MIN_POINTS)
        {
            clustered_points[i].cluster_id = 0; // 标记为噪声
            continue;
        }

        cluster_id++;
        for (int j = 0; j < neighbor_count; ++j)
        {
            clustered_points[neighbors[j]].cluster_id = cluster_id;
        }

        for (int j = 0; j < neighbor_count; ++j)
        {
            int current_point = neighbors[j];

            int new_neighbors[num_points];
            int new_neighbor_count = 0;

            for (int k = 0; k < num_points; ++k)
            {
                float distance = sqrtf((points[current_point].x - points[k].x) * (points[current_point].x - points[k].x) +
                                       (points[current_point].y - points[k].y) * (points[current_point].y - points[k].y));
                if (distance < EPSILON)
                {
                    new_neighbors[new_neighbor_count++] = k;
                }
            }

            if (new_neighbor_count >= MIN_POINTS)
            {
                for (int k = 0; k < new_neighbor_count; ++k)
                {
                    if (clustered_points[new_neighbors[k]].cluster_id == -1 || clustered_points[new_neighbors[k]].cluster_id == 0)
                    {
                        clustered_points[new_neighbors[k]].cluster_id = cluster_id;
                    }
                }
            }
        }
    }
    return cluster_id; // 返回总共的聚类数
}


int HoughArc(CartesianPoint *points, int Cnt, int r, Circle* Arc) {
    double theta;
    int a, b;
    int minA, maxA, minB, maxB;
    double deltaTheta = PI / 180; // 间隔1度
    double startAngle = 150.0 * PI / 180;
    double endAngle = PI * 2 + PI / 6;

    minA = maxA = (int)(points[0].x - r);
    minB = maxB = (int)points[0].y; // theta = 0
    
    // 计算a，b的最小和最大值
    for (int i = 0; i < Cnt; i++) {
        
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {
            a = (int)(points[i].x - r * cos(theta) + 0.5);
            b = (int)(points[i].y - r * sin(theta) + 0.5);
            if (a > maxA) {
                maxA = a;
            } else if (a < minA) {
                minA = a;
            }

            if (b > maxB) {
                maxB = b;
            } else if (b < minB) {
                minB = b;
            }
        }
    }
    
    // 确定a，b的范围之后，即确定了票箱的大小
    int aScale = maxA - minA + 1;
    int bScale = maxB - minB + 1;

    int* VoteBox = (int*)calloc(aScale * bScale, sizeof(int));
    if (VoteBox == NULL) {
        print("begin");
        return 0; // 分配内存失败
    }

    // 开始投票
    for (int i = 0; i < Cnt; i++) {
        for (theta = startAngle; theta < endAngle; theta += deltaTheta) {
            a = (int)(points[i].x - r * cos(theta) + 0.5);
            b = (int)(points[i].y - r * sin(theta) + 0.5);
            VoteBox[(b - minB) * aScale + (a - minA)]++;
        }
    }

    // 筛选票箱
    int VoteMax = 0;
    int VoteMaxX = 0, VoteMaxY = 0;
    for (int i = 0; i < bScale; i++) {
        for (int j = 0; j < aScale; j++) {
            if (VoteBox[i * aScale + j] > VoteMax) {
                VoteMax = VoteBox[i * aScale + j];
                VoteMaxY = i;
                VoteMaxX = j;
            }
        }
    }

    // 打印投票结果
    print("Vote");print_int32_t(VoteMax);

    // 释放内存
    free(VoteBox);

    if (VoteMax > 3) {
        Arc->center.x = VoteMaxX + minA;
        Arc->center.y = VoteMaxY + minB;
        Arc->radius = r;
        return 1;
    } else {
        print("u");
        return 0;
    }
}
// 计算距离和角度
PolarCoord calculateDistanceAndAngle(CartesianPoint center)
{
    PolarCoord polarCoord;
    polarCoord.r = sqrtf(center.x * center.x + center.y * center.y);
    polarCoord.theta = atan2f(center.y, center.x);
    return polarCoord;
}
