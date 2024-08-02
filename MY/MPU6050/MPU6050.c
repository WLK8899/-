#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "oled.h"
#define DEFAULT_MPU_HZ  (100)
#define Q30				(1073741824.0f)
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }else
	{
		return -1;
	}
	return 0;
}

int MPU6050_DMP_Init(void)
{
	int result;
	struct int_param_s int_param;
	result = mpu_init(&int_param);	//mpu初始化
	if(result != 0)
	{
		return -1;
	}
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); //设置传感器
	if(result != 0)
	{
		return -2;
	}
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置fifo
	if(result != 0)
	{
		return -3;
	}
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);//设置采样率
	if(result != 0)
	{
		return -4;
	}
	result = dmp_load_motion_driver_firmware(); //加载dmp固件
	if(result != 0)
	{
		return -5;
	}
    result = dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
	if(result != 0)
	{
		return -6;
	}
    result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
								DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
								DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);//设置dmp功能
	if(result != 0)
	{
		return -7;
	}
    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);//设置输出速率
	if(result != 0)
	{
		return -8;
	}
	result = run_self_test();//自检
	if(result != 0)
	{
		return -9;
	}
    result = mpu_set_dmp_state(1);//使能dmp
	if(result != 0)
	{
		return -10;
	}
	return 0;

}

// 全局变量
float previous_pitch = 0.0f;
float previous_roll = 0.0f;
float previous_yaw = 0.0f;
const float alpha = 0.9f; // 低通滤波系数，0到1之间，越接近1滤波效果越强

#define ALPHA 0.2  // 调整 ALPHA 参数以实现不同的滤波效果
long quat_offset[4] = {0, 0, 0, 0}; // 四元数零偏值
// 低通滤波器函数
long low_pass_filter(long current_value, long previous_value) {
    return (long)(ALPHA * current_value + (1.0 - ALPHA) * previous_value);
}

// 四元数零点校准函数
void calibrate_quaternion_offset() {
    long quat[4];
    int num_samples = 1000;
    long quat_sum[4] = {0, 0, 0, 0};
    long filtered_quat[4] = {0, 0, 0, 0};

    for (int i = 0; i < num_samples; i++) {
        if (dmp_read_fifo(NULL, NULL, quat, NULL, NULL, NULL) == 0) {
            // 对四元数数据进行滤波处理
            filtered_quat[0] = low_pass_filter(quat[0], filtered_quat[0]);
            filtered_quat[1] = low_pass_filter(quat[1], filtered_quat[1]);
            filtered_quat[2] = low_pass_filter(quat[2], filtered_quat[2]);
            filtered_quat[3] = low_pass_filter(quat[3], filtered_quat[3]);

            // 累加滤波后的四元数数据
            quat_sum[0] += filtered_quat[0];
            quat_sum[1] += filtered_quat[1];
            quat_sum[2] += filtered_quat[2];
            quat_sum[3] += filtered_quat[3];
        }
        HAL_Delay(1);
    }

    // 计算平均值并作为零点偏移
    quat_offset[0] = quat_sum[0] / num_samples;
    quat_offset[1] = quat_sum[1] / num_samples;
    quat_offset[2] = quat_sum[2] / num_samples;
    quat_offset[3] = quat_sum[3] / num_samples;
}

int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw) {
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) {
        return -1;
    }

    if (sensors & INV_WXYZ_QUAT) {
        // 应用四元数零偏
        quat[0] -= quat_offset[0];
        quat[1] -= quat_offset[1];
        quat[2] -= quat_offset[2];
        quat[3] -= quat_offset[3];

        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        float raw_pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
        float raw_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
        float raw_yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;

        // 低通滤波
        *pitch = alpha * previous_pitch + (1.0f - alpha) * raw_pitch;
        *roll = alpha * previous_roll + (1.0f - alpha) * raw_roll;
        *yaw = alpha * previous_yaw + (1.0f - alpha) * raw_yaw;

        // 更新前一次的姿态数据
        previous_pitch = *pitch;
        previous_roll = *roll;
        previous_yaw = *yaw;
    } else {
        return -2;
    }

    return 0;
}

