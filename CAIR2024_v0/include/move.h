#ifndef MOV_H
#define MOV_H

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <Eigen/Dense>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "tasks.h"

using namespace UNITREE_LEGGED_SDK;

extern Task task[6];

double degreesToRadians(double degrees);

class Custom
{
public:
	Custom(uint8_t level);
	void Start();
	void RobotControl();
	void UDPRecv();
	void UDPSend();

	void setVelocity(float vx, float vy, float vr);
	void cmdReset();

	void rotateLeft(float angle);
	void forwardWalkNew(float distance, float speed);
	void still();
	void sitDown();
	void standUp();
	void putLeft();
	void putRight();
	void moveRight(float vy, int s);
	

	Safety safe;
	UDP udp;
	HighCmd cmd = {0};
	HighState state = {0};
	int motiontime = 0;
	// float dt = 0.002; // 0.001~0.01
	float dt = 0.01; // 0.001~0.01

	float vx = 0;
	float vy = 0;
	float vr = 0;

	const float vx_max = 0.2, vx_min = -0.2;
	const float vy_max = 0.2, vy_min = -0.2;
	const float vr_max = 1, vr_min = -1;
};

class PIDController
{
	double Kp, Ki, Kd;
	double integral = 0.0;
	double previous_error = 0.0;

public:
	PIDController(double Kp, double Ki, double Kd);
	float PT(float setpoint, float measured_value, float tolerance);
	float PIT(float setpoint, float measured_value, float tolerance);
};

// 卡尔曼滤波器类
class KalmanFilter
{
public:
	KalmanFilter(float processNoise, float measurementNoise, float estimationError);
	void predict(float u);
	void update(float z);
	float getPosition();
	float getVelocity();

private:
	Eigen::Matrix<float, 2, 2> A; // 状态转移矩阵
	Eigen::Matrix<float, 2, 1> B; // 控制输入矩阵
	Eigen::Matrix<float, 1, 2> H; // 测量矩阵
	Eigen::Matrix<float, 2, 2> Q; // 过程噪声协方差矩阵
	Eigen::Matrix<float, 1, 1> R; // 测量噪声协方差矩阵
	Eigen::Matrix<float, 2, 2> P; // 估计误差协方差矩阵
	Eigen::Matrix<float, 2, 2> I; // 单位矩阵
	Eigen::Matrix<float, 2, 1> x; // 状态向量
};

#endif