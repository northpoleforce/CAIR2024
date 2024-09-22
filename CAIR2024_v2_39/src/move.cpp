/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "move.h"

Custom::Custom(uint8_t level) : safe(LeggedType::Go1),
								udp(level, 8090, "192.168.123.161", 8082)
{
	std::cout << "Communication level is set to HIGH-level." << std::endl;
	udp.InitCmdData(cmd);
}

void Custom::Start()
{
	LoopFunc loop_control("control_loop", dt, boost::bind(&Custom::RobotControl, this));
	LoopFunc loop_udpSend("udp_send", dt, 3, boost::bind(&Custom::UDPSend, this));
	LoopFunc loop_udpRecv("udp_recv", dt, 3, boost::bind(&Custom::UDPRecv, this));
	loop_control.start();
	loop_udpSend.start();
	loop_udpRecv.start();
	while (1)
	{
		sleep(10);
	};
}

void Custom::RobotControl()
{
	udp.GetRecv(state);
	udp.SetSend(cmd);
}
void Custom::UDPRecv()
{
	udp.Recv();
}
void Custom::UDPSend()
{
	udp.Send();
}

void Custom::putLeft()
{
	cmdReset();
	cmd.mode = 1;
	cmd.euler[1] = -0.75;
	sleep(1);
	cmd.euler[1] = 0.0;
	sleep(1);
	cmd.bodyHeight = -0.2;
	sleep(1);
	cmd.euler[0] = -0.75;
	sleep(1);
	cmd.euler[0] = 0;
	sleep(1);
	cmd.bodyHeight = 0;
	sleep(1);
}
void Custom::putRight()
{
	cmdReset();
	cmd.mode = 1;
	cmd.euler[1] = +0.75;
	sleep(1);
	cmd.euler[1] = 0.0;
	sleep(1);
	cmd.bodyHeight = -0.2;
	sleep(1);
	cmd.euler[0] = +0.75;
	sleep(1);
	cmd.euler[0] = 0;
	sleep(1);
	cmd.bodyHeight = 0;
	sleep(1);
}
void Custom::sitDown()
{
	cmdReset();
	cmd.mode = 1;
	sleep(1);
	cmd.mode = 6;
	sleep(1);
	cmd.mode = 5;
	sleep(1);
	cmd.mode = 7;
	sleep(1);
}
void Custom::standUp()
{
	cmdReset();
	cmd.mode = 5;
	sleep(1);
	cmd.mode = 6;
	sleep(1);
	cmd.mode = 1;
	sleep(1);
}

void Custom::setVelocity(float vx, float vy, float vr)
{
	cmdReset();
	cmd.mode = 2;
	cmd.gaitType = 1;
	cmd.velocity[0] = vx;
	cmd.velocity[1] = vy;
	cmd.yawSpeed = vr;
	cmd.velocity[0] = std::max(vx_min, std::min(vx_max, cmd.velocity[0]));
	cmd.velocity[1] = std::max(vy_min, std::min(vy_max, cmd.velocity[1]));
	cmd.yawSpeed = std::max(vr_min, std::min(vr_max, cmd.yawSpeed));
	std::cout << "vx: " << vx << " vy: " << vy << " vr: " << vr << std::endl;
}
void Custom::cmdReset()
{
	cmd.mode = 0;
	cmd.gaitType = 0;
	cmd.speedLevel = 0;
	cmd.footRaiseHeight = 0;
	cmd.bodyHeight = 0;
	cmd.euler[0] = 0;
	cmd.euler[1] = 0;
	cmd.euler[2] = 0;
	cmd.velocity[0] = 0.0f;
	cmd.velocity[1] = 0.0f;
	cmd.yawSpeed = 0.0f;
	cmd.reserve = 0;
}

void Custom::still()
{
	cmdReset();
	sleep(1);
}

double degreesToRadians(double degrees)
{
	return degrees * M_PI / 180.0;
}

void Custom::rotateLeft(float angle)
{
	cmdReset();
	// 角度转换为弧度
	float angleRad = degreesToRadians(angle);
	float initialYaw = state.imu.rpy[2];
	float targetYaw = initialYaw + angleRad;
	PIDController pidYaw(1, 0, 0);
	const float updateInterval = 0.01; // 更新间隔（秒）
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		// udp.GetRecv(state);
		float currentYaw = state.imu.rpy[2];
		// Normalize currentYaw and targetYaw to the range [-pi, pi]
		float yawDifference = targetYaw - currentYaw;
		while (yawDifference > M_PI)
			yawDifference -= 2 * M_PI;
		while (yawDifference < -M_PI)
			yawDifference += 2 * M_PI;
		// Normalize currentYaw to the range [-pi, pi]
		while (currentYaw > M_PI)
			currentYaw -= 2 * M_PI;
		while (currentYaw < -M_PI)
			currentYaw += 2 * M_PI;
		float tolerance = 0.1;
		if (std::abs(yawDifference) <= tolerance) // 停止条件
		{
			std::cout << "Initial Yaw: " << initialYaw << std::endl;
			std::cout << "Final Yaw: " << currentYaw << std::endl;
			break;
		}
		double speedYaw = pidYaw.PT(targetYaw, currentYaw, tolerance);
		if (angle < 0)
			setVelocity(0, 0, speedYaw);
		else
			setVelocity(0.04, 0, speedYaw);
	}
	cmdReset();
}

void Custom::forwardWalkNew(float distance, float speed)
{
	cmdReset();
	double tolerance = 0.05;
	double minus = 1;
	if (distance < 0)
		minus = -1;
	distance = abs(distance);
	float initialPositionX = state.position[0];
	float initialPositionY = state.position[1];
	float initialYaw = state.imu.rpy[2]; // 获取初始朝向角度
	const float updateInterval = 0.01;	 // 更新间隔（秒）
	KalmanFilter kfX(0.1, 0.1, 1.0);	 // 初始化X方向卡尔曼滤波器
	KalmanFilter kfY(0.1, 0.1, 1.0);	 // 初始化Y方向卡尔曼滤波器
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(updateInterval * 1000)));
		udp.GetRecv(state);
		float currentPositionX = state.position[0];
		float currentPositionY = state.position[1];
		float accelX = state.imu.accelerometer[0];
		float accelY = state.imu.accelerometer[1];
		float currentYaw = state.imu.rpy[2];  // 获取当前朝向角度
		kfX.predict(accelX * updateInterval); // 预测步骤X方向
		kfX.update(currentPositionX);		  // 更新步骤X方向
		kfY.predict(accelY * updateInterval); // 预测步骤Y方向
		kfY.update(currentPositionY);		  // 更新步骤Y方向
		float filteredPositionX = kfX.getPosition();
		float filteredPositionY = kfY.getPosition();
		std::cout << "Filtered Position: X=" << filteredPositionX << " Y=" << filteredPositionY << std::endl;
		// 计算当前相对于初始位置和朝向的位移
		float deltaX = filteredPositionX - initialPositionX;
		float deltaY = filteredPositionY - initialPositionY;
		// 将位移转换到初始坐标系中
		float relativeX = deltaX * cos(initialYaw) + deltaY * sin(initialYaw);
		float relativeY = deltaY * cos(initialYaw) - deltaX * sin(initialYaw);
		// 计算与目标位置的距离
		float distanceTraveled = sqrt(pow(relativeX, 2) + pow(relativeY, 2));
		std::cout << "distanceTraveled: " << distanceTraveled << std::endl;
		std::cout << "distance: " << distance << std::endl;
		// 检查是否达到目标距离
		if (std::abs(distance - distanceTraveled) <= tolerance)
		{
			std::cout << "Final Position: X=" << state.position[0] << " Y=" << state.position[1] << " Z=" << state.position[2] << std::endl;
			break;
		}
		// 保持X方向速度
		double speedX = speed * minus;
		setVelocity(speedX, 0, 0);
	}
	cmdReset();
}

void Custom::moveRight(float vy, int s)
{
	cmdReset();
	setVelocity(0, -vy, 0);
	sleep(s);
	cmdReset();
}


PIDController::PIDController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
float PIDController::PT(float setpoint, float measured_value, float tolerance)
{
	float error = setpoint - measured_value;
	if (abs(error) < tolerance)
	{
		return 0;
	}
	return Kp * error;
}
float PIDController::PIT(float setpoint, float measured_value, float tolerance)
{
	float error = setpoint - measured_value;
	integral += error;
	if (abs(error) < tolerance)
	{
		return 0;
	}
	return Kp * error + Ki * integral;
}


KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimationError)
{
	A << 1, 1,
		0, 1;	 // 状态转移矩阵
	B << 0.5, 1; // 控制输入矩阵
	H << 1, 0;	 // 测量矩阵
	Q << processNoise, 0,
		0, processNoise;   // 过程噪声协方差矩阵
	R << measurementNoise; // 测量噪声协方差矩阵
	P << estimationError, 0,
		0, estimationError; // 估计误差协方差矩阵
	I.setIdentity();		// 单位矩阵
	x.setZero();			// 状态向量初始化为零
}
void KalmanFilter::predict(float u)
{
	x = A * x + B * u;
	P = A * P * A.transpose() + Q;
}
void KalmanFilter::update(float z)
{
	Eigen::Matrix<float, 1, 1> z_matrix;
	z_matrix(0, 0) = z;
	Eigen::Matrix<float, 1, 1> y = z_matrix - H * x;				// 计算测量预测误差
	Eigen::Matrix<float, 1, 1> S = H * P * H.transpose() + R;		// 计算测量预测误差协方差
	Eigen::Matrix<float, 2, 1> K = P * H.transpose() * S.inverse(); // 计算卡尔曼增益
	x = x + K * y;													// 更新状态向量
	P = (I - K * H) * P;											// 更新估计误差协方差矩阵
}
float KalmanFilter::getPosition()
{
	return x(0); // 返回位置
}
float KalmanFilter::getVelocity()
{
	return x(1); // 返回速度
}