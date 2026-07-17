/*
 * can_imu_mpu9250.h
 *
 *  Created on: 2016/11/07
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_

#include "CAN/can_device.h"
#include "sensors/imu/imu_basic.h"

class CANIMU : public CANDevice, public IMU {
private:
	void updateRawData() override;
	int16_t r_gyro_data[3], r_acc_data[3], r_mag_data[3];
	uint32_t last_gyro_receive_time_, last_acc_receive_time_, last_mag_receive_time_;
	bool send_data_flag_;
	bool gyro_received_, acc_received_, mag_received_;
public:
	CANIMU():
		CANDevice(),
		IMU(),
		last_gyro_receive_time_(0),
		last_acc_receive_time_(0),
		last_mag_receive_time_(0),
		send_data_flag_(false),
		gyro_received_(false),
		acc_received_(false),
		mag_received_(false)
	{}
	CANIMU(uint8_t slave_id, bool send_data_flag) :
		CANDevice(CAN::DEVICEID_IMU, slave_id),
		IMU(),
		last_gyro_receive_time_(0),
		last_acc_receive_time_(0),
		last_mag_receive_time_(0),
		send_data_flag_(send_data_flag),
		gyro_received_(false),
		acc_received_(false),
		mag_received_(false)
	{}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	bool getSendDataFlag() const { return send_data_flag_;}
	void setSendDataFlag(bool send_data_flag) {send_data_flag_ = send_data_flag;}
	bool hasGyroData() const { return gyro_received_; }
	bool hasAccData() const { return acc_received_; }
	bool hasMagData() const { return mag_received_; }
	bool hasData() const { return gyro_received_ && acc_received_ && mag_received_;}
	uint32_t getLastGyroReceiveTime() const { return last_gyro_receive_time_; }
	uint32_t getLastAccReceiveTime() const { return last_acc_receive_time_; }
	uint32_t getLastMagReceiveTime() const { return last_mag_receive_time_; }
	int16_t getRawGyroData(uint8_t axis) const { return r_gyro_data[axis]; }
	int16_t getRawAccData(uint8_t axis) const { return r_acc_data[axis]; }
	int16_t getRawMagData(uint8_t axis) const { return r_mag_data[axis]; }

	static constexpr float GYRO_SCALE = 2000.0f / 32767.0f * M_PI / 180.0f;
	static constexpr float ACC_SCALE = GRAVITY_MSS / 4096.0f;
	static constexpr float MAG_SCALE = 4912.0f / 32760.0f;
};




#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_ */
