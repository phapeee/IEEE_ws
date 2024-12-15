#include <ros/ros.h>
#include <wiringpi.h>
#include <mecanum_ik/vector4_msg.h>

#define I2C_ADD 0x24
#define MOTOR1_PWM 0x20

#define SPEED_MODE 0x02

mecanum_ik::vector4_msg motor_speeds;
mecanum_ik::vector4_msg read_motor_sp;
bool received;

void setMotorSpeed(const mecanum_ik::vector4_msg &motor_sp){
	read_motor_sp = motor_sp;
	received = true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	ros::Rate rate(60);
	ros::Subscriber name = nh.subscribe("wheel_angle_vels", 10, setMotorSpeed);
	ros::Publisher name1 = nh.advertise<mecanum_ik::vector4_msg>("real_motor_speeds");

	wiringPiSetup();
	motor_control_I2C = wiringPiI2CSetup(I2C_ADD);

	// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR1_MAX_SPEED,  <scaled max motor speed>);
	// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR2_MAX_SPEED,  <scaled max motor speed>);
	// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR3_MAX_SPEED,  <scaled max motor speed>);
	// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR4_MAX_SPEED,  <scaled max motor speed>);

	wiringPiI2CWriteReg8(motor_control_I2C, MOTOR1_MODE, SPEED_MODE);

	received = false;

	while(ros::ok()){
		motor_speeds.x = wiringPiI2CReadReg8(motor_control_I2C, MOTOR1_SPEED);
		motor_speeds.y = wiringPiI2CReadReg8(motor_control_I2C, MOTOR2_SPEED);
		motor_speeds.z = wiringPiI2CReadReg8(motor_control_I2C, MOTOR3_SPEED);
		motor_speeds.w = wiringPiI2CReadReg8(motor_control_I2C, MOTOR4_SPEED);

		name1.publish(motor_speeds);

		if (received){
			received = false;
			// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR1_SPEED_POINT,  <scaled motor speed>);
			// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR2_SPEED_POINT,  <scaled motor speed>);
			// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR3_SPEED_POINT,  <scaled motor speed>);
			// wiringPiI2CWriteReg8(motor_control_I2C, MOTOR4_SPEED_POINT,  <scaled motor speed>);
		}

		rate.sleep();
	}
}
