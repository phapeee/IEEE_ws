#include <ros/ros.h>
#include <wiringpi.h>
#include <mecanum_ik/vector4_msg.h>
#define I2C_ADD 0x24

// motor mode defines
#define SPEED_MODE 0x02
//defines for the PWM of the motors
#define MOTOR1_PWM 0x20
#define MOTOR2_PWM 0x21
#define MOTOR3_PWM 0x22
#define MOTOR4_PWM 0x23
//defines for the speeds of the motors
#define MOTOR1_SPEED 0x40
#define MOTOR2_SPEED 0x41
#define MOTOR3_SPEED 0x42
#define MOTOR4_SPEED 0x43
//defines for the VIN current, float datatype, safety threshold of 2.7 amp
#define VIN_CURRENT_B0 0x90
#define VIN_CURRENT_B1 0x91
#define VIN_CURRENT_B2 0x92
#define VIN_CURRENT_B3 0x93
//defines for motor modes
//defines for motor 1
#define MOTOR1_MODE 0x50
#define MOTOR1_MAX_SPEED 0x58
#define MOTOR1_SPEED_P 0x59
#define MOTOR1_SPEED_I 0x5A
#define MOTOR1_SPEED_D 0x5B
#define MOTOR1_SPEED_POINT 0x5C
//defines for motor 2
#define MOTOR2_MODE 0x60
#define MOTOR2_MAX_SPEED 0x68
#define MOTOR2_SPEED_P 0x69
#define MOTOR2_SPEED_I 0x6A
#define MOTOR2_SPEED_D 0x6B
#define MOTOR2_SPEED_POINT 0x6C
//defines for motor 3
#define MOTOR3_MODE 0x70
#define MOTOR3_MAX_SPEED 0x78
#define MOTOR3_SPEED_P 0x79
#define MOTOR3_SPEED_I 0x7A
#define MOTOR3_SPEED_D 0x7B
#define MOTOR3_SPEED_POINT 0x7C
//defines for motor 4
#define MOTOR4_MODE 0x80
#define MOTOR4_MAX_SPEED 0x88
#define MOTOR4_SPEED_P 0x89
#define MOTOR4_SPEED_I 0x8A
#define MOTOR4_SPEED_D 0x8B
#define MOTOR4_SPEED_POINT 0x8C

mecanum_ik::vector4_msg motor_speeds;
mecanum_ik::vector4_msg read_motor_sp;
bool received;
void setMotorSpeed( const mecanum_ik::vector4_msg &motor_sp){
	read_motor_sp = motor_sp;
	received = true;
}

int main(int argc, char** argv){
	wiringPiSetup();
	motor_control_I2C = wiringPiI2CSetup(I2C_ADD);
	ros::init(argc, argv, "motor_control");
	ros::NodeHandle nh;
	ros::Rate rate(60);
	ros::Subscriber wheel_angle_vels = nh.subscribe("wheel_angle_vels", 10,setMotorSpeed);
	ros::Publisher MotorSpeeds = nh.advertise<mecanum_ik::vector4_msg>("real_motor_speeds");

//writing the max speeds into each motor
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR1_MAX_SPEED, <scaled max motor speed>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR2_MAX_SPEED, <scaled max motor speed>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR3_MAX_SPEED, <scaled max motor speed>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR4_MAX_SPEED, <scaled max motor speed>);

//writing the mode to each motor. Chosen mode: SPEED
	wiringPiI2CWriteReg8(mc_I2C, MOTOR1_MODE, SPEED_MODE);
	wiringPiI2CWriteReg8(mc_I2C, MOTOR2_MODE, SPEED_MODE);
	wiringPiI2CWriteReg8(mc_I2C, MOTOR3_MODE, SPEED_MODE);
	wiringPiI2CWriteReg8(mc_I2C, MOTOR4_MODE, SPEED_MODE);

//writing PID control into each motor. Should be nearly Identical
	//Writing PID control for motor 1
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR1_SPEED_P, <Proportional value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR1_SPEED_I, <Integral value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR1_SPEED_D, <Derivative value>);

	//Writing PID control for motor 2
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR2_SPEED_P, <Proportional value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR2_SPEED_I, <Integral value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR2_SPEED_D, <Derivative value>);

	//Writing PID control for motor 3
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR3_SPEED_P, <Proportional value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR3_SPEED_I, <Integral  value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR3_SPEED_D, <Derivative value>);

	//Writing PID control for motor 4
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR4_SPEED_P, <Proportional value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR4_SPEED_I, <Integral value>);
	//wiringPiI2CWriteReg8(mc_I2C, MOTOR4_SPEED_D, <Derivative value>);


	received = false;
	while(ros::ok()){
		motor_speed.x = wiringPiI2CReadReg8(mc_I2C, MOTOR1_SPEED);
		motor_speed.y = wiringPiI2CReadReg8(mc_I2C, MOTOR2_SPEED);
		motor_speed.z = wiringPiI2CReadReg8(mc_I2C, MOTOR3_SPEED);
		motor_speed.w = wiringPiI2CReadReg8(mc_I2C, MOTOR4_SPEED);
		MotorSpeeds.publish(motor_speeds);
		if(received) {
			received = false;
			//wiringPiI2CWriteReg8(mc_I2C, MOTOR1_SPEED_POINT, <scaled motor speed>);
			//wiringPiI2CWriteReg8(mc_I2C, MOTOR2_SPEED_POINT, <scaled motor speed>);
			//wiringPiI2CWriteReg8(mc_I2C, MOTOR3_SPEED_POINT, <scaled motor speed>);
			//wiringPiI2CWriteReg8(mc_I2C, MOTOR4_SPEED_POINT, <scaled motor speed>);
		}
		rate.sleep();
	}
}
