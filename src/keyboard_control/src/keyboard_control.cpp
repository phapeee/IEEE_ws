#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <termios.h>
#include <iostream>

#define MAX_VELOCITY 1.0
#define MAX_ANG_VELOCITY 1.0

char getKeyPress() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_velocity_publisher");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3>("Bot_Velocities", 10);

    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 last_velocity;
    
    std::cout << "Control the robot using WASD keys, QEZC for diagonals, and S/Shift+S for rotation" << std::endl;

    while (ros::ok()) {
        char key = getKeyPress();
        velocity.x = 0;
        velocity.y = 0;
        velocity.z = 0;
        bool key_pressed = true;

        switch (key) {
            case 'w': velocity.x = MAX_VELOCITY; break;
            case 'x': velocity.x = -MAX_VELOCITY; break;
            case 'd': velocity.y = MAX_VELOCITY; break;
            case 'a': velocity.y = -MAX_VELOCITY; break;
            case 'q': velocity.x = MAX_VELOCITY; velocity.y = -MAX_VELOCITY; break;
            case 'e': velocity.x = MAX_VELOCITY; velocity.y = MAX_VELOCITY; break;
            case 'z': velocity.x = -MAX_VELOCITY; velocity.y = -MAX_VELOCITY; break;
            case 'c': velocity.x = MAX_VELOCITY; velocity.y = -MAX_VELOCITY; break;
            case 's': velocity.z = MAX_ANG_VELOCITY; break;
            case 'S': velocity.z = -MAX_ANG_VELOCITY; break; // Shift + S
            default: 
                key_pressed = false;
                break;
        }

        // Only publish if a key was pressed or if last velocity was non-zero (to stop movement)
        if (key_pressed || last_velocity.x != 0 || last_velocity.y != 0 || last_velocity.z != 0) {
            vel_pub.publish(velocity);
            last_velocity = velocity;  // Store last velocity
        }

        ros::spinOnce();
    }

    return 0;
}

