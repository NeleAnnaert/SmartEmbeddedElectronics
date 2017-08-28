/*
 * motor_control.cpp
 * Motor Control Node For Serial Communication With The RP6 Robot
 * Bevers Tim - tim.bevers@student.kuleuven.be
 * 
 * Wiki: https://github.com/GreenTeam-KUL/ParkingWithROS/wiki/Motor-Ros
 */

#include "motor_control.hpp"

#include <ros/ros.h>
#include <serial/serial.h>
#include <motor_control/motor.h>

#include <queue>
#include <string>

#define CM2MM(x) (x)*10

using namespace std;
using namespace serial;
using std::vector;

/// Serial settings
const std::string my_serial_port = "/dev/serial0"; /// rpi = /dev/serial0
const int my_serial_baud = 38400; /// Baud rate used by rp6 controller
const int my_serial_timeout = 100;

/// Default speeds
const int rotate_speed = 45;
const int max_move_speed = 200;
const int min_move_speed = 20;

/// Maximum distance to move in mm
const int max_move_dist = 999;

/// Time in secconds to wait between commands
/// Should be long enough for most movements to be finshed
const int wait_sec = 4; 

class Mc {
    ros::NodeHandle nh;
    ros::Subscriber sub;
    std::queue<int> movements;
    std::queue<int> rotations;
    public:
    Mc(Serial *s);
    void run();
    serial::Serial *my_serial;
    private:
    void clearQueues(void);
    void callback(const motor_control::motor &msg);
    void sendRotation(int rot);
    void sendMovement(int mov);
    int dist2speed(int dist);
};

Mc::Mc(Serial *s)
{ 

    my_serial = s;

    if(my_serial->isOpen())
    {
        ROS_INFO_STREAM("Serial port is open.");

        /// If the serial port is open start the program in the controller
        my_serial->write("s\n"); /// The 's' command is the default to start the program

        ROS_INFO_STREAM("Start Command Send.");
    }
    else
    {
        /// If the serial connection can't be opened, send a error message and quit. 
        ROS_FATAL_STREAM("Exiting node: Could not open : " << my_serial_port);
        ros::shutdown();
    }

    /// Subscripe to the mapper
    sub = nh.subscribe( MAPPER_TOPIC, 1000, &Mc::callback, this);

    ROS_INFO_STREAM("Node started succesfully.");
}

void Mc::callback(const motor_control::motor &msg)
{
    ROS_DEBUG_STREAM("Message with length " << msg.length << " recieved.");
    if( msg.array_distance[0] == 0 && msg.array_angle[0] == 0)
    {
        clearQueues();
        ROS_DEBUG_STREAM("Stop message recieved.");
    }
    else
    {
        /// Add all elements to the queue
        for(size_t i = 0; i < msg.length; i++)
        {
            int movement = CM2MM(msg.array_distance[i]);
            while(movement > max_move_dist)
            {
                movements.push(max_move_dist);  // Create extra step in movement
                rotations.push(0);              // Add empty rotation to keep the lists the same length
                movement -= max_move_dist;
            }
            movements.push(movement);
            rotations.push(msg.array_angle[i]);
        }
    }
}
void Mc::clearQueues(void)
{
    movements = queue<int>();
    rotations = queue<int>();
}

void Mc::run(void)
{
    if( movements.size() <= 0)
    {
        return;
    }

    int rot = rotations.front();
    rotations.pop();
    if( rot != 0 )
    {
        sendRotation(rot);
    }

    int mov = movements.front();
    movements.pop();
    if( mov != 0 )
    {
        sendMovement(mov);
    }
}

int Mc::dist2speed(int dist)
{
    dist = abs(dist)/10; /// Use distance in cm as reference for a good speed.
    if(dist < min_move_speed)
    {
        return min_move_speed;
    }
    if(dist > max_move_speed)
    {
        return max_move_speed;
    }
    return dist;
}

void Mc::sendRotation(int rot)
{
    /// Type, direction, newline
    char type[] = "rr";
    if(rot < 0)
    {
        type[1] = 'l';
    }
    my_serial->write(type);

    /// Speed
    char speed[3];
    sprintf(speed, "%03d", rotate_speed);
    my_serial->write(speed);

    /// Angle 
    char angle[3];
    sprintf(angle, "%03d", abs(rot));
    my_serial->write(angle);

    /// Distance
    my_serial->write(angle);
    my_serial->write("\n");
    
    ROS_DEBUG_STREAM(string(type) + string(speed) + string(angle) + string(angle));

    int wait = wait_sec;
    sleep(wait);
}

void Mc::sendMovement(int mov)
{

    /// Type, direction, newline
    char type[] = "mf";
    if(mov < 0)
    {
        type[1] = 'b';
    }
    my_serial->write(type);

    /// Speed
    char speed[3];
    sprintf(speed, "%03d", dist2speed(mov));
    my_serial->write(speed);

    /// Angle 
    char angle[3];
    sprintf(angle, "%03d", abs(mov));
    my_serial->write(angle);

    /// Distance
    my_serial->write(angle);
    my_serial->write("\n");

    ROS_DEBUG_STREAM(string(type) + string(speed) + string(angle) + string(angle));

    int wait = wait_sec;
    sleep(wait);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control");
    ROS_INFO_STREAM("Serial Control Node starting, will try and connecto to " << my_serial_port << ".");

    Serial s(my_serial_port, my_serial_baud, Timeout::simpleTimeout(my_serial_timeout));

    Mc motorControl(&s);

    ros::Rate r(100);
    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        motorControl.run();
    }
}
