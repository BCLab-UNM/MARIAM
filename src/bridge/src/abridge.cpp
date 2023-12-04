#define BOOST_BIND_NO_PLACEHOLDERS

#include <cmath>
#include <string>
#include <sstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <boost/thread.hpp>

//ROS libraries
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <dynamic_reconfigure/server.h>
//#include <dynamic_reconfigure/client.h>

//ROS messages
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int8.hpp>
//#include <std_srvs/srv/empty.hpp>

// Project messages 

//#include <bridge_msg/bridge_msg/msg/pid_state.hpp>
///home/carter/MARIAM/install/bridge/include/bridge/bridge/msg/pid_state.hpp

//Package include
#include <usbSerial.h>

#include "pid.h"
#include <bridge_msg/msg/pid_state.hpp>
//#include <pid_state__struct.hpp>
using std::placeholders::_1;

using namespace std;


class abridge : public rclcpp::Node {
public:
    abridge(): Node("abridge"){
        string devicePath = "/dev/ttyACM0"; //Hardcoded for now works fine as there is no GPS, @TODO use the param
        //rclcpp::param::param("~device", devicePath, string("/dev/ttyUSB0"));
        usb.openUSBPort(devicePath, baud);
        odomPublish = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        infoLogPublisher = this->create_publisher<std_msgs::msg::String>("/infoLog", 1); //@TODO fix qos to be latching if needed
        debugPIDPublisher = this->create_publisher<bridge_msg::msg::PidState>("bridge/debugPID", 1); //@TODO fix qos to be latching if needed
        heartbeatPublisher = this->create_publisher<std_msgs::msg::String>("bridge/heartbeat", 1);

        driveControlSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("driveControl", 10, std::bind(&abridge::driveCommandHandler, this, _1));
        modeSubscriber = this->create_subscription<std_msgs::msg::UInt8>("mode", 1, std::bind(&abridge::modeHandler, this, _1));

        serialActivityTimerTimer = this->create_wall_timer(deltaTime,
                                                                std::bind(&abridge::serialActivityTimer,
                                                                          this));
        publishHeartBeatTimerEventHandlerTimer = this->create_wall_timer(heartbeat_publish_interval,
                                                                                          std::bind(
                                                                                                  &abridge::publishHeartBeatTimerEventHandler,
                                                                                                  this));
        this->declare_parameter("odom_frame", "odom");
        odom.header.frame_id="odom";
        odom.child_frame_id="base_link";
        /*
        rclcpp::param::param<std_msgs::msg::string>("odom_frame", odom.header.frame_id, "odom");
        rclcpp::param::param<std_msgs::msg::string>("base_link_frame", odom.child_frame_id, "base_link");
        */
    }

private:
    nav_msgs::msg::Odometry odom;
    double odomTheta = 0;
    USBSerial usb;
    const int baud = 115200;
    char moveCmd[16];
    char host[128];
    chrono::duration<int_least64_t, milli> deltaTime = 100ms; //abridge's update interval
    chrono::duration<int_least64_t, milli> heartbeat_publish_interval = 2s;
    rclcpp::TimerBase::SharedPtr serialActivityTimerTimer;
    rclcpp::TimerBase::SharedPtr publishHeartBeatTimerEventHandlerTimer;
    int currentMode = 0;
    geometry_msgs::msg::Twist speedCommand;
    void driveCommandHandler(const geometry_msgs::msg::Twist::SharedPtr message);
    void serialActivityTimer();
    void modeHandler(const std_msgs::msg::UInt8::SharedPtr message);
    void publishRosTopics();
    void parseData(string str);
    //void initialconfig();

    double leftTicksToMeters(double leftTicks) const;
    double rightTicksToMeters(double rightTicks) const;
    double metersToTicks(double meters) const;
    double leftMetersToTicks(double meters) const;
    double rightMetersToTicks(double meters) const;
    double diffToTheta(double right, double left) const;
    double thetaToDiff(double theta) const;

    //Callback handlers
    void publishHeartBeatTimerEventHandler();
    //void reconfigure(bridge::pidConfig &cfg, uint32_t level);

    // Allowing messages to be sent to the arduino too fast causes a disconnect
    // This is the minimum time between messages to the arduino in microseconds.
    // Only used with the gripper commands to fix a manual control bug.
    unsigned int min_usb_send_delay = 100;

    const double wheelBase = 0.278; //distance between left and right wheels (in M)
    const double leftWheelCircumference = 0.361283; // hardwheels
    const double rightWheelCircumference = 0.361283; // (in M)
    const int cpr = 2100; //2094.625; //Was with old encoder lib 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution

    // running counts of encoder ticks
    int leftTicks = 0;
    int rightTicks = 0;
    // wheel velocities in ticks/sec
    double leftTickVel = 0;
    double rightTickVel = 0;
    double odomTS = 0;

    // Immobilize robot until the first PID configuration.
    //PID left_pid = PID(0, 0, 0, 0, 120, -120, 0, -1);
    //PID right_pid = PID(0, 0, 0, 0, 120, -120, 0, -1);
    //Be Free
    // p,  i,  d,  db,  hi,  lo,  stick=-1,  wu=0
    PID left_pid = PID(0.0075, 0.15, 0.002, 0, 120, -120, -1, 3.0);
    PID right_pid = PID(0.0075, 0.15, 0.002, 0, 120, -120, -1, 3.0);

    /*
      <param name="Kp" value="0.0075" />
      <param name="Ki" value="0.15" />
      <param name="Kd" value="0.002" />
      <param name="db" value="0.0" />
      <param name="st" value="0.0" />
      <param name="wu" value="3.0" />
      <param name="ff_l" value="0.0095" />
      <param name="ff_r" value="0.012" />
    
    */

    //Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublish;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr infoLogPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPublisher;
    rclcpp::Publisher<bridge_msg::msg::PidState>::SharedPtr debugPIDPublisher;

    //rclcpp::Publisher debugPIDPublisher;
    bridge_msg::msg::PidState pid_state;

    //Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr driveControlSubscriber;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  modeSubscriber;

    //Timers
    //rclcpp::Timer publishTimer;
    //rclcpp::Timer publish_heartbeat_timer;

    // Feed-forward constants
    double ff_l = 0.0095;
    double ff_r = 0.012;

    //void publishRosTopics();
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<abridge>());
    rclcpp::shutdown();
    return 0;
} //end main

void abridge::driveCommandHandler(const geometry_msgs::msg::Twist::SharedPtr message) {
  speedCommand.linear.x = message->linear.x;
  speedCommand.angular.z = message->angular.z;
  speedCommand.angular.y = message->angular.y;
}

double abridge::leftTicksToMeters(double leftTicks_arg) const {
	return (leftWheelCircumference * leftTicks_arg) / cpr;
}

double abridge::rightTicksToMeters(double rightTicks_arg) const {
	return (rightWheelCircumference * rightTicks_arg) / cpr;
}

double abridge::metersToTicks(double meters) const {
    return (meters * cpr) / ((leftWheelCircumference + rightWheelCircumference) / 2);
}

double abridge::leftMetersToTicks(double meters) const {
    return (meters * cpr) / leftWheelCircumference;
}

double abridge::rightMetersToTicks(double meters) const {
    return (meters * cpr) / rightWheelCircumference;
}

double abridge::diffToTheta(double right, double left) const {
	return (right - left) / wheelBase;
}

double abridge::thetaToDiff(double theta) const {
	return theta * wheelBase;
}

void abridge::serialActivityTimer() {
    static unsigned delay = 0;
    
	int cmd_mode = round(speedCommand.angular.y);
    //RCLCPP_INFO(this->get_logger(), "greetings: '%f'", speedCommand.linear.x);
    //RCLCPP_INFO(this->get_logger(), "DRIVE_MODE: '%d'", cmd_mode);

	if (cmd_mode == DRIVE_MODE_STOP) {
		left_pid.reset();
		right_pid.reset();
		memset(&moveCmd, ' ', sizeof (moveCmd));
		sprintf(moveCmd, "\n 0 0 \n");
		usb.sendData(moveCmd);		
	}
	else {
		// Calculate tick-wise velocities.
		double linear_sp = metersToTicks(speedCommand.linear.x);
		double angular_sp = metersToTicks(thetaToDiff(speedCommand.angular.z));

        double left_sp = leftMetersToTicks(speedCommand.linear.x) - angular_sp;
        double right_sp = rightMetersToTicks(speedCommand.linear.x) + angular_sp;

		int l = round(left_pid.step(left_sp, leftTickVel, odomTS));
		int r = round(right_pid.step(right_sp, rightTickVel, odomTS));

		// Feed forward
		l += ff_l * left_sp;
		r += ff_r * right_sp;

		// Debugging: Report PID performance for tuning.
		// Output of the PID is in Linear:
		pid_state.header.stamp = this->get_clock()->now();
		pid_state.left_wheel.output = l; // sp_linear * ff
		pid_state.right_wheel.output = r;
		pid_state.left_wheel.error = left_sp - leftTickVel; // sp - feedback
		pid_state.right_wheel.error = right_sp - rightTickVel; // sp - feedback
		pid_state.left_wheel.p_term = left_pid.getP();
		pid_state.right_wheel.p_term = right_pid.getP();
		pid_state.left_wheel.i_term = left_pid.getI();
		pid_state.right_wheel.i_term = right_pid.getI();
		pid_state.left_wheel.d_term = left_pid.getD();
		pid_state.right_wheel.d_term = right_pid.getD();
		pid_state.linear_setpoint = linear_sp;
		pid_state.left_wheel.setpoint = left_sp;
		pid_state.right_wheel.setpoint = right_sp;

		// Feedback function is in Angular:
		pid_state.angular_setpoint = angular_sp;
		pid_state.left_wheel.feedback = leftTickVel;
		pid_state.right_wheel.feedback = rightTickVel;
		//debugPIDPublisher->publish(pid_state); //@TODO put this back
        delay++;
        if(delay==100){
            delay=0;
            sprintf(moveCmd, "\n %d %d \n", l, r); //format data for arduino into c string
            usb.sendData(moveCmd);                      //send movement command to arduino over usb
            memset(&moveCmd, ' ', sizeof (moveCmd));   //clear the movement command string
        }
        //RCLCPP_INFO(this->get_logger(), "l:%d,r:%d", l, r);
	}

	try {
		parseData(usb.readData());
		publishRosTopics();
	} catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Exception while parsing Arduino data.");
	}
}

void abridge::publishRosTopics() {
	/*
    odomPublish->publish(odom);
    */
}

void abridge::parseData(string str) {
    static double lastOdomTS = 0;

    std::istringstream fullStream(str);
    std::string line;

    while (std::getline(fullStream, line)) {
        std::istringstream lineStream(line);
        lineStream >> leftTicks >> rightTicks;

        if (lineStream.fail()) {
            std::cerr << "Error: Unable to parse the line '" << line << "'" << std::endl;
            continue; // Skip to the next line
        }

            //odomTS = strtod(dataSet.at(4).c_str(), nullptr) / 1000; // Seconds
            rclcpp::Time now = this->get_clock()->now();
            odomTS =  now.seconds() + now.nanoseconds() / 1e9; // double check this
            double rightWheelDistance = rightTicksToMeters(rightTicks);

            double leftWheelDistance = leftTicksToMeters(leftTicks);

            //Calculate relative angle that robot has turned
            double dtheta = diffToTheta(rightWheelDistance, leftWheelDistance);

            //Accumulate angles to calculate absolute heading
            odomTheta += dtheta;

            //Decompose linear distance into its component values
            double meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
            //Twist is in base_link frame, use relative heading
            double twistX = meanWheelDistance * cos(dtheta);
            double twistY = meanWheelDistance * sin(dtheta);
            //Pose is in the odom frame, use absolute heading
            double poseX = meanWheelDistance * cos(odomTheta);
            double poseY = meanWheelDistance * sin(odomTheta);


            // Calculate velocities if possible.
            double vtheta = 0;
            double vx = 0;
            double vy = 0;
            if (lastOdomTS > 0) {
                double deltaT = odomTS - lastOdomTS;
                vtheta = dtheta / deltaT;
                vx = twistX / deltaT;
                vy = twistY / deltaT;

                // Normalize ticks to ticks/s
                leftTickVel = leftTicks / deltaT;
                rightTickVel = rightTicks / deltaT;
            }
            lastOdomTS = odomTS;

            odom.header.stamp = this->get_clock()->now();
            odom.pose.pose.position.x += poseX;
            odom.pose.pose.position.y += poseY;
            odom.pose.pose.position.z = 0;

            tf2::Quaternion quat_tf;
            quat_tf.setRPY(0,0, odomTheta);
            odom.pose.pose.orientation = tf2::toMsg(quat_tf);

            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vtheta;

            odomPublish->publish(odom);
		
	}
}

void abridge::modeHandler(const std_msgs::msg::UInt8::SharedPtr message) {
	currentMode = message->data;
}

void abridge::publishHeartBeatTimerEventHandler() {
    std_msgs::msg::String msg;
    msg.data = "";
    heartbeatPublisher->publish(msg);
}
