// Include various libraries
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub_quat;

// Hajime Cart parameter
//const double wheel_radius= 0.085;
//const double base_width = 0.236;
//const double_pi = 3.141592654;
//const double xp_enc_ticks_meter = 6250.0 / (2.0 * _pi * wheel_radius);

// Robot physical constants
#if 1
const double TICKS_PER_REVOLUTION = 102.0; // For reference purposes.
#else
const double TICKS_PER_REVOLUTION = 1860.0; // For reference purposes.
#endif
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.15; // Center of left tire to center of right tire
const double TICKS_PER_METER = (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS));

double encoder_left = 0;
double encoder_right = 0;
double encoder_left_old = 0;
double encoder_right_old = 0;
double pos_x = 0.0;
double pos_y = 0.0;
double pos_th = 0.0;

int flag_firsttime = 1;

//next, we'll publish the odometry message over ROS
nav_msgs::Odometry odom;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {
  encoder_left = leftCount.data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
  encoder_right = rightCount.data;
}

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odom.pose.pose.position.x = rvizClick.pose.position.x;
  odom.pose.pose.position.y = rvizClick.pose.position.y;
  odom.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;

  ROS_WARN("set_initial_2d %d", initialPoseRecieved);
}

ros::Time current_time, last_time;

// Update odometry information
void update_odom() {
     static tf::TransformBroadcaster odom_broadcaster;

    if( flag_firsttime )
    {
        flag_firsttime = 0;

        encoder_left_old = encoder_left;
        encoder_right_old = encoder_right;

	      last_time = ros::Time::now();

        return;
    }

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    //double dt=((double)current_time.sec + 1e-9*(double)current_time.nsec) - ((double)last_time.sec + 1e-9*(double)last_time.nsec);
    double dt = (current_time - last_time).toSec();

    double d_left = (encoder_left -  encoder_left_old ) / TICKS_PER_METER;
    double d_right = (encoder_right -  encoder_right_old ) / TICKS_PER_METER;

    encoder_left_old = encoder_left;
    encoder_right_old = encoder_right;

    double d = (d_left + d_right) / 2.0;
    double th = (d_right - d_left) / WHEEL_BASE;

    // calc dt becomes big for the first time
    double dx;
    double dr;
    if( dt < 1000.0 ) 
    {
        dx = d / dt;
        dr = th / dt;
    } else
    {
        dt = 0.0;
        dx = 0;
        dr = 0;
    }

    if( fabs(d) > 1.0e-5 )
    {
        double _x = cos(th) * d;
        double _y = -sin(th) * d;
        pos_x += (_x * cos(pos_th) - _y * sin(pos_th));
        pos_y += (_x * sin(pos_th) + _y * cos(pos_th));
    }
    if(fabs(th) > 1.0e-5 )
    {
        pos_th += th;
    }

    // const double wr= 0.085;
    // const double ws = 0.236;
    //    r =  ws/2.0 * (encoder_left + encoder_right)/(encoder_right - encoder_left );
    //  omega_dt = (encoder_right - encoder_left) * step / ws
    //  iccx = x - r * sin(theta)
    //  iccy = y + r * cos(theta)
    //  x_new = (x - iccx) * cos(omega_dt) - (y - iccy) * sin(omega_dt) + iccx;
    //  y_new = (x - iccx) * sin(omega_dt) + (y - iccy) * cos(omega_dt) + iccy;
    //  theta_new = theta + omega_dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_th);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_footprint";

    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "/odom";

    //set the position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    for(int i = 0; i<36; i++) {
      if(i == 0 || i == 7 || i == 14) {
        odom.pose.covariance[i] = .01;
      }
      else if (i == 21 || i == 28 || i== 35) {
        odom.pose.covariance[i] += 0.1;
      }
      else {
        odom.pose.covariance[i] = 0;
      }
    }
    
    //set the velocity
    odom.child_frame_id = "/base_footprint";
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dr;

    //publish the message
    odom_data_pub_quat.publish(odom);

    last_time = current_time;
}
 
int main(int argc, char **argv) {
  // Launch ROS and create a node
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle node;

  ros::param::get("/odom_pub/initialPoseRecieved", initialPoseRecieved);
  ROS_WARN("InitPose %d", initialPoseRecieved);

  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);

  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
 
  ros::Rate loop_rate(30); 
     
  while(ros::ok()) {
  if(initialPoseRecieved) {
      update_odom();
  }
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
