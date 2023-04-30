#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <marvelmind_nav/hedge_pos_ang.h>
#include <marvelmind_nav/hedge_imu_raw.h>

ros::Subscriber sub_imu;
ros::Subscriber sub_pose;

ros::Publisher pub_imu;
ros::Publisher pub_pose;

long count_imu = 0;
long count_pose = 0;

void imuCallback(const marvelmind_nav::hedge_imu_raw::ConstPtr& msg)
{
    ros::Time current_time;
    current_time = ros::Time::now();
        
    sensor_msgs::Imu msg_imu;

    msg_imu.header.seq = count_imu++;
    msg_imu.header.stamp = current_time;
    msg_imu.header.frame_id = "hedgehog_imu_link";

    msg_imu.angular_velocity.x = d2rad(msg->gyro_x);
    msg_imu.angular_velocity.y = d2rad(msg->gyro_y);
    msg_imu.angular_velocity.z = d2rad(msg->gyro_z);

    msg_imu.linear_acceleration.x = g2ms(msg->acc_x);
    msg_imu.linear_acceleration.y = g2ms(msg->acc_y);
    msg_imu.linear_acceleration.z = g2ms(msg->acc_z);

    pub_imu.publish(msg_imu);
}

void poseCallback(const marvelmind_nav::hedge_pos_ang::ConstPtr& msg)
{
    ros::Time current_time;
    current_time = ros::Time::now();
        
    geometry_msgs::PoseWithCovarianceStamped msg_pose;

    msg_pose.header.seq = count_pose++;
    msg_pose.header.stamp = current_time;
    msg_pose.header.frame_id = "hedgehog_link";
    
    msg_pose.pose.pose.position.x = msg->x_m;
    msg_pose.pose.pose.position.y = msg->x_y;
    msg_pose.pose.pose.position.z = msg->x_z;

    msg_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg->angle);

    pub_pose.publish(msg_pose);
}

// convert raw IMU value from G to m/s
float64 g2ms(int16 val){
    return val * 9.80665 / 1000;
}

// convert raw IMU value from 0.0175deg/unit to rad
float64 d2rad(int16 val){
    return val * 0.0175 * (3.1416 / 180);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "marvelmind_republisher");
	ros::NodeHandle nh;

    // subscribe to topics
    sub_imu = nh.subscribe<marvelmind_nav::hedge_imu_raw>("hedge_imu_raw", 1, imuCallback, this)
    sub_imu = nh.subscribe<marvelmind_nav::hedge_pos_ang>("hedge_pos_ang", 1, poseCallback, this)

    // advertise topics
	pub_imu = nh.advertise<sensor_msgs::Imu>("hedge_imu", 1); // imu1 in case there is already an internal IMU topic
	pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("hedge_pose", 1);

	ros::spin()

    return 0;
}