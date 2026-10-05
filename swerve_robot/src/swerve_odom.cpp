// 里程计节点，订阅/joint_states获取真实转向角于驱动轮角速度，航迹推算发布odom+TF





#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>   //ros消息，关节状态jointstate,保存各个关节位置、速度
#include <tf/transform_broadcaster.h>  //tf广播器头文件，发布坐标系变换 odom->base_link
#include <cmath>
#include <string>


//定义c++类，把所有变量、回调函数，里程计算法全部封装在一起
class SwerveOdom
{
public:

    //构造函数：函数名和类名同名，无返回值
    SwerveOdom(ros::NodeHandle& nh)  //这里的nh是一个实例化对象
    {
        //底盘参数：与 swerve_controller.cpp / urdf 保持一致（半轴距、半轮距）
        L_ = 0.175;
        W_ = 0.125;
        wheel_r_ = 0.075;

        fl_steer_ = 0.0; fr_steer_ = 0.0; rl_steer_ = 0.0; rr_steer_ = 0.0;
        fl_drive_ = 0.0; fr_drive_ = 0.0; rl_drive_ = 0.0; rr_drive_ = 0.0;

        x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        var_x_ = 1e-4; var_y_ = 1e-4; var_th_ = 1e-4;
        last_time_ = ros::Time::now();

        // 订阅关节状态：在同一个回调里拿到角度与速度，天然时间一致，避免指令/反馈错帧
        //sub_joint_ pub_odom_ 是一个对象哦！后面定义了呢

        sub_joint_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &SwerveOdom::jointStateCallback, this);

        pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        ROS_INFO("Swerve Odom Node Ready");
    }




    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        for(size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string& n = msg->name[i];
            double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
            double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;

            if(n == "front_left_steering_joint")       fl_steer_ = pos;
            else if(n == "front_right_steering_joint") fr_steer_ = pos;
            else if(n == "rear_left_steering_joint")   rl_steer_ = pos;
            else if(n == "rear_right_steering_joint")  rr_steer_ = pos;

            else if(n == "front_left_drive_joint")       fl_drive_ = vel;
            else if(n == "front_right_drive_joint")      fr_drive_ = vel;
            else if(n == "rear_left_drive_joint")        rl_drive_ = vel;
            else if(n == "rear_right_drive_joint")       rr_drive_ = vel;
        }
        updateOdom(msg->header.stamp);
    }

    void updateOdom(const ros::Time& stamp)
    {
        ros::Time current_time = stamp.isZero() ? ros::Time::now() : stamp;
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        if(dt <= 1e-6)
            return;
        if(dt > 0.5)   // 时间跳变（如仿真重启），丢弃该帧，避免积分出巨大位移
            return;
        if(dt > 0.1) dt = 0.1;   // 轻微超时则钳位，而不是整段丢弃

        double v_fl = fl_drive_ * wheel_r_;
        double v_fr = fr_drive_ * wheel_r_;
        double v_rl = rl_drive_ * wheel_r_;
        double v_rr = rr_drive_ * wheel_r_;

        double v_fl_x = v_fl * cos(fl_steer_);
        double v_fl_y = v_fl * sin(fl_steer_);
        double v_fr_x = v_fr * cos(fr_steer_);
        double v_fr_y = v_fr * sin(fr_steer_);
        double v_rl_x = v_rl * cos(rl_steer_);
        double v_rl_y = v_rl * sin(rl_steer_);
        double v_rr_x = v_rr * cos(rr_steer_);
        double v_rr_y = v_rr * sin(rr_steer_);

        double vx_sum = v_fl_x + v_fr_x + v_rl_x + v_rr_x;
        double vy_sum = v_fl_y + v_fr_y + v_rl_y + v_rr_y;
        // 分子：Σ(x_i * v_iy - y_i * v_ix)，四轮对称布置时 Σ(x_i²+y_i²) = 4(L²+W²)
        double w_sum = (-W_)*v_fl_x + L_*v_fl_y
                     + ( W_)*v_fr_x + L_*v_fr_y
                     + (-W_)*v_rl_x + (-L_)*v_rl_y
                     + ( W_)*v_rr_x + (-L_)*v_rr_y;

        double vx = vx_sum / 4.0;
        double vy = vy_sum / 4.0;
        double w = w_sum / (4.0 * (L_*L_ + W_*W_));

        double delta_theta = w * dt;
        // 先转角度再用新姿态积分位移，减小大转角时的离散误差
        double theta_mid = theta_ + delta_theta * 0.5;
        double delta_x = (vx * cos(theta_mid) - vy * sin(theta_mid)) * dt;
        double delta_y = (vx * sin(theta_mid) + vy * cos(theta_mid)) * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        theta_ = fmod(theta_, 2*M_PI);
        if(theta_ > M_PI) theta_ -= 2*M_PI;
        if(theta_ < -M_PI) theta_ += 2*M_PI;

        // 协方差随运动累积，给上层提供真实的里程计置信度（而非写死的极小值）
        var_x_  += 1e-3 * fabs(delta_x)  + 1e-4 * fabs(delta_theta) + 1e-6;
        var_y_  += 1e-3 * fabs(delta_y)  + 1e-4 * fabs(delta_theta) + 1e-6;
        var_th_ += 5e-3 * fabs(delta_theta) + 1e-5 * (fabs(delta_x) + fabs(delta_y)) + 1e-6;

        //填充odom消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = w;

        for(int i=0; i<36; i++) odom_msg.pose.covariance[i] = 0.0;
        odom_msg.pose.covariance[0]  = var_x_;
        odom_msg.pose.covariance[7]  = var_y_;
        odom_msg.pose.covariance[35] = var_th_;

        for(int i=0; i<36; i++) odom_msg.twist.covariance[i] = 0.0;
        odom_msg.twist.covariance[0]  = 1e-3;
        odom_msg.twist.covariance[7]  = 1e-3;
        odom_msg.twist.covariance[35] = 1e-3;

        pub_odom_.publish(odom_msg);

        // ========== TF广播，使用类成员tf_broadcaster_ ==========
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_, y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0,0,theta_);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
    }

private:
    // TF广播器放到类成员，只创建一次
    tf::TransformBroadcaster tf_broadcaster_;

    double L_, W_, wheel_r_;
    double fl_steer_, fr_steer_, rl_steer_, rr_steer_;
    double fl_drive_, fr_drive_, rl_drive_, rr_drive_;
    double x_, y_, theta_;
    double var_x_, var_y_, var_th_;
    ros::Time last_time_;



    //sub_joint_、pub_odom_是实例化对象
    ros::Subscriber sub_joint_;
    ros::Publisher pub_odom_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swerve_odom_node");
    ros::NodeHandle nh;
    SwerveOdom odom(nh);

    ROS_INFO("Swerve odom subscribes /joint_states");
    ros::spin();
    return 0;
}
