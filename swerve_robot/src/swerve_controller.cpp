
// 四轮舵轮机器人运动学解算
// 订阅底盘速度指令cmd_vel,结算出4个转向角和驱动电机角速度并发布


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  //速度
#include <std_msgs/Float64.h>
#include <cmath>  //数序库
#include <algorithm>  //算法库 std::max

class SwerveController //舵轮控制器类
{
public:
    SwerveController(ros::NodeHandle& nh)   //构造函数,引用形参传参，nh是ros::NodeHandle类的一个实例
    {
        // ========== 30cm×30cm正方形底盘：半轴距L=0.15，半轮距W=0.15 ==========
        L_ = 0.15;  //底盘半长，前后方向，质心道前轮的距离
        W_ = 0.15;   //底盘半宽，左右方向，质心到左轮的距离
        wheel_r_ = 0.075;  //轮子的半径   0.075m,7.5cm
        max_wheel_speed_ = 5.0;   //驱动电机允许最大角速度 rad/s

        // 发布器        advertise是nh实例化对象的一个成员函数，这里就是调用成员函数的函数模板，需要传
        //转向电机发布器  发布舵轮目标转向角
        pub_fl_steer_ = nh.advertise<std_msgs::Float64>("/front_left_steer_pos/command", 10);
        pub_fr_steer_ = nh.advertise<std_msgs::Float64>("/front_right_steer_pos/command", 10);
        pub_rl_steer_ = nh.advertise<std_msgs::Float64>("/rear_left_steer_pos/command", 10);
        pub_rr_steer_ = nh.advertise<std_msgs::Float64>("/rear_right_steer_pos/command", 10);
        //驱动电机发布器   发布驱动电机目标角速度
        pub_fl_drive_ = nh.advertise<std_msgs::Float64>("/front_left_drive_vel/command", 10);
        pub_fr_drive_ = nh.advertise<std_msgs::Float64>("/front_right_drive_vel/command", 10);
        pub_rl_drive_ = nh.advertise<std_msgs::Float64>("/rear_left_drive_vel/command", 10);
        pub_rr_drive_ = nh.advertise<std_msgs::Float64>("/rear_right_drive_vel/command", 10);

        // 订阅cmd_vel
        //调用nh成员函数的subscribe成员函数，自动调用回调函数
        sub_cmd_vel_ = nh.subscribe("/cmd_vel", 10, &SwerveController::cmdVelCallback, this);
        ROS_INFO("Swerve OOP Controller Ready (30x30 square chassis)");
    }

private:  
    double L_, W_, wheel_r_, max_wheel_speed_;

    //pub_rr_driver是ros::Publisher类的实例对象
    ros::Publisher pub_fl_steer_, pub_fr_steer_, pub_rl_steer_, pub_rr_steer_;
    ros::Publisher pub_fl_drive_, pub_fr_drive_, pub_rl_drive_, pub_rr_drive_;
    ros::Subscriber sub_cmd_vel_;
    

    //cmd_vel回调函数
    //msg：智能指针
    //要学一下typedef，我忘记了
    //geometry_msgs::Twist::Cinstptr就是boost::shared_ptr<const geometry_msg::Twist>
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        double vx = msg->linear.x;   //底盘质心x方向线速度
        double vy = msg->linear.y;   //y
        double w = msg->angular.z;   //z轴旋转角速度

        // 静止保护，当速度指令接近0，直接置零 ，防止atan2（0，0）算出NaN非法数值
        if(fabs(vx) < 1e-4 && fabs(vy) <1e-4 && fabs(w) <1e-4)
        {
            std_msgs::Float64 ang_msg, spd_msg;
            ang_msg.data = 0.0;
            pub_fl_steer_.publish(ang_msg);
            pub_fr_steer_.publish(ang_msg);
            pub_rl_steer_.publish(ang_msg);
            pub_rr_steer_.publish(ang_msg);

            spd_msg.data = 0.0;
            pub_fl_drive_.publish(spd_msg);
            pub_fr_drive_.publish(spd_msg);
            pub_rl_drive_.publish(spd_msg);
            pub_rr_drive_.publish(spd_msg);
            return;   //直接退出回调
        }

        // 四舵轮模块速度


        //舵轮点速度=底盘质心速度+旋转带来的线速度 v=v_body+w*r

        double v_fl_x = vx - w * W_;
        double v_fl_y = vy + w * L_;
        double v_fr_x = vx + w * W_;
        double v_fr_y = vy + w * L_;
        double v_rl_x = vx - w * W_;
        double v_rl_y = vy - w * L_;
        double v_rr_x = vx + w * W_;
        double v_rr_y = vy - w * L_;

        // cmath库函数，根据y,x分量求角度，返回弧度
        double ang_fl = atan2(v_fl_y, v_fl_x);
        double ang_fr = atan2(v_fr_y, v_fr_x);
        double ang_rl = atan2(v_rl_y, v_rl_x);
        double ang_rr = atan2(v_rr_y, v_rr_x);

        // sqrt(),cmath库平方根函数，计算矢量模长，勾股定理
        double spd_fl = sqrt(v_fl_x*v_fl_x + v_fl_y*v_fl_y);
        double spd_fr = sqrt(v_fr_x*v_fr_x + v_fr_y*v_fr_y);
        double spd_rl = sqrt(v_rl_x*v_rl_x + v_rl_y*v_rl_y);
        double spd_rr = sqrt(v_rr_x*v_rr_x + v_rr_y*v_rr_y);

        // 线速度转驱动轮角速度
        double w_fl = spd_fl / wheel_r_;
        double w_fr = spd_fr / wheel_r_;
        double w_rl = spd_rl / wheel_r_;
        double w_rr = spd_rr / wheel_r_;

        // std::max({})c++11初始化列表，一次性取多个值的最大值
        double max_spd = std::max({fabs(w_fl), fabs(w_fr), fabs(w_rl), fabs(w_rr)});
        if(max_spd > max_wheel_speed_)   //判断：最大轮速超过限制
        {
            double scale = max_wheel_speed_ / max_spd;
            w_fl *= scale;
            w_fr *= scale;
            w_rl *= scale;
            w_rr *= scale;
        }

        // 发布转向角度
        //发布四个转向角速度指令
        std_msgs::Float64 ang_msg;
        ang_msg.data = ang_fl; pub_fl_steer_.publish(ang_msg);
        ang_msg.data = ang_fr; pub_fr_steer_.publish(ang_msg);
        ang_msg.data = ang_rl; pub_rl_steer_.publish(ang_msg);
        ang_msg.data = ang_rr; pub_rr_steer_.publish(ang_msg);

        // 发布驱动轮角速度
        //发布四个驱动电机角速度指令
        std_msgs::Float64 spd_msg;
        spd_msg.data = w_fl; pub_fl_drive_.publish(spd_msg);
        spd_msg.data = w_fr; pub_fr_drive_.publish(spd_msg);
        spd_msg.data = w_rl; pub_rl_drive_.publish(spd_msg);
        spd_msg.data = w_rr; pub_rr_drive_.publish(spd_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swerve_control_node");
    ros::NodeHandle nh;
    SwerveController controller(nh);
    ros::spin();
    return 0;
}
