#include "rclcomm.h"


rclcomm::rclcomm()  {
  node=rclcpp::Node::make_shared("node_hunter_ros2_qt");
  _publisher =
      node->create_publisher<std_msgs::msg::Int32>("node_hunter_ros2_qt", 10);
  _subscription = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,std::bind(&rclcomm::recv_callback,this,std::placeholders::_1));
  m_HunterDriver.initRobot();
  this->start();
}

void rclcomm::run(){
    std_msgs::msg::Int32 pub_msg;
    QVariantMap qvHunterInfo;
    pub_msg.data=0;
    rclcpp::WallRate loop_rate(50); // 50HZ 20ms
    while (true)//rclcpp::ok())
    {
        m_HunterDriver.hunterGo();
        qvHunterInfo=m_HunterDriver.GetCurData();
        emitRobotInfo(qvHunterInfo);
        _publisher->publish(pub_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void rclcomm::recv_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    emitTopicData(QString("Recived Motion Command, Speed:%1\n Angle:%2").arg(msg->linear.x,0,'G',5).arg(msg->angular.z,0,'G',5));
    if(m_HunterDriver.m_bAutoPilot){
        m_HunterDriver.angleSet(msg->angular.z);
        m_HunterDriver.SpeedSet(msg->linear.x);
    }
}
