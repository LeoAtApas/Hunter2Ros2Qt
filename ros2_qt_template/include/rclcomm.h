#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "hunterdriver.h"

class rclcomm : public QThread
{
    Q_OBJECT
public:
    rclcomm();
    void publish_topic(int count);
    void recv_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    HunterDriver m_HunterDriver;    
protected:
    void run();

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
    std::shared_ptr<rclcpp::Node> node;
signals:
    void emitTopicData(QString);
    void emitRobotInfo(QVariantMap qvInfo);
        //AgxControlMode controlmode,AgxVehicleState vehicle_state,uint16_t error_code,float battery_voltage,float linear_velocity,float steering_angule,double m_setInnerAngle);
};

#endif // RCLCOMM_H
