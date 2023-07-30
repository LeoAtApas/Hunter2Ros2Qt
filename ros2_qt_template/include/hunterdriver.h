#ifndef HUNTERDRIVER_H
#define HUNTERDRIVER_H

#include <string>
#include <unistd.h>
#include <QVariantMap>

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;


class HunterDriver
{
public:
    HunterDriver();
    bool initRobot();
    QVariantMap GetCurData();
    AgxControlMode m_controlmode;  /*0 CONTROL_MODE_RC 1 CONTROL_MODE_CAN 2 CONTROL_MODE_UART*/
    AgxVehicleState m_vehicle_state; /*V0 EHICLE_STATE_NORMAL 1 VEHICLE_STATE_ESTOP 2 VEHICLE_STATE_EXCEPTION*/
    uint16_t m_error_code;
    float m_battery_voltage;
    float m_linear_velocity;
    float m_steering_angule;
    double m_set_velocity=0;
    double m_set_angle=0;
    double m_setInnerAngle=0;
    static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees
    static constexpr double track =
        0.605;  // in meter (left & right wheel distance)
    static constexpr double wheelbase =
        0.650;  // in meter (front & rear wheel distance)
    static constexpr double wheel_radius = 0.165;              // in meter
    static constexpr double transmission_reduction_rate = 30;  // 1:30

    // from user manual v1.2.6_S P4
    // max linear velocity: 1.5 m/s
    static constexpr double max_steer_angle =
        0.58;  // in rad, 0.75 for inner wheel
    static constexpr double max_steer_angle_central =
        0.461;  // max central angle
    static constexpr double max_linear_speed = 1.5;  // in m/ss
    static constexpr double SPEEDSTEP = 0.01;  //
    static constexpr double ANGLESTEP = 0.01;  //

    double ConvertCentralAngleToInner(double angle);
    double SpeedUp();
    double SpeedDown();
    double SpeedSet(double setSpeed);
    double angleLeft();
    double angleRight();
    double angleSet(double angle);

    void hunterHoldbrake();
    void hunterReleasebrake();
    void hunterGo();
    void hunterCAN();
    void hunterRst();
    bool m_bBreakReleased=false;
    bool m_bAutoPilot=true;
private:
    bool m_bHunterConnected=false;
    ProtocolDetector detector;
    std::unique_ptr<HunterRobot> robot;
    double l,w;// size of hunter2

};

#endif // HUNTERDRIVER_H
