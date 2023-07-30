#include "hunterdriver.h"
#include <unistd.h>
#include <iostream>
#include <cmath>
HunterDriver::HunterDriver()
{
    l=wheelbase;
    w=track;
}

bool HunterDriver::initRobot()
{
    m_bHunterConnected=false;
    try
    {
        detector.Connect("can0");
        auto proto = detector.DetectProtocolVersion(5);
        if (proto == ProtocolVersion::AGX_V2) {
            std::cout << "Detected protocol: AGX_V2" << std::endl;
            robot = std::unique_ptr<HunterRobot>(
                new HunterRobot(ProtocolVersion::AGX_V2));
        } else {
            std::cout << "Detected protocol: UNKONWN" << std::endl;
        }
        if (robot == nullptr){
            std::cout << "Failed to create robot object" << std::endl;
            return false;
        }
    }
    catch (std::exception error)
    {
        std::cout << "please bringup up can or make sure can port exist"<< std::endl;
        return false;
    }
    robot->Connect("can0");
    robot->EnableCommandedMode();
    m_bHunterConnected=true;
    return true;
}

QVariantMap HunterDriver::GetCurData()
{
    QVariantMap qvmInfo;
    if(m_bHunterConnected && robot != nullptr){
        auto state = robot->GetRobotState();
        m_controlmode=state.system_state.control_mode;  /*0 CONTROL_MODE_RC 1 CONTROL_MODE_CAN 2 CONTROL_MODE_UART*/
        m_vehicle_state=state.system_state.vehicle_state; /*V0 EHICLE_STATE_NORMAL 1 VEHICLE_STATE_ESTOP 2 VEHICLE_STATE_EXCEPTION*/
        m_error_code=state.system_state.error_code;
        m_battery_voltage=state.system_state.battery_voltage;
        m_linear_velocity=state.motion_state.linear_velocity;
        m_steering_angule=state.motion_state.steering_angle;
        qvmInfo["connectionstate"]=QString("Connection State: Connected");
        qvmInfo["controlmode"]= QString("Control Mode: %1").arg(m_controlmode,2,16,QChar(' '));
        qvmInfo["vehiclestate"]= QString("Vehicle State: %1").arg(m_vehicle_state,2,16,QChar(' '));
        qvmInfo["errorcode"]= QString("Error Code: 0X%1").arg(m_error_code,4,16,QChar('0'));
        qvmInfo["batteryvoltage"]= QString("Battery Voltage: %1 V").arg(m_battery_voltage,0,'G',5);
        qvmInfo["linearvelocity"]= QString("linear_velocity: %1 m/s").arg(m_linear_velocity,0,'G',5);
        qvmInfo["steeringangule"]= QString("steering_angle: %1").arg(m_steering_angule,0,'G',5);      
    }    else{
        qvmInfo["connectionstate"]=QString("Connection State: DisConnected");
        qvmInfo["controlmode"]= QString("Control Mode: NA");
        qvmInfo["vehiclestate"]= QString("Vehicle State: NA");
        qvmInfo["errorcode"]= QString("Error Code: NA");
        qvmInfo["batteryvoltage"]= QString("Battery Voltage: NA");
        qvmInfo["linearvelocity"]= QString("linear_velocity: NA");
        qvmInfo["steeringangule"]= QString("steering_angle: NA");
    }
    qvmInfo["autopilot"]=m_bAutoPilot?QString("Enable"):QString("Disable");

    return qvmInfo;

}

double HunterDriver::ConvertCentralAngleToInner(double angle)
{
    double phi = angle;
    double phi_i = 0;
    if (phi > steer_angle_tolerance) {
        // left turn
        phi_i = std::atan(2 * l * std::sin(phi) /
                          (2 * l * std::cos(phi) - w * std::sin(phi)));
    } else if (phi < -steer_angle_tolerance) {
        // right turn
        phi = -phi;
        phi_i = std::atan(2 * l * std::sin(phi) /
                          (2 * l * std::cos(phi) - w * std::sin(phi)));
        phi_i = -phi_i;
    }
    return phi_i;
}

double HunterDriver::SpeedUp()
{
    m_set_velocity+=SPEEDSTEP;
    if(m_set_velocity>max_linear_speed)m_set_velocity=max_linear_speed;
    return m_set_velocity;
}

double HunterDriver::SpeedDown()
{
    m_set_velocity-=SPEEDSTEP;
    if(m_set_velocity<-max_linear_speed)m_set_velocity=-max_linear_speed;
    return m_set_velocity;
}

double HunterDriver::SpeedSet(double setSpeed)
{
    m_set_velocity=setSpeed;
    if(m_set_velocity>max_linear_speed) m_set_velocity=max_linear_speed;
    if(m_set_velocity<-max_linear_speed)m_set_velocity=-max_linear_speed;


    return m_set_velocity;
}

double HunterDriver::angleLeft()
{
    m_set_angle-=ANGLESTEP;
    if(m_set_angle<-max_steer_angle_central)m_set_angle=-max_steer_angle_central;

    m_setInnerAngle=ConvertCentralAngleToInner(m_set_angle);
    return m_set_angle;
}

double HunterDriver::angleRight()
{
    m_set_angle+=ANGLESTEP;
    if(m_set_angle>max_steer_angle_central)m_set_angle=max_steer_angle_central;

    m_setInnerAngle=ConvertCentralAngleToInner(m_set_angle);
    return m_set_angle;
}

double HunterDriver::angleSet(double angle)
{
    m_set_angle=angle;
    if(m_set_angle<-max_steer_angle_central)m_set_angle=-max_steer_angle_central;
    if(m_set_angle>max_steer_angle_central)m_set_angle=max_steer_angle_central;

    m_setInnerAngle=ConvertCentralAngleToInner(m_set_angle);
    return m_set_angle;
}

void HunterDriver::hunterHoldbrake()
{
    if(m_bHunterConnected && robot != nullptr){
        robot->ActivateBrake();
        m_bBreakReleased=false;
    }
}

void HunterDriver::hunterReleasebrake()
{
     if(m_bHunterConnected && robot != nullptr)
    {
        robot->ReleaseBrake();
        m_bBreakReleased=true;
     }
}

void HunterDriver::hunterGo()
{
     if(m_bHunterConnected && robot != nullptr && m_bBreakReleased)
        robot->SetMotionCommand(m_set_velocity,m_setInnerAngle);
}

void HunterDriver::hunterCAN()
{
    if(m_bHunterConnected && robot != nullptr )
        robot->EnableCommandedMode();
}

void HunterDriver::hunterRst()
{
    if(m_bHunterConnected && robot != nullptr )
        robot->ResetRobotState();

}





