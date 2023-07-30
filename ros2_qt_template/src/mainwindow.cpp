#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QImage img;
    img.load(":/icon/images/APAS.jpg");
    img.scaled(ui->label->width(),ui->label->height());
    ui->label->setPixmap(QPixmap::fromImage(img).scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    // 初始化 ROS2
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);

    // 实例化节点（可以在这里实例化多个节点，用于不同的功能）
    commNode = new rclcomm();
    connect(commNode, SIGNAL(emitTopicData(QString)), this, SLOT(updateTopicInfo(QString)));
    connect(commNode, SIGNAL(emitRobotInfo(QVariantMap)), this, SLOT(updateRobotInfo(QVariantMap)));
    ui->label_info->clear();
}
void MainWindow::updateTopicInfo(QString data){
    ui->label_topic->setText(data);
}

void MainWindow::updateRobotInfo(QVariantMap qvInfo)
{
    ui->label_connection_state->setText(qvInfo.value("connectionstate","Error QVariantMap").toString());
    ui->label_control_mode->setText(qvInfo.value("controlmode","Error QVariantMap").toString());
    ui->label_vehicle_state->setText(qvInfo.value("vehiclestate","Error QVariantMap").toString());
    ui->label_Error_code->setText(qvInfo.value("errorcode","Error QVariantMap").toString());
    ui->label_battery_voltage->setText(qvInfo.value("batteryvoltage","Error QVariantMap").toString());
    ui->label_linear_velocity->setText(qvInfo.value("linearvelocity","Error QVariantMap").toString());
    ui->label_steering_angle->setText(qvInfo.value("steeringangule","Error QVariantMap").toString());
    if(qvInfo.value("autopilot","Disable").toString()=="Enable"){
        ui->pbIsAuto->setChecked(true);
//        ui->pbIsAuto->setText("Enable");
        QPixmap icon1(tr(":/icon/images/enable.jpg"));
        ui->pbIsAuto->setIcon(icon1);
    }
    else {
        ui->pbIsAuto->setChecked(false);
//        ui->pbIsAuto->setText("Disable");
        QPixmap icon1(tr(":/icon/images/disable.jpg"));
        ui->pbIsAuto->setIcon(icon1);
    }

}
MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pbreleasebrake_clicked()
{
    commNode->m_HunterDriver.hunterReleasebrake();
}


void MainWindow::on_pbholdbrake_clicked()
{
    commNode->m_HunterDriver.hunterHoldbrake();
}


void MainWindow::on_pbSpeedUp_clicked()
{
    ui->lineEdit_Speed->setText(QString("%1").arg(commNode->m_HunterDriver.SpeedUp(),0,'G',5));
}


void MainWindow::on_pbSpeedDown_clicked()
{
    ui->lineEdit_Speed->setText(QString("%1").arg(commNode->m_HunterDriver.SpeedDown(),0,'G',5));
}


void MainWindow::on_pbSpeedzero_clicked()
{
    ui->lineEdit_Speed->setText(QString("%1").arg(commNode->m_HunterDriver.SpeedSet(0),0,'G',5));
}



void MainWindow::on_pbDirLeft_clicked()
{
    ui->lineEdit_angle->setText(QString("%1").arg(commNode->m_HunterDriver.angleRight(),0,'G',5));
}


void MainWindow::on_pbDirStraight_clicked()
{
    ui->lineEdit_angle->setText(QString("%1").arg(commNode->m_HunterDriver.angleSet(0),0,'G',5));
}


void MainWindow::on_pbDirRight_clicked()
{
    ui->lineEdit_angle->setText(QString("%1").arg(commNode->m_HunterDriver.angleLeft(),0,'G',5));
}




void MainWindow::on_lineEdit_Speed_textEdited(const QString &arg1)
{
    bool ok;
    double dSetVal;
    dSetVal=arg1.toDouble(&ok);
    if(ok) ui->lineEdit_Speed->setText(QString("%1").arg(commNode->m_HunterDriver.SpeedSet(dSetVal),0,'G',5));


}


void MainWindow::on_lineEdit_angle_textEdited(const QString &arg1)
{
    bool ok;
    double dSetVal;
    dSetVal=arg1.toDouble(&ok);
    if(ok)ui->lineEdit_angle->setText(QString("%1").arg(commNode->m_HunterDriver.angleSet(dSetVal),0,'G',5));
}


void MainWindow::on_pbSwitchCntl_clicked()
{
    commNode->m_HunterDriver.hunterCAN();
}


void MainWindow::on_pbEMG_clicked()
{
    ui->lineEdit_Speed->setText(QString("%1").arg(commNode->m_HunterDriver.SpeedSet(0),0,'G',5));
    commNode->m_HunterDriver.hunterHoldbrake();
    commNode->m_HunterDriver.hunterRst();
}


void MainWindow::on_pbStartAuto_clicked()
{
    commNode->m_HunterDriver.m_bAutoPilot=true;
}


void MainWindow::on_pbreleasebrake_2_clicked()
{
    commNode->m_HunterDriver.m_bAutoPilot=false;
}

