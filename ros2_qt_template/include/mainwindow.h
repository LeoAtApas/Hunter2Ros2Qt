#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcomm.h"
#include <iostream>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    rclcomm *commNode;
public slots:
    void updateTopicInfo(QString);
    void updateRobotInfo(QVariantMap qvInfo);
private slots:
    void on_pbreleasebrake_clicked();
    void on_pbholdbrake_clicked();
    void on_pbSpeedUp_clicked();
    void on_pbSpeedDown_clicked();
    void on_pbSpeedzero_clicked();

    void on_pbDirLeft_clicked();
    void on_pbDirStraight_clicked();
    void on_pbDirRight_clicked();

    void on_lineEdit_Speed_textEdited(const QString &arg1);
    void on_lineEdit_angle_textEdited(const QString &arg1);
    void on_pbSwitchCntl_clicked();
    void on_pbEMG_clicked();
    void on_pbStartAuto_clicked();
    void on_pbreleasebrake_2_clicked();
};
#endif // MAINWINDOW_H
