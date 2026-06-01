/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGroupBox *groupBox_manual;
    QPushButton *btnForward;
    QPushButton *btnLeft;
    QPushButton *btnStop;
    QPushButton *btnRight;
    QPushButton *btnBack;
    QTabWidget *tabWidget;
    QWidget *tab;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QComboBox *comboBox;
    QWidget *tab_2;
    QPushButton *btn_SlamToolBox;
    QPushButton *btn_SaveMap;
    QWidget *tab_3;
    QPushButton *btnStop_lidar_2;
    QWidget *tab_4;
    QWidget *tab_5;
    QGroupBox *groupBox_manual_2;
    QPushButton *btnRvizUp;
    QPushButton *btnRvizLeft;
    QPushButton *btnRvizRight;
    QPushButton *btnRvizDown;
    QPushButton *btnRvizUp_2;
    QPushButton *btnRvizUp_3;
    QPushButton *btnRvizUp_4;
    QPushButton *btn_RvizRotateRight;
    QPushButton *btnRvizResetView;
    QPushButton *btnRvizZoomOut;
    QPushButton *btnRvizZoomIn;
    QPushButton *btn_LoadRobot;
    QGroupBox *groupBox;
    QPushButton *btn_FixFrame;
    QComboBox *comboBox_FixFrame;
    QGroupBox *groupBox_3;
    QPushButton *btn_LaserScan;
    QComboBox *comboBox_Laser;
    QGroupBox *groupBox_4;
    QPushButton *btn_Maps;
    QComboBox *comboBox_Maps;
    QCheckBox *checkBox_TF;
    QPushButton *btnRvizReset;
    QPushButton *btn_LoadMap;
    QCheckBox *checkBox_UseSimTime;
    QPushButton *btn_StopSlam;
    QGroupBox *groupBox_pose;
    QLabel *label_pose;
    QLineEdit *lineEditX;
    QLineEdit *lineEditY;
    QLineEdit *lineEditTheta;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QLineEdit *lineEdit_Lin;
    QSlider *Slider_Lin;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_7;
    QLineEdit *lineEdit_ip;
    QPushButton *btnConnect;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_8;
    QLineEdit *lineEdit_port;
    QLabel *label_statustcp;
    QPushButton *btnDisConnect;
    QPushButton *btnConnect_lidar;
    QPushButton *btnStop_lidar;
    QTextEdit *textEdit_log;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_Rviz;
    QWidget *rvizWidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1280, 720);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        groupBox_manual = new QGroupBox(centralwidget);
        groupBox_manual->setObjectName(QString::fromUtf8("groupBox_manual"));
        groupBox_manual->setEnabled(true);
        groupBox_manual->setGeometry(QRect(910, 10, 301, 161));
        groupBox_manual->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        groupBox_manual->setFlat(false);
        groupBox_manual->setCheckable(false);
        btnForward = new QPushButton(groupBox_manual);
        btnForward->setObjectName(QString::fromUtf8("btnForward"));
        btnForward->setGeometry(QRect(110, 0, 81, 41));
        btnLeft = new QPushButton(groupBox_manual);
        btnLeft->setObjectName(QString::fromUtf8("btnLeft"));
        btnLeft->setGeometry(QRect(10, 60, 81, 41));
        btnStop = new QPushButton(groupBox_manual);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setGeometry(QRect(110, 60, 81, 41));
        btnRight = new QPushButton(groupBox_manual);
        btnRight->setObjectName(QString::fromUtf8("btnRight"));
        btnRight->setGeometry(QRect(210, 60, 81, 41));
        btnBack = new QPushButton(groupBox_manual);
        btnBack->setObjectName(QString::fromUtf8("btnBack"));
        btnBack->setGeometry(QRect(110, 120, 81, 41));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(690, 180, 591, 311));
        tabWidget->setStyleSheet(QString::fromUtf8(""));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(20, 50, 89, 41));
        pushButton_2 = new QPushButton(tab);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(140, 50, 89, 41));
        comboBox = new QComboBox(tab);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(20, 10, 211, 25));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        btn_SlamToolBox = new QPushButton(tab_2);
        btn_SlamToolBox->setObjectName(QString::fromUtf8("btn_SlamToolBox"));
        btn_SlamToolBox->setGeometry(QRect(10, 10, 101, 51));
        btn_SaveMap = new QPushButton(tab_2);
        btn_SaveMap->setObjectName(QString::fromUtf8("btn_SaveMap"));
        btn_SaveMap->setGeometry(QRect(120, 10, 101, 51));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        btnStop_lidar_2 = new QPushButton(tab_3);
        btnStop_lidar_2->setObjectName(QString::fromUtf8("btnStop_lidar_2"));
        btnStop_lidar_2->setGeometry(QRect(40, 30, 101, 41));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        groupBox_manual_2 = new QGroupBox(tab_5);
        groupBox_manual_2->setObjectName(QString::fromUtf8("groupBox_manual_2"));
        groupBox_manual_2->setEnabled(true);
        groupBox_manual_2->setGeometry(QRect(280, 10, 290, 141));
        groupBox_manual_2->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        groupBox_manual_2->setFlat(false);
        groupBox_manual_2->setCheckable(false);
        btnRvizUp = new QPushButton(groupBox_manual_2);
        btnRvizUp->setObjectName(QString::fromUtf8("btnRvizUp"));
        btnRvizUp->setGeometry(QRect(70, 0, 61, 41));
        btnRvizLeft = new QPushButton(groupBox_manual_2);
        btnRvizLeft->setObjectName(QString::fromUtf8("btnRvizLeft"));
        btnRvizLeft->setGeometry(QRect(0, 50, 61, 41));
        btnRvizRight = new QPushButton(groupBox_manual_2);
        btnRvizRight->setObjectName(QString::fromUtf8("btnRvizRight"));
        btnRvizRight->setGeometry(QRect(140, 50, 61, 41));
        btnRvizDown = new QPushButton(groupBox_manual_2);
        btnRvizDown->setObjectName(QString::fromUtf8("btnRvizDown"));
        btnRvizDown->setGeometry(QRect(70, 100, 61, 41));
        btnRvizUp_2 = new QPushButton(groupBox_manual_2);
        btnRvizUp_2->setObjectName(QString::fromUtf8("btnRvizUp_2"));
        btnRvizUp_2->setGeometry(QRect(0, 0, 61, 41));
        btnRvizUp_3 = new QPushButton(groupBox_manual_2);
        btnRvizUp_3->setObjectName(QString::fromUtf8("btnRvizUp_3"));
        btnRvizUp_3->setGeometry(QRect(0, 100, 61, 41));
        btnRvizUp_4 = new QPushButton(groupBox_manual_2);
        btnRvizUp_4->setObjectName(QString::fromUtf8("btnRvizUp_4"));
        btnRvizUp_4->setGeometry(QRect(140, 100, 61, 41));
        btn_RvizRotateRight = new QPushButton(groupBox_manual_2);
        btn_RvizRotateRight->setObjectName(QString::fromUtf8("btn_RvizRotateRight"));
        btn_RvizRotateRight->setGeometry(QRect(140, 0, 61, 41));
        btnRvizResetView = new QPushButton(groupBox_manual_2);
        btnRvizResetView->setObjectName(QString::fromUtf8("btnRvizResetView"));
        btnRvizResetView->setGeometry(QRect(70, 50, 61, 41));
        btnRvizZoomOut = new QPushButton(groupBox_manual_2);
        btnRvizZoomOut->setObjectName(QString::fromUtf8("btnRvizZoomOut"));
        btnRvizZoomOut->setGeometry(QRect(220, 0, 70, 41));
        btnRvizZoomIn = new QPushButton(groupBox_manual_2);
        btnRvizZoomIn->setObjectName(QString::fromUtf8("btnRvizZoomIn"));
        btnRvizZoomIn->setGeometry(QRect(220, 100, 70, 41));
        btn_LoadRobot = new QPushButton(tab_5);
        btn_LoadRobot->setObjectName(QString::fromUtf8("btn_LoadRobot"));
        btn_LoadRobot->setGeometry(QRect(0, 0, 91, 41));
        groupBox = new QGroupBox(tab_5);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(0, 50, 221, 41));
        groupBox->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        btn_FixFrame = new QPushButton(groupBox);
        btn_FixFrame->setObjectName(QString::fromUtf8("btn_FixFrame"));
        btn_FixFrame->setGeometry(QRect(0, 0, 80, 31));
        comboBox_FixFrame = new QComboBox(groupBox);
        comboBox_FixFrame->addItem(QString());
        comboBox_FixFrame->setObjectName(QString::fromUtf8("comboBox_FixFrame"));
        comboBox_FixFrame->setGeometry(QRect(80, 0, 141, 31));
        comboBox_FixFrame->setEditable(true);
        groupBox_3 = new QGroupBox(tab_5);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(0, 90, 221, 41));
        groupBox_3->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        btn_LaserScan = new QPushButton(groupBox_3);
        btn_LaserScan->setObjectName(QString::fromUtf8("btn_LaserScan"));
        btn_LaserScan->setGeometry(QRect(0, 0, 80, 31));
        comboBox_Laser = new QComboBox(groupBox_3);
        comboBox_Laser->addItem(QString());
        comboBox_Laser->setObjectName(QString::fromUtf8("comboBox_Laser"));
        comboBox_Laser->setGeometry(QRect(80, 0, 141, 31));
        comboBox_Laser->setEditable(true);
        groupBox_4 = new QGroupBox(tab_5);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(0, 130, 221, 41));
        groupBox_4->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        btn_Maps = new QPushButton(groupBox_4);
        btn_Maps->setObjectName(QString::fromUtf8("btn_Maps"));
        btn_Maps->setGeometry(QRect(0, 0, 80, 31));
        comboBox_Maps = new QComboBox(groupBox_4);
        comboBox_Maps->addItem(QString());
        comboBox_Maps->setObjectName(QString::fromUtf8("comboBox_Maps"));
        comboBox_Maps->setGeometry(QRect(80, 0, 141, 31));
        comboBox_Maps->setEditable(true);
        checkBox_TF = new QCheckBox(tab_5);
        checkBox_TF->setObjectName(QString::fromUtf8("checkBox_TF"));
        checkBox_TF->setGeometry(QRect(180, 10, 41, 23));
        checkBox_TF->setLayoutDirection(Qt::RightToLeft);
        btnRvizReset = new QPushButton(tab_5);
        btnRvizReset->setObjectName(QString::fromUtf8("btnRvizReset"));
        btnRvizReset->setGeometry(QRect(0, 250, 91, 31));
        btn_LoadMap = new QPushButton(tab_5);
        btn_LoadMap->setObjectName(QString::fromUtf8("btn_LoadMap"));
        btn_LoadMap->setGeometry(QRect(0, 170, 81, 31));
        checkBox_UseSimTime = new QCheckBox(tab_5);
        checkBox_UseSimTime->setObjectName(QString::fromUtf8("checkBox_UseSimTime"));
        checkBox_UseSimTime->setGeometry(QRect(100, 170, 121, 31));
        checkBox_UseSimTime->setLayoutDirection(Qt::RightToLeft);
        btn_StopSlam = new QPushButton(tab_5);
        btn_StopSlam->setObjectName(QString::fromUtf8("btn_StopSlam"));
        btn_StopSlam->setGeometry(QRect(100, 250, 111, 31));
        tabWidget->addTab(tab_5, QString());
        groupBox_pose = new QGroupBox(centralwidget);
        groupBox_pose->setObjectName(QString::fromUtf8("groupBox_pose"));
        groupBox_pose->setGeometry(QRect(720, 0, 131, 145));
        groupBox_pose->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        label_pose = new QLabel(groupBox_pose);
        label_pose->setObjectName(QString::fromUtf8("label_pose"));
        label_pose->setGeometry(QRect(10, 10, 91, 17));
        lineEditX = new QLineEdit(groupBox_pose);
        lineEditX->setObjectName(QString::fromUtf8("lineEditX"));
        lineEditX->setGeometry(QRect(70, 30, 51, 25));
        lineEditY = new QLineEdit(groupBox_pose);
        lineEditY->setObjectName(QString::fromUtf8("lineEditY"));
        lineEditY->setGeometry(QRect(70, 70, 51, 25));
        lineEditTheta = new QLineEdit(groupBox_pose);
        lineEditTheta->setObjectName(QString::fromUtf8("lineEditTheta"));
        lineEditTheta->setGeometry(QRect(70, 110, 51, 25));
        label_4 = new QLabel(groupBox_pose);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 30, 41, 17));
        label_5 = new QLabel(groupBox_pose);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 70, 51, 17));
        label_6 = new QLabel(groupBox_pose);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 110, 51, 17));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(390, 10, 301, 50));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        lineEdit_Lin = new QLineEdit(layoutWidget);
        lineEdit_Lin->setObjectName(QString::fromUtf8("lineEdit_Lin"));

        horizontalLayout->addWidget(lineEdit_Lin);


        verticalLayout_3->addLayout(horizontalLayout);

        Slider_Lin = new QSlider(layoutWidget);
        Slider_Lin->setObjectName(QString::fromUtf8("Slider_Lin"));
        Slider_Lin->setMinimum(0);
        Slider_Lin->setMaximum(100);
        Slider_Lin->setSingleStep(10);
        Slider_Lin->setPageStep(10);
        Slider_Lin->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(Slider_Lin);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 10, 301, 62));
        verticalLayout_4 = new QVBoxLayout(layoutWidget1);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_7 = new QLabel(layoutWidget1);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_2->addWidget(label_7);

        lineEdit_ip = new QLineEdit(layoutWidget1);
        lineEdit_ip->setObjectName(QString::fromUtf8("lineEdit_ip"));

        horizontalLayout_2->addWidget(lineEdit_ip);

        btnConnect = new QPushButton(layoutWidget1);
        btnConnect->setObjectName(QString::fromUtf8("btnConnect"));

        horizontalLayout_2->addWidget(btnConnect);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_8 = new QLabel(layoutWidget1);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_3->addWidget(label_8);

        lineEdit_port = new QLineEdit(layoutWidget1);
        lineEdit_port->setObjectName(QString::fromUtf8("lineEdit_port"));

        horizontalLayout_3->addWidget(lineEdit_port);

        label_statustcp = new QLabel(layoutWidget1);
        label_statustcp->setObjectName(QString::fromUtf8("label_statustcp"));
        label_statustcp->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(label_statustcp);

        btnDisConnect = new QPushButton(layoutWidget1);
        btnDisConnect->setObjectName(QString::fromUtf8("btnDisConnect"));

        horizontalLayout_3->addWidget(btnDisConnect);


        verticalLayout_4->addLayout(horizontalLayout_3);

        btnConnect_lidar = new QPushButton(centralwidget);
        btnConnect_lidar->setObjectName(QString::fromUtf8("btnConnect_lidar"));
        btnConnect_lidar->setGeometry(QRect(10, 80, 101, 41));
        btnStop_lidar = new QPushButton(centralwidget);
        btnStop_lidar->setObjectName(QString::fromUtf8("btnStop_lidar"));
        btnStop_lidar->setGeometry(QRect(120, 80, 101, 41));
        textEdit_log = new QTextEdit(centralwidget);
        textEdit_log->setObjectName(QString::fromUtf8("textEdit_log"));
        textEdit_log->setGeometry(QRect(690, 490, 591, 171));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 130, 691, 541));
        verticalLayout_Rviz = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_Rviz->setObjectName(QString::fromUtf8("verticalLayout_Rviz"));
        verticalLayout_Rviz->setContentsMargins(0, 0, 0, 0);
        rvizWidget = new QWidget(verticalLayoutWidget);
        rvizWidget->setObjectName(QString::fromUtf8("rvizWidget"));

        verticalLayout_Rviz->addWidget(rvizWidget);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1280, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(4);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        groupBox_manual->setTitle(QString());
        btnForward->setText(QCoreApplication::translate("MainWindow", "forward", nullptr));
        btnLeft->setText(QCoreApplication::translate("MainWindow", "left", nullptr));
        btnStop->setText(QCoreApplication::translate("MainWindow", "stop", nullptr));
        btnRight->setText(QCoreApplication::translate("MainWindow", "right", nullptr));
        btnBack->setText(QCoreApplication::translate("MainWindow", "back", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Send Goal", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "Cancel Goal", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("MainWindow", "Home", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("MainWindow", "Corner", nullptr));

        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Auto Mode", nullptr));
        btn_SlamToolBox->setText(QCoreApplication::translate("MainWindow", "Slam ToolBox", nullptr));
        btn_SaveMap->setText(QCoreApplication::translate("MainWindow", "Save Map", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        btnStop_lidar_2->setText(QCoreApplication::translate("MainWindow", "A* NAV", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "Planning", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "Status", nullptr));
        groupBox_manual_2->setTitle(QString());
        btnRvizUp->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        btnRvizLeft->setText(QCoreApplication::translate("MainWindow", "<", nullptr));
        btnRvizRight->setText(QCoreApplication::translate("MainWindow", ">", nullptr));
        btnRvizDown->setText(QCoreApplication::translate("MainWindow", "v", nullptr));
        btnRvizUp_2->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        btnRvizUp_3->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        btnRvizUp_4->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        btn_RvizRotateRight->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        btnRvizResetView->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
        btnRvizZoomOut->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        btnRvizZoomIn->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
        btn_LoadRobot->setText(QCoreApplication::translate("MainWindow", "Load Robot", nullptr));
        groupBox->setTitle(QString());
        btn_FixFrame->setText(QCoreApplication::translate("MainWindow", "Fix Frame", nullptr));
        comboBox_FixFrame->setItemText(0, QCoreApplication::translate("MainWindow", "map", nullptr));

        groupBox_3->setTitle(QString());
        btn_LaserScan->setText(QCoreApplication::translate("MainWindow", "LaserScan", nullptr));
        comboBox_Laser->setItemText(0, QCoreApplication::translate("MainWindow", "/scan", nullptr));

        groupBox_4->setTitle(QString());
        btn_Maps->setText(QCoreApplication::translate("MainWindow", "Map", nullptr));
        comboBox_Maps->setItemText(0, QCoreApplication::translate("MainWindow", "/map", nullptr));

        checkBox_TF->setText(QCoreApplication::translate("MainWindow", "TF", nullptr));
        btnRvizReset->setText(QCoreApplication::translate("MainWindow", "ResetRViz", nullptr));
        btn_LoadMap->setText(QCoreApplication::translate("MainWindow", "Load Map", nullptr));
        checkBox_UseSimTime->setText(QCoreApplication::translate("MainWindow", "Use Sim Time", nullptr));
        btn_StopSlam->setText(QCoreApplication::translate("MainWindow", "Kill Slam/Map", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QCoreApplication::translate("MainWindow", "Rviz", nullptr));
        groupBox_pose->setTitle(QString());
        label_pose->setText(QCoreApplication::translate("MainWindow", "Robot Pose:", nullptr));
        lineEditX->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        lineEditY->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        lineEditTheta->setText(QCoreApplication::translate("MainWindow", "0.0", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "X :", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Y :", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Theta :", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Linear Vel:", nullptr));
        lineEdit_Lin->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "TCP IP:", nullptr));
        lineEdit_ip->setText(QCoreApplication::translate("MainWindow", "192.168.0.105", nullptr));
        btnConnect->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Port:", nullptr));
        lineEdit_port->setText(QCoreApplication::translate("MainWindow", "1234", nullptr));
        label_statustcp->setText(QCoreApplication::translate("MainWindow", "DisConnected", nullptr));
        btnDisConnect->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        btnConnect_lidar->setText(QCoreApplication::translate("MainWindow", "Connect Lidar", nullptr));
        btnStop_lidar->setText(QCoreApplication::translate("MainWindow", "Stop Lidar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
