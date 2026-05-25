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
    QWidget *tab_3;
    QWidget *tab_4;
    QWidget *tab_5;
    QGroupBox *groupBox_manual_2;
    QPushButton *forward_2;
    QPushButton *left_2;
    QPushButton *Reset;
    QPushButton *right_2;
    QPushButton *back_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_6;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *lineEdit_3;
    QPushButton *pushButton_5;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox_3;
    QGroupBox *groupBox_pose;
    QLabel *label_pose;
    QLineEdit *lineEditX;
    QLineEdit *lineEditY;
    QLineEdit *lineEditTheta;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QLineEdit *lineEdit_Lin;
    QSlider *Slider_Lin;
    QWidget *layoutWidget3;
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
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_5;
    QWidget *rvizWidget;
    QPushButton *btnStop_lidar;
    QTextEdit *textEdit_log;
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
        groupBox_manual->setGeometry(QRect(60, 10, 340, 160));
        groupBox_manual->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        groupBox_manual->setFlat(false);
        groupBox_manual->setCheckable(false);
        btnForward = new QPushButton(groupBox_manual);
        btnForward->setObjectName(QString::fromUtf8("btnForward"));
        btnForward->setGeometry(QRect(120, 10, 100, 40));
        btnLeft = new QPushButton(groupBox_manual);
        btnLeft->setObjectName(QString::fromUtf8("btnLeft"));
        btnLeft->setGeometry(QRect(10, 60, 100, 40));
        btnStop = new QPushButton(groupBox_manual);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setGeometry(QRect(120, 60, 100, 40));
        btnRight = new QPushButton(groupBox_manual);
        btnRight->setObjectName(QString::fromUtf8("btnRight"));
        btnRight->setGeometry(QRect(230, 60, 100, 40));
        btnBack = new QPushButton(groupBox_manual);
        btnBack->setObjectName(QString::fromUtf8("btnBack"));
        btnBack->setGeometry(QRect(120, 110, 100, 40));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 180, 551, 411));
        tabWidget->setStyleSheet(QString::fromUtf8(""));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(20, 50, 89, 25));
        pushButton_2 = new QPushButton(tab);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(140, 50, 89, 25));
        comboBox = new QComboBox(tab);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(20, 10, 211, 25));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        groupBox_manual_2 = new QGroupBox(tab_5);
        groupBox_manual_2->setObjectName(QString::fromUtf8("groupBox_manual_2"));
        groupBox_manual_2->setEnabled(true);
        groupBox_manual_2->setGeometry(QRect(0, 10, 250, 110));
        groupBox_manual_2->setStyleSheet(QString::fromUtf8("QGroupBox {\n"
"    border: none;\n"
"}"));
        groupBox_manual_2->setFlat(false);
        groupBox_manual_2->setCheckable(false);
        forward_2 = new QPushButton(groupBox_manual_2);
        forward_2->setObjectName(QString::fromUtf8("forward_2"));
        forward_2->setGeometry(QRect(90, 0, 70, 30));
        left_2 = new QPushButton(groupBox_manual_2);
        left_2->setObjectName(QString::fromUtf8("left_2"));
        left_2->setGeometry(QRect(10, 40, 70, 30));
        Reset = new QPushButton(groupBox_manual_2);
        Reset->setObjectName(QString::fromUtf8("Reset"));
        Reset->setGeometry(QRect(90, 40, 70, 30));
        right_2 = new QPushButton(groupBox_manual_2);
        right_2->setObjectName(QString::fromUtf8("right_2"));
        right_2->setGeometry(QRect(170, 40, 70, 30));
        back_2 = new QPushButton(groupBox_manual_2);
        back_2->setObjectName(QString::fromUtf8("back_2"));
        back_2->setGeometry(QRect(90, 80, 70, 30));
        pushButton_3 = new QPushButton(tab_5);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(280, 10, 70, 25));
        pushButton_4 = new QPushButton(tab_5);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(280, 90, 70, 25));
        pushButton_6 = new QPushButton(tab_5);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setGeometry(QRect(60, 140, 142, 25));
        layoutWidget = new QWidget(tab_5);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(380, 10, 144, 58));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        lineEdit_3 = new QLineEdit(layoutWidget);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        verticalLayout_2->addWidget(lineEdit_3);

        pushButton_5 = new QPushButton(layoutWidget);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));

        verticalLayout_2->addWidget(pushButton_5);

        layoutWidget1 = new QWidget(tab_5);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 210, 59, 83));
        verticalLayout = new QVBoxLayout(layoutWidget1);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        checkBox = new QCheckBox(layoutWidget1);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        verticalLayout->addWidget(checkBox);

        checkBox_2 = new QCheckBox(layoutWidget1);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));

        verticalLayout->addWidget(checkBox_2);

        checkBox_3 = new QCheckBox(layoutWidget1);
        checkBox_3->setObjectName(QString::fromUtf8("checkBox_3"));

        verticalLayout->addWidget(checkBox_3);

        tabWidget->addTab(tab_5, QString());
        groupBox_pose = new QGroupBox(centralwidget);
        groupBox_pose->setObjectName(QString::fromUtf8("groupBox_pose"));
        groupBox_pose->setGeometry(QRect(430, 0, 131, 145));
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
        layoutWidget2 = new QWidget(centralwidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(600, 10, 301, 50));
        verticalLayout_3 = new QVBoxLayout(layoutWidget2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_2 = new QLabel(layoutWidget2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        lineEdit_Lin = new QLineEdit(layoutWidget2);
        lineEdit_Lin->setObjectName(QString::fromUtf8("lineEdit_Lin"));

        horizontalLayout->addWidget(lineEdit_Lin);


        verticalLayout_3->addLayout(horizontalLayout);

        Slider_Lin = new QSlider(layoutWidget2);
        Slider_Lin->setObjectName(QString::fromUtf8("Slider_Lin"));
        Slider_Lin->setMinimum(0);
        Slider_Lin->setMaximum(100);
        Slider_Lin->setSingleStep(10);
        Slider_Lin->setPageStep(10);
        Slider_Lin->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(Slider_Lin);

        layoutWidget3 = new QWidget(centralwidget);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(970, 10, 301, 62));
        verticalLayout_4 = new QVBoxLayout(layoutWidget3);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_7 = new QLabel(layoutWidget3);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_2->addWidget(label_7);

        lineEdit_ip = new QLineEdit(layoutWidget3);
        lineEdit_ip->setObjectName(QString::fromUtf8("lineEdit_ip"));

        horizontalLayout_2->addWidget(lineEdit_ip);

        btnConnect = new QPushButton(layoutWidget3);
        btnConnect->setObjectName(QString::fromUtf8("btnConnect"));

        horizontalLayout_2->addWidget(btnConnect);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_8 = new QLabel(layoutWidget3);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_3->addWidget(label_8);

        lineEdit_port = new QLineEdit(layoutWidget3);
        lineEdit_port->setObjectName(QString::fromUtf8("lineEdit_port"));

        horizontalLayout_3->addWidget(lineEdit_port);

        label_statustcp = new QLabel(layoutWidget3);
        label_statustcp->setObjectName(QString::fromUtf8("label_statustcp"));
        label_statustcp->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(label_statustcp);

        btnDisConnect = new QPushButton(layoutWidget3);
        btnDisConnect->setObjectName(QString::fromUtf8("btnDisConnect"));

        horizontalLayout_3->addWidget(btnDisConnect);


        verticalLayout_4->addLayout(horizontalLayout_3);

        btnConnect_lidar = new QPushButton(centralwidget);
        btnConnect_lidar->setObjectName(QString::fromUtf8("btnConnect_lidar"));
        btnConnect_lidar->setGeometry(QRect(970, 80, 111, 41));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(610, 140, 661, 531));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        rvizWidget = new QWidget(verticalLayoutWidget);
        rvizWidget->setObjectName(QString::fromUtf8("rvizWidget"));

        verticalLayout_5->addWidget(rvizWidget);

        btnStop_lidar = new QPushButton(centralwidget);
        btnStop_lidar->setObjectName(QString::fromUtf8("btnStop_lidar"));
        btnStop_lidar->setGeometry(QRect(1160, 80, 111, 41));
        textEdit_log = new QTextEdit(centralwidget);
        textEdit_log->setObjectName(QString::fromUtf8("textEdit_log"));
        textEdit_log->setGeometry(QRect(10, 590, 551, 81));
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
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "Planning", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "Status", nullptr));
        groupBox_manual_2->setTitle(QString());
        forward_2->setText(QCoreApplication::translate("MainWindow", "^", nullptr));
        left_2->setText(QCoreApplication::translate("MainWindow", "<", nullptr));
        Reset->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
        right_2->setText(QCoreApplication::translate("MainWindow", ">", nullptr));
        back_2->setText(QCoreApplication::translate("MainWindow", "v", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        pushButton_4->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "Initial Pose", nullptr));
        lineEdit_3->setText(QCoreApplication::translate("MainWindow", "base_footprint", nullptr));
        pushButton_5->setText(QCoreApplication::translate("MainWindow", "Fix Frame", nullptr));
        checkBox->setText(QCoreApplication::translate("MainWindow", "TF", nullptr));
        checkBox_2->setText(QCoreApplication::translate("MainWindow", "Scan", nullptr));
        checkBox_3->setText(QCoreApplication::translate("MainWindow", "Map", nullptr));
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
