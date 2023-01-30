/**
 * @file /include/launchgui/main_window.hpp
 *
 * @brief Qt based gui for launchgui.
 *
 * @date November 2010
 **/
#ifndef launchgui_MAIN_WINDOW_H
#define launchgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "sc_dialog.hpp" //add

#include <stdlib.h>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace launchgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void rtsp();
    void GPS();
    void Fast();
    void Slow();
    void Start();
    void MD();
    void JOY();
    void Front_CAM();

    void Gmapping();
    void Map_save();
    void Odom();
    void Lidar();
    //void Navigation();
    //void RViz();

    void All_stop();

    void Off_rtsp();
    void Off_GPS();
    void Off_Fast();
    void Off_Slow();
    void Off_MD();
    void Off_JOY();
    void Off_Front_CAM();

    void Off_Gmapping();
    void Off_Odom();
    void Off_Lidar();

    void updateState();
    void updateState_sc();//add

    void getReady();

    void Html();
    void controll_pc_websocket();
    void Edit_html();

    void NUC1_screenshot_clicked(bool checked);
    //void NUC1_screenshot_clicked();
    void NUC2_screenshot_clicked(bool checked);
    void Button_Cmd_go_clicked();


private:
	Ui::MainWindowDesign ui;
  Sc_Dialog *dialog; //add
	QNode qnode;
        QPixmap m_lightimg[2];
        QPixmap m_readyimg[2];

        QPixmap KUDOS_img;
        QPixmap cat1_img;
        QPixmap cat2_img;
  QLineEdit qline;

};

}  // namespace launchgui

#endif // launchgui_MAIN_WINDOW_H
