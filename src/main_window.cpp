/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/launchgui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace launchgui {

int ros_topic_data;
bool ros_status_flag = 0;

bool ros_status_flag_cmd = 0;

QString q_command_string;

extern int State[8];
extern int Ready;
extern QImage qt_image;
extern QImage qt_image_gripper;

using namespace Qt;
//using namespace Ui;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
        : QMainWindow(parent)
	, qnode(argc,argv)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  ui.Button_nuc1_screenshot->setCheckable(true);

  dialog = new Sc_Dialog; //add
  //dialog->setModal(true); //add
  //dialog->show();//add

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(updateState()));
    QObject::connect(&qnode, SIGNAL(statusUpdated_sc()), this, SLOT(updateState_sc())); //add
    QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(getReady()));

    QObject::connect(ui.Button_getmission, SIGNAL(clicked()), this, SLOT(launch_getmission()));
    //QObject::connect(ui.Button_Web, SIGNAL(clicked()), this, SLOT(Refresh_Web()));

    QObject::connect(ui.Button_rtsp, SIGNAL(clicked()), this, SLOT(rtsp()));
    QObject::connect(ui.Button_GPS, SIGNAL(clicked()), this, SLOT(GPS()));
    QObject::connect(ui.Button_Fast, SIGNAL(clicked()), this, SLOT(Fast()));
    QObject::connect(ui.Button_Slow, SIGNAL(clicked()), this, SLOT(Slow()));

    QObject::connect(ui.Button_Gmapping, SIGNAL(clicked()), this, SLOT(Gmapping()));
    QObject::connect(ui.Button_Map_save, SIGNAL(clicked()), this, SLOT(Map_save()));
    //QObject::connect(ui.Button_Map_save, SIGNAL(clicked(bool)), this, SLOT(Map_save(bool)));
    QObject::connect(ui.Button_Odom, SIGNAL(clicked()), this, SLOT(Odom()));
    QObject::connect(ui.Button_Lidar, SIGNAL(clicked()), this, SLOT(Lidar()));
    //QObject::connect(ui.Button_Navigation, SIGNAL(clicked()), this, SLOT(Navigation()));
    //QObject::connect(ui.Button_RViz, SIGNAL(clicked()), this, SLOT(RViz()));

    QObject::connect(ui.Button_md, SIGNAL(clicked()), this, SLOT(MD()));
    QObject::connect(ui.Button_Joy, SIGNAL(clicked()), this, SLOT(JOY()));

    QObject::connect(ui.Button_Start, SIGNAL(clicked()), this, SLOT(Start()));    
    QObject::connect(ui.Button_All_stop, SIGNAL(clicked()), this, SLOT(All_stop()));

    QObject::connect(ui.Off_rtsp, SIGNAL(clicked()), this, SLOT(Off_rtsp()));
    QObject::connect(ui.Off_GPS, SIGNAL(clicked()), this, SLOT(Off_GPS()));
    QObject::connect(ui.Off_Fast, SIGNAL(clicked()), this, SLOT(Off_Fast()));
    QObject::connect(ui.Off_Slow, SIGNAL(clicked()), this, SLOT(Off_Slow()));

    QObject::connect(ui.Off_Gmapping, SIGNAL(clicked()), this, SLOT(Off_Gmapping()));
    QObject::connect(ui.Off_Odom, SIGNAL(clicked()), this, SLOT(Off_Odom()));
    QObject::connect(ui.Off_Lidar, SIGNAL(clicked()), this, SLOT(Off_Lidar()));

    QObject::connect(ui.Off_md, SIGNAL(clicked()), this, SLOT(Off_MD()));
    QObject::connect(ui.Off_Joy, SIGNAL(clicked()), this, SLOT(Off_JOY()));

    QObject::connect(ui.Button_Front_CAM, SIGNAL(clicked()), this, SLOT(Front_CAM()));   //Front CAM
    QObject::connect(ui.Off_Front_CAM, SIGNAL(clicked()), this, SLOT(Off_Front_CAM()));  //Front CAM OFf


    //COMM
    QObject::connect(ui.Button_html, SIGNAL(clicked()), this, SLOT(Html()));
    QObject::connect(ui.Button_controll_pc_websocket, SIGNAL(clicked()), this, SLOT(controll_pc_websocket()));
    QObject::connect(ui.Button_Edit_html, SIGNAL(clicked()), this, SLOT(Edit_html()));

    //Screenshot
    QObject::connect(ui.Button_nuc1_screenshot, SIGNAL(clicked(bool)), this, SLOT(NUC1_screenshot_clicked(bool)));
    //QObject::connect(ui.Button_nuc1_screenshot, SIGNAL(clicked()), this, SLOT(NUC1_screenshot_clicked()));
    QObject::connect(ui.Button_nuc2_screenshot, SIGNAL(clicked(bool)), this, SLOT(NUC2_screenshot_clicked(bool)));

    //command_linedeit
    QObject::connect(ui.command_lineEdit, SIGNAL(testChanged(QString)), this, SLOT((testChanged(QString))));
    QObject::connect(ui.Button_Cmd_go, SIGNAL(clicked(bool)), this, SLOT(Button_Cmd_go_clicked()));




    /*********************
    ** Label
    **********************/

   m_lightimg[0].load(":/images/led-off.png");
   m_lightimg[1].load(":/images/led-on.png");

   m_readyimg[0].load(":/images/switch2.jpg");
   m_readyimg[1].load(":/images/switch1.jpg");

   KUDOS_img.load(":/images/KUDOS2.png");


   //cat1_img.load(":/images/cat1.jpg");
   //cat2_img.load(":/images/cat2.jpg");

   //ui.label_7->setPixmap(cat1_img);
   //ui.label_9->setPixmap(cat2_img);
   ui.label_11->setPixmap(KUDOS_img);

}


MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateState() {
    if(State[1] == 1){
        ui.state_label_2->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_2->setPixmap(m_lightimg[0]);
    }

    if(State[2] == 1){
        ui.state_label_3->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_3->setPixmap(m_lightimg[0]);
    }

    if(State[3] == 1){
        ui.state_label_4->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_4->setPixmap(m_lightimg[0]);
    }

    if(State[4] == 1){
        ui.state_label_5->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_5->setPixmap(m_lightimg[0]);
    }

    if(State[5] == 1){
        ui.state_label_6->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_6->setPixmap(m_lightimg[0]);
    }

    if(State[6] == 1){
        ui.state_label_7->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_7->setPixmap(m_lightimg[0]);
    }
    if(State[7] == 1){
      ui.state_label_8->setPixmap(m_lightimg[1]);
      //ui.state_label_9->setPixmap(m_lightimg[1]);
    }
    else{
      ui.state_label_8->setPixmap(m_lightimg[0]);
      //ui.state_label_9->setPixmap(m_lightimg[0]);
    }
    if(State[8] == 1){
        ui.state_label_9->setPixmap(m_lightimg[0]);
      //ui.state_label_9->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_9->setPixmap(m_lightimg[1]);
      //ui.state_label_9->setPixmap(m_lightimg[0]);
    }

    ui.label_8->setPixmap(QPixmap::fromImage(qt_image));
    ui.label_8->resize(ui.label_8->pixmap()->size());

}

void MainWindow::updateState_sc(){
    dialog->setWindowTitle("NUC Screen");
    dialog->show();//add
    dialog->show_screenshot();//add
}

void MainWindow::getReady() {
    if(Ready == 1){
        ui.get_ready->setPixmap(m_readyimg[1]);
    }
    else{
        ui.get_ready->setPixmap(m_readyimg[0]);
    }
}


void MainWindow::rtsp() {
    std::string command= "gnome-terminal -- rosrun rtsp_to_ros rtsp2ros2.py";
    const char *s=command.c_str();
    system(s);
}

void MainWindow::GPS() {
    // GPS = obstacle	
    ros_topic_data = 3;
    ros_status_flag = true;

    ui.mode_ready -> setText("GPS");
}

void MainWindow::Fast() {
    std::string command= "gnome-terminal -- roslaunch rosjoy_to_cmdvel rosjoy_to_cmdvel.launch";
    const char *c=command.c_str();
    system(c);

/*
    std::string command = "gnome-terminal -- adb devices";
    std::string command_2 = "gnome-terminal -- adb forward tcp:20175 tcp:50000";
    std::string command_3 = "gnome-terminal -- rosrun gps_data gps_record.py";

    const char *c=command.c_str();
    const char *c_c=command_2.c_str();
    const char *c_com=command_3.c_str();

    system(c);
    system(c_c);
    system(c_com);
*/
}

void MainWindow::Slow() {
    std::string command= "gnome-terminal -- roslaunch rosjoy_to_cmdvel rosjoy_to_cmdvel_2.launch";
    const char *d=command.c_str();
    system(d);
}

void MainWindow::MD() {
    ros_topic_data = 60;
    ros_status_flag = true;
}

void MainWindow::JOY() {
    ros_topic_data = 70;
    ros_status_flag = true;
}

void MainWindow::Start() {
    ros_topic_data = 0;
    ros_status_flag = true;
}

void MainWindow::All_stop() {
    ros_topic_data = 9;
    ros_status_flag = true;
}

void MainWindow::Front_CAM()
{
  ros_topic_data = 20;
  ros_status_flag = true;
}

void MainWindow::Gmapping()
{
  ros_topic_data = 100;
  ros_status_flag = true;
}

void MainWindow::Map_save()
{
  ros_topic_data = 55;
  ros_status_flag = true;
}


void MainWindow::Odom()
{
  ros_topic_data = 56;
  ros_status_flag = true;
}

void MainWindow::Lidar()
{
  ros_topic_data = 57;
  ros_status_flag = true;
}
/*
void MainWindow::Navigation()
{
  ros_topic_data = 58;
  ros_status_flag = true;
}

void MainWindow::RViz()
{
  ros_topic_data = 59;
  ros_status_flag = true;
}
*/

//COMM
void MainWindow::Html()
{
  //ROS_INFO("Html");
  std::string command_html = "gnome-terminal -- firefox ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927.html";
  const char *c_html = command_html.c_str();
  system(c_html);
}

void MainWindow::controll_pc_websocket()
{
  std::string command_web = "gnome-terminal -- roslaunch rosbridge_server rosbridge_websocket.launch";
  const char *c_web = command_web.c_str();
  system(c_web);
}

void MainWindow::Edit_html()
{
  std::string command_edit = "gedit ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927.html";
  const char *c_edit = command_edit.c_str();
  system(c_edit);
}

//Screenshot
void MainWindow::NUC1_screenshot_clicked(bool checked)
{
  if(checked == true){
    ros_topic_data = 1000;
    ros_status_flag = true;
    //dialog->show();//add
  }
  else{
    dialog->close();//add
  }
}

/*
{
  ros_topic_data = 1000;
  ros_status_flag = true;
}
*/


void MainWindow::NUC2_screenshot_clicked(bool checked)
{
  if(checked == true){
    ros_topic_data = 2000;
    ros_status_flag = true;
    //dialog->show();//add
  }
  else{
    dialog->close();//add

  }
}

void MainWindow::Button_Cmd_go_clicked()
{
  q_command_string = ui.command_lineEdit->text();
  ros_status_flag_cmd = true;
}

/*****************************************************************************
** Stop the selected thing
*****************************************************************************/

void MainWindow::Off_rtsp() {
    ros_topic_data = 11;
    ros_status_flag = true;
}

void MainWindow::Off_GPS() {
    ros_topic_data = 13;
    ros_status_flag = true;
}

void MainWindow::Off_Fast() {
    std::string command_stop= "gnome-terminal -- rosnode kill /joy_node /rosjoy_2_cmdvel_node /teleop_twist_joy";
    const char *of=command_stop.c_str();
    system(of);
}

void MainWindow::Off_Slow() {
    std::string command_stop_2= "gnome-terminal -- rosnode kill /joy_node /rosjoy_1_cmdvel_node /teleop_twist_joy";
    const char *od=command_stop_2.c_str();
    system(od);
}

void MainWindow::Off_MD() {     //md_driver
    ros_topic_data = 61;
    ros_status_flag = true;
}

void MainWindow::Off_JOY() {     //md_driver
    ros_topic_data = 71;
    ros_status_flag = true;
}

void MainWindow::Off_Front_CAM()
{
  ros_topic_data = 21;
  ros_status_flag = true;
}

void MainWindow::Off_Gmapping() {
    ros_topic_data = 85;
    ros_status_flag = true;
}

void MainWindow::Off_Odom() {
    ros_topic_data = 86;
    ros_status_flag = true;
}

void MainWindow::Off_Lidar() {
    ros_topic_data = 87;
    ros_status_flag = true;
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "launchgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://10.30.94.232:11311/")).toString();
    QString host_url = settings.value("host_url", QString("10.30.93.206")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "launchgui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace launchgui

