#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include <Ultrasonic.h>
#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>

#include <thread>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "Pipeline.h"


class Robot: public frc::IterativeRobot {
public:

	// déclaration des capteurs et actionneurs
	Joystick* Joystick1;
	ADXRS450_Gyro* gyro;
	Ultrasonic* ultraSon_G;
	Ultrasonic* ultraSon_D;
	// déclaration des objets
	BaseRoulante BR;
	// déclaration des variables
	int robotMode ;

	void RobotInit() {

		// initialisation des objets et données
		gyro = new ADXRS450_Gyro(); 								// à connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro
		robotMode = MODE_TANK; // on démarre en mode TANK par défaut
		Joystick1 = new Joystick(0);								// à connecter sur port USB0
		ultraSon_G = new Ultrasonic(0,1,Ultrasonic::kMilliMeters); 	// à connecter sur DIO-0 et DIO-1
		ultraSon_D = new Ultrasonic(2,3,Ultrasonic::kMilliMeters); 	// à connecter sur DIO-2 et DIO-3

		//lancement de la video
		std::thread visionThread(VisionThread);
		visionThread.detach();

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		// si appui sur bouton depose_roue_auto:
		if(Joystick1->GetRawButton(BTN_DEPOSE_ROUE_AUTO)){
			// gestion du depot de roue en mode automatique
			BR.deposeRoueAuto(Joystick1,gyro,ultraSon_G,ultraSon_D);
		}
		else{

			BR.resetModeAuto();


			// Si selection du mode deplacement TANK
			if(Joystick1->GetRawButton(BTN_TANK))
			{
				BR.setRobotMode(MODE_TANK);
			}

			// Si selection du mode deplacement MECA
			if(Joystick1->GetRawButton(BTN_MECA))
			{
				BR.setRobotMode(MODE_MECA);
			}

			// Si mouvement du Joystik
			if (Joystick1->GetX() || Joystick1->GetY() || Joystick1->GetZ() )
			{
				BR.mvtJoystick(Joystick1,gyro);
			}
		}

		// FOR TEST //
		double angle=gyro->GetAngle();
		SmartDashboard::PutString("DB/String 0",std::to_string(angle));
		// END OF TEST

	}

	void TestPeriodic() {

		lw->Run();

	}


private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();

	static void VisionThread() {
		// Get the USB camera from CameraServer
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(); // ("cam0");
		// Set the resolution
		camera.SetResolution(640, 480);

		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		cs::CvSource outputStream = CameraServer::GetInstance()->
				PutVideo("Rectangle", 640, 480);

		// Mats are very memory expensive. Lets reuse this Mat.
		cv::Mat mat;

		// FRED TEST
		//Pipeline* myPipe = new Pipeline();


		while (true) {
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.GrabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
					cv::Scalar(255, 255, 255), 5);
			// Give the output stream a new image to display

			// FRED MESSAGE
			/*if(BR.getRobotMode() == MODE_TANK)
				putText(mat,"Mode TANK",cv::Point(140,140),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255, 255, 255));
			else
				putText(mat,"Mode MECANUM",cv::Point(140,140),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255, 255, 255));
			*/

			outputStream.PutFrame(mat);

			// FRED TEST
			//myPipe->Process();
		}
	}
};

START_ROBOT_CLASS(Robot)
