#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include <Ultrasonic.h>
#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>
<<<<<<< HEAD

#include <Encoder.h>


=======
>>>>>>> origin/master
#include <Bac.h>
#include <Pince.h>
#include <test_contour.h>
#include <CameraServer.h>

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
	Pince pince;
	Bac bac;
	// déclaration des objets
	BaseRoulante BR;
	// déclaration des variables
	int robotMode ;

	void RobotInit() {

		// initialisation des objets et données
		gyro = new ADXRS450_Gyro(); // à connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro
		robotMode = MODE_TANK; // on démarre en mode TANK par défaut
		Joystick1 = new Joystick(0);								// à connecter sur port USB0
		ultraSon_G = new Ultrasonic(8,9,Ultrasonic::kMilliMeters); 	// à connecter sur DIO-0 et DIO-1
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
		std::cout<<gyro->GetAngle()<<endl;
		std::cout<<ultraSon_G->GetRangeMM()<<endl;
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
			//si boutton lever bac
						if(Joystick1->GetRawButton(BTN_BAC_UP))
							bac.leverBac();

						//si bouton abaisser bac
						if(Joystick1->GetRawButton(BTN_BAC_DOWN))
							bac.rentrerBac();

						if(Joystick1->GetRawButton(BTN_SER_PINCE))
						    pince.serrerPince();

						if(Joystick1->GetRawButton(BTN_DESSER_PINCE))
							pince.desserrerPince();

						/*if(Joystick1->GetRawButton(BTN_PINCE_UP))
							pince.leverPince(bac);*/

						if(Joystick1->GetRawButton(BTN_PINCE_DOWN))
							pince.abaisserPince();
		}

		// FOR TEST //
<<<<<<< HEAD
		//double angle=gyro->GetAngle();
		//SmartDashboard::PutString("DB/String 0",std::to_string(angle));
		// END OF TEST
=======
		double angle=gyro->GetAngle();
		SmartDashboard::PutString("DB/String 0",std::to_string(angle));
		//std::cout<<BR.mecaBackLeft.GetDistance()<<endl;
		std::cout<<BR.mecaFrontLeft.GetDistance()<<endl;
		//std::cout<<BR.mecaBackRight.GetDistance()<<endl;
		std::cout<<BR.mecaFrontRight.GetDistance()<<endl;
//ce test sert a tester les encodeurs de chaques moteurs
>>>>>>> origin/master



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

			//Reconnaissance visuelle
			test_contour* p= new test_contour();
			std::shared_ptr<NetworkTable> table;
			table = NetworkTable::GetTable("GRIP/myContoursReport");

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

				//appel fonction reconnaissance visuelle
				p->Process(mat);
							//std::cout << "findBlobsInput = " << std::endl << " " << mat << std::endl << std::endl;
								/*double[] defaultValue= new double[0];
								table.GetNumberArray("CenterX",defaultValue);*/

				//tentative lecture des données renvoyées par fonctions de reconnaissance visuelle
				std::vector<double> arr= table->GetNumberArray("Width", llvm::ArrayRef<double>());
				std::cout<<"avant boucle"<<endl<<arr.size()<<endl;
				for(unsigned int i=0;i<arr.size();i++){
						std::cout<<arr[i]<<""<<endl;
						std::cout<<"dans boucle"<<endl;
				}
			}
		}





};

START_ROBOT_CLASS(Robot)
