#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include <Ultrasonic.h>
#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>
#include "Bac.h"
#include "Pince.h"

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
#include "WPILib.h"

enum type_etape {AUCUN, FIN, AVANCER, TOURNER, TIRER, ATTENDRE};

struct etape{
	float param;
	enum type_etape type;
};




struct etape Tableau_Actions[] {
		{100, AVANCER},
		/*{-45,TOURNER},
		{2000,AVANCER},
		{-0.8f, TIRER},
		{2, ATTENDRE},
		{0, TIRER},*/
		{0,FIN}
};

class Robot: public frc::IterativeRobot {
public:

	// dï¿½claration des capteurs et actionneurs
	Joystick* Joystick1;
	ADXRS450_Gyro* gyro;
	Ultrasonic* ultraSon_G;
	Ultrasonic* ultraSon_D;


	BaseRoulante BR;
	// dï¿½claration des variables
	Bac bac;
	Pince pince;
	int robotMode ;
	int etape_actuelle;
	int etape_suivante;
	double position_x = 0;
	double position_y = 0;

	double ecart_roues_largeur_mm = 1100;  //740

	void RobotInit() {

		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro
		robotMode = MODE_TANK; // on dï¿½marre en mode TANK par dï¿½faut
		Joystick1 = new Joystick(0);								// ï¿½ connecter sur port USB0
		ultraSon_G = new Ultrasonic(0,1,Ultrasonic::kMilliMeters); 	// ï¿½ connecter sur DIO-0 et DIO-1
		ultraSon_D = new Ultrasonic(2,3,Ultrasonic::kMilliMeters); 	// ï¿½ connecter sur DIO-2 et DIO-3


		//lancement de la video
//		std::thread visionThread(VisionThread);
//		visionThread.detach();

	}

	void etapeSuivante()
		{
			etape_actuelle=etape_suivante;
			double angle, distance;
			switch(Tableau_Actions[etape_actuelle].type)
			{
			case AVANCER:
				BR.parcourirDistance(
						Tableau_Actions[etape_actuelle].param,
						Tableau_Actions[etape_actuelle].param);
				etape_suivante++;
				break;
			case TOURNER:
				angle = Tableau_Actions[etape_actuelle].param;
				distance = M_PI*ecart_roues_largeur_mm*angle/360;
				BR.parcourirDistance(
						-distance,
						distance);
				etape_suivante++;
				break;
			case TIRER:
				//rouleau.autonom(Tableau_Actions[etape_actuelle].param);
				etape_suivante++;
				break;
			case ATTENDRE:
				Wait(Tableau_Actions[etape_actuelle].param);
				etape_suivante++;
				break;

			case FIN:
				return;
			default:
				etape_suivante++;
				return;
			}
		}

	void AutonomousInit() override {
		BR.SetVitesseMax(0.1); // m/s
				std::cout<<" DÃ©but autonome"<<std::endl;
				BR.reset();
				etape_suivante=0;
				etape_actuelle=0;
				etapeSuivante();

	}

	void asservissement() {
		double droite,gauche,angle,distance;

		droite = BR.mecaBackRight.GetDistance();
		gauche = BR.mecaBackLeft.GetDistance();
		BR.mecaBackRight.Reset();
		BR.mecaBackLeft.Reset();
		angle = gyro->GetAngle();
		gyro->Reset();
		distance = (droite+gauche)/2;
		position_x += distance*cos(angle);
		position_y += distance*sin(angle);
		std::cout<<"codeurD :"<<droite<<"; codeurG :"<<gauche<<"; angle :"<<angle<<"; distance :"<<distance<<endl;
		std::cout<<"positionx :"<<position_x<<"; positiony :"<<position_y<<endl;
	}


	void AutonomousPeriodic() {


		Scheduler::GetInstance()->Run();
				double erreurMaxi = 0;
				if(Tableau_Actions[etape_actuelle].type == AVANCER
					&& Tableau_Actions[etape_actuelle].param < 2000 )
				{
					erreurMaxi = 0.1*std::abs(Tableau_Actions[etape_actuelle].param); // 10 % quand infÃ©rieur Ã  2m
					// todo : timeout

				}
				else
				{
					erreurMaxi = 300; //mm
				}
				double delta=0;
				if( (delta= BR.effectuerConsigne()) < erreurMaxi)
					etapeSuivante();
	}

	void TeleopInit() {
		std::cout<<" DÃ©but tÃ©lÃ©opÃ©rÃ©"<<std::endl;
				BR.reset();
				BR.SetVitesseMax(30.0); // m/s
	}

	void TeleopPeriodic() {
		asservissement();

// si appui sur bouton depose_roue_auto:

		// si appui sur bouton depose_roue_auto:

		if(Joystick1->GetRawButton(BTN_DEPOSE_ROUE_AUTO)){
			// gestion du depot de roue en mode automatique
			//BR.deposeRoueAuto(Joystick1,gyro,ultraSon_G,ultraSon_D);
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

			if (Joystick1->GetRawButton(3))
			{
				pince.serrerPince();
				std::cout<<"je serre"<<std::endl;
			}


			if (Joystick1->GetRawButton(4))
			{
				pince.desserrerPince();
				std::cout<<"je deserre"<<std::endl;

			}

			if (Joystick1->GetRawButton(5))
			{
				pince.leverPince();

				std::cout<<"je leve"<<std::endl;

			}

			if (Joystick1->GetRawButton(6))
			{
				pince.abaisserPince();
				std::cout<<"je baisse"<<std::endl;

			}

			if (Joystick1->GetRawButton(8))
			{
			bac.leverBac();
			std::cout<<"je baisse"<<std::endl;

			}

			if (Joystick1->GetRawButton(9))
			{
			bac.rentrerBac();
			std::cout<<"je baisse"<<std::endl;

			}
BR.mvtTreuil( Joystick1);




		// END OF TEST

	}
/*
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
				// Give the output stream a new image to display*/

				// FRED MESSAGE*/
				/*if(BR.getRobotMode() == MODE_TANK)
					putText(mat,"Mode TANK",cv::Point(140,140),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255, 255, 255));
				else
					putText(mat,"Mode MECANUM",cv::Point(140,140),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255, 255, 255));
				*/
		/*
				outputStream.PutFrame(mat);

				//appel fonction reconnaissance visuelle
				p->Process(mat);
							//std::cout << "findBlobsInput = " << std::endl << " " << mat << std::endl << std::endl;
								/*double[] defaultValue= new double[0];
								table.GetNumberArray("CenterX",defaultValue);*/

				//tentative lecture des données renvoyées par fonctions de reconnaissance visuelle
		/*		std::vector<double> arr= table->GetNumberArray("Width", llvm::ArrayRef<double>());
				std::cout<<"avant boucle"<<endl<<arr.size()<<endl;
				for(unsigned int i=0;i<arr.size();i++){
						std::cout<<arr[i]<<""<<endl;
						std::cout<<"dans boucle"<<endl;
				}
			}
		}

*/

	}

};

START_ROBOT_CLASS(Robot)
