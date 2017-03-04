#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include <Ultrasonic.h>
#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>
#include <Bac.h>
#include <Pince.h>

#include <thread>
#include <test_contour.h>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>


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
	Bac* bac;
	Pince pince;
	int robotMode ;

	void RobotInit() {

		// initialisation des objets et données
		gyro = new ADXRS450_Gyro(); // à connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro
		robotMode = MODE_TANK; // on démarre en mode TANK par défaut
		Joystick1 = new Joystick(0);								// à connecter sur port USB0
		ultraSon_G = new Ultrasonic(0,1,Ultrasonic::kMilliMeters); 	// à connecter sur DIO-0 et DIO-1
		ultraSon_D = new Ultrasonic(2,3,Ultrasonic::kMilliMeters); 	// à connecter sur DIO-2 et DIO-3

		// initialisation de la networkTable
		//table = NetworkTable::GetTable("GRIP/myContoursReport");

		//lancement de la video
		std::thread visionThread(VisionThread(centerX));
		visionThread.detach();


		bac = new Bac();

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		//BR.MonterCorde(Joystick1);
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

			//si boutton lever bac
			if(Joystick1->GetRawButton(BTN_BAC_UP))
				bac->leverBac();

			//si bouton abaisser bac
			if(Joystick1->GetRawButton(BTN_BAC_DOWN))
				bac->rentrerBac();

			if(Joystick1->GetRawButton(BTN_SER_PINCE))
			    pince.serrerPince();

			if(Joystick1->GetRawButton(BTN_DESSER_PINCE))
				pince.desserrerPince();

			/*if(Joystick1->GetRawButton(BTN_PINCE_UP))
				pince.leverPince(bac);*/

			if(Joystick1->GetRawButton(BTN_PINCE_DOWN))
				pince.abaisserPince();

			/*if(Robot::centerX < 0)
				printf("hello X");*/


		}

		// FOR TEST //
		double angle=gyro->GetAngle();
		SmartDashboard::PutString("DB/String 0",std::to_string(angle));
		// END OF TEST

	}

	void TestPeriodic() {

		lw->Run();

	}


	frc::LiveWindow* lw = LiveWindow::GetInstance();
	static int* centerX;
	static void VisionThread(int *classCenter) {

			*classCenter=1;

	} // end of static thread
};

START_ROBOT_CLASS(Robot)
