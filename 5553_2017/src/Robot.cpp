#include <iostream>
#include <memory>
#include <string>
#include <LiveWindow/LiveWindow.h>

#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>
#include <CameraServer.h>


#include <thread>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "WPILib.h"

enum type_etape {AUCUN, FIN, AVANCER, TOURNER, TIRER, ATTENDRE};

struct etape{
	float param;
	enum type_etape type;
};

struct etape Tableau_Actions[] {
		{150*4500/110, AVANCER},
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
	BaseRoulante BR;
	Servo* plaque_Zepplin;
	DoubleSolenoid* Pince_Vertical;
	DoubleSolenoid* Pince_Horizontal;
	DoubleSolenoid* Bac;
	VictorSP* Treuil;

	//P
	double throttle=0;
	int robotMode ;
	int etape_actuelle;
	int etape_suivante;
	double ecart_roues_largeur_mm = 1100;  //740
	double P_Value;
	double I_Value;
	double D_Value;


	void RobotInit() {

		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro

		Pince_Vertical= new DoubleSolenoid(2,3);
		Pince_Horizontal= new DoubleSolenoid(6,7);
		Bac= new DoubleSolenoid(4,5);
		plaque_Zepplin=new Servo(5);
		plaque_Zepplin->SetAngle(5.0);
		Treuil=new VictorSP(4);
		Treuil->Set(0);
		robotMode = MODE_TANK; // on dï¿½marre en mode TANK par dï¿½faut
		Joystick1 = new Joystick(0);								// ï¿½ connecter sur port USB0
		std::thread visionThread(VisionThread);
		visionThread.detach();
		P_Value = SmartDashboard::GetNumber	("P_Value", 0.00010);
		I_Value = SmartDashboard::GetNumber ("I_Value", 0.00010);
		D_Value = SmartDashboard::GetNumber	("D_Value", 0.00010);



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
				if( (delta= BR.effectuerConsigne(P_Value, I_Value, D_Value)) < erreurMaxi)
					std::cout<<"fini"<<std::endl;
	}

	void TeleopInit() {
		std::cout<<" DÃ©but tÃ©lÃ©opÃ©rÃ©"<<std::endl;
				BR.reset();
				BR.SetVitesseMax(30.0); // m/s
	}

	void TeleopPeriodic() {

					if(Joystick1->GetRawButton(BTN_TANK))
					{
						BR.setRobotMode(MODE_TANK);
					}

					if(Joystick1->GetRawButton(BTN_MECA))
					{
						BR.setRobotMode(MODE_MECA);
					}

					if (Joystick1->GetX() || Joystick1->GetY() || Joystick1->GetZ() )
					{
						BR.mvtJoystick(Joystick1,gyro);
					}
					if (Joystick1->GetRawButton(3))
						Pince_Horizontal->Set(frc::DoubleSolenoid::kForward);

					if (Joystick1->GetRawButton(4))
						Pince_Horizontal->Set(frc::DoubleSolenoid::kReverse);

					if (Joystick1->GetRawButton(5))
						Pince_Vertical->Set(frc::DoubleSolenoid::kForward);

					if (Joystick1->GetRawButton(6))
						Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);

					if (Joystick1->GetRawButton(8))
							Bac->Set(frc::DoubleSolenoid::kForward);

					if (Joystick1->GetRawButton(9))
							Bac->Set(frc::DoubleSolenoid::kReverse);

					if((throttle=(Joystick1->GetThrottle()-1))<=0)
						Treuil->Set(throttle);

					if(Joystick1->GetRawButton(12))
						plaque_Zepplin->SetAngle(5.0);

					if(Joystick1->GetRawButton(11))
						plaque_Zepplin->SetAngle(90.0);


	}
	static void VisionThread() {
			// Get the USB camera from CameraServer
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
					// Set the resolution
					camera.SetResolution(640, 480);
					camera.SetFPS(20);
					cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
								camera2.SetResolution(160, 120);
								camera2.SetFPS(5);
					// Get a CvSink. This will capture Mats from the Camera
					cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		}


};
START_ROBOT_CLASS(Robot)
