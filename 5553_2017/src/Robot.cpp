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

enum type_etape {AUCUN, FIN, AVANCER, TOURNER, ATTENDRE,PINCE_H,BAC,PINCE_V};

struct etape{
	float param;
	float param2;
	enum type_etape type;
};

struct etape Tableau_Actions[] {
		/*{180*17,0, AVANCER},
		{0,0,PINCE_H},
		{0*17,0, AVANCER},
		{0,0,PINCE_V},*/
		{180*17,90, AVANCER},
		{90*17,45, AVANCER},
		{0,0,PINCE_H},
		{150*17,45, AVANCER},
		{0,0,PINCE_V},
		/*{2000,AVANCER},
		{-0.8f, TIRER},
		{2, ATTENDRE},
		{0, TIRER},*/
		{0,0,FIN}
};



class Robot: public frc::IterativeRobot {
public:

	// dï¿½claration des capteurs et actionneurs
	Joystick* Joystick1;
	ADXRS450_Gyro* gyro;
	BaseRoulante BR;
	Servo* Plaque_Zeppelin;
	DoubleSolenoid* Pince_Vertical;
	DoubleSolenoid* Pince_Horizontal;
	DoubleSolenoid* Bac;
	VictorSP* Treuil;
	Ultrasonic *Ultrason_Avant; // creates the ultra object
	Preferences *Val;
	//P
	int Mode_Servo=0;
	double throttle=0;
	int robotMode ;
	int etape_actuelle;
	int etape_suivante;
	double ecart_roues_largeur_mm = 1100;  //740
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
	void RobotInit() {

		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro

		Plaque_Zeppelin =new Servo(5);
		Plaque_Zeppelin->SetAngle(0);
		Pince_Vertical= new DoubleSolenoid(4,5);
		Pince_Horizontal= new DoubleSolenoid(6,7);
		Bac= new DoubleSolenoid(2,3);
		Treuil=new VictorSP(4);
		Treuil->Set(0);
		robotMode = MODE_TANK; // on dï¿½marre en mode TANK par dï¿½faut
		Joystick1 = new Joystick(0);								// ï¿½ connecter sur port USB0
		std::thread visionThread(VisionThread);
		visionThread.detach();



	}

	void etapeSuivante()
		{
		BR.counteur_Fin=0;
			etape_actuelle=etape_suivante;
			double angle, distance;
			switch(Tableau_Actions[etape_actuelle].type)
			{
			case AVANCER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				std::cout<<"Etape Avancer"<<std::endl;
				etape_suivante++;
				break;
			case TOURNER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				break;
			case PINCE_H:
				Pince_Horizontal->Set(frc::DoubleSolenoid::kForward);
				frc::Wait(0.5);
				std::cout<<"PAss�"<<std::endl;
				etape_suivante++;
				etapeSuivante();
				break;
			case PINCE_V:
				Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);
				etape_suivante++;
				etapeSuivante();
				break;
			case FIN:
				return;
			default:
				etape_suivante++;
				return;
			}
			BR.reset();
		}

	void AutonomousInit() override {
		BR.SetVitesseMax(0.1); // m/s
				std::cout<<" D�but autonome"<<std::endl;
				BR.reset();
				BR.setRobotMode(MODE_TANK);
				BR.setConsigne(0,0);
				etape_suivante=0;
				etape_actuelle=0;
				etapeSuivante();
						Pince_Horizontal->Set(frc::DoubleSolenoid::kReverse);
						frc::Wait(1);
						Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
						Bac->Set(frc::DoubleSolenoid::kReverse);

	}

	void AutonomousPeriodic() {

		if(Tableau_Actions[etape_actuelle].type==AVANCER||Tableau_Actions[etape_actuelle].type==TOURNER)
				{
			std::cout<<" Avancement"<<std::endl;

					if(BR.effectuerConsigne(gyro->GetAngle())==1)
						etapeSuivante();
				}
		Plaque_Zeppelin->SetAngle(48);

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

					if (Joystick1->GetRawButton(7))
							Bac->Set(frc::DoubleSolenoid::kForward);

					if (Joystick1->GetRawButton(8))
							Bac->Set(frc::DoubleSolenoid::kReverse);

					if((throttle=(Joystick1->GetThrottle()-1))<=0)
						Treuil->Set(throttle);
					if (Joystick1->GetRawButton(11))
						Mode_Servo=0;
					if (Joystick1->GetRawButton(12))
						Mode_Servo=1;

					if(Mode_Servo==0)
					{
						Plaque_Zeppelin->SetAngle(48);

					}
					else
					{
						Plaque_Zeppelin->SetAngle(145);
					}

	}



};
START_ROBOT_CLASS(Robot)
