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
extern float P_COEFF_A;//0.017
extern int TOLERANCE;

enum type_etape {AUCUN, FIN, AVANCER, TOURNER, ATTENDRE,PINCE_H,BAC,PINCE_V};

struct etape{
	float param;
	float param2;
	enum type_etape type;
};

#define MILLIEU false
#define GAUCHE false
#define BLEU true
#define ROUGE false
/*
#if MILLIEU==true && (ROUGE==true || BLEU==true)//Millieu
struct etape Tableau_Actions[] {
		{215*17,0, AVANCER},
		{0,0,PINCE_H},
		{-50*34,0, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};
#elif MILLIEU==false && GAUCHE == true && BLEU ==true && ROUGE ==false//Gauche Bleu
struct etape Tableau_Actions[] {
		{150*34,0, AVANCER},
		{0,60, TOURNER},
		{200*34,60, AVANCER},
		{0,0,PINCE_H},
		{-50*34,60, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};
#elif MILLIEU==false && GAUCHE == false && BLEU ==true && ROUGE ==false//Droite Bleu
struct etape Tableau_Actions[] {
		{100*34,0, AVANCER},
		{0,-60, TOURNER},
		{200*34,-60, AVANCER},
		{0,0,PINCE_H},
		{-50*34,-60, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};

#elif MILLIEU==false && GAUCHE == false && BLEU ==false && ROUGE ==true//Droite Rouge boiler
struct etape Tableau_Actions[] {
		{187.56*34,0, AVANCER},
		{0,-62, TOURNER},
		{152*34,-62, AVANCER},
		{0,0,PINCE_H},
		{-50*34,-62, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};
#elif MILLIEU==false && GAUCHE == true && BLEU ==false && ROUGE ==true//gauche  Rouge
struct etape Tableau_Actions[] {
		{188*34,0, AVANCER},
		{0,62, TOURNER},
		{145*34,62, AVANCER},
		{0,0,PINCE_H},
		{-50*34,62, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};
#endif*/
/*struct etape Tableau_Actions[] {
		{150*34,0, AVANCER},
		{0,-60, TOURNER},
		{170*34,-60, AVANCER},
		{0,0,PINCE_H},
		{-50*34,-60, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};*/
/*
struct etape Tableau_Actions[] {
		{80*34,0, AVANCER},
		{0,-60, TOURNER},
		{110*34,-60, AVANCER},
		{0,0,PINCE_H},
		{-50*34,-60, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};*/
/* MILLIEU*/
/*
  struct etape Tableau_Actions[] {
		//{100*17,0, AVANCER},
		{195*17.5,0, AVANCER},
		{0,0,PINCE_H},
		{-100*17.5,0, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};*/
//Coté
struct etape Tableau_Actions[] {
		{180*39,0, AVANCER},
				/*{0,0,PINCE_H},
				{-100*17.5,0, AVANCER},
				{0,0,PINCE_V},*/
				{0,0,FIN}
};
class Robot: public frc::IterativeRobot {
public:


	// dï¿½claration des capteurs et actionneurs
	Joystick* Joystick1;
	ADXRS450_Gyro* gyro;
	BaseRoulante BR;
	Servo* Plaque_Zeppelin; // servo min/max : 48/150
	DoubleSolenoid* Pince_Vertical;
	DoubleSolenoid* Pince_Horizontal;
	DoubleSolenoid* Bac;
	VictorSP* Treuil;
	Ultrasonic *Ultrason_Avant; // creates the ultra object
	Preferences *prefs;
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
						/*cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
									camera2.SetResolution(160, 120);
									camera2.SetFPS(5);*/
						// Get a CvSink. This will capture Mats from the Camera
						cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
			}






	void RobotInit() {

		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro

		Plaque_Zeppelin =new Servo(5);
		Plaque_Zeppelin->SetAngle(60);
		Pince_Vertical= new DoubleSolenoid(4,5);
		Pince_Horizontal= new DoubleSolenoid(6,7);
		Bac= new DoubleSolenoid(2,3);
		Treuil=new VictorSP(4);
		Treuil->Set(0);
		robotMode = MODE_TANK; // on dï¿½marre en mode TANK par dï¿½faut
		Joystick1 = new Joystick(0);								// ï¿½ connecter sur port USB0
		std::thread visionThread(VisionThread);
		visionThread.detach();
		Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
		Pince_Horizontal->Set(frc::DoubleSolenoid::kForward);
		prefs = Preferences::GetInstance();
		BR.setRobotMode(MODE_TANK);

	}

	void etapeSuivante()
		{
		BR.counteur_Fin=0;
			etape_actuelle=etape_suivante;

			switch(Tableau_Actions[etape_actuelle].type)
			{
			case AVANCER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				P_COEFF_A=0.04;
				TOLERANCE=200;
				break;
			case TOURNER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				P_COEFF_A=0.017;
				TOLERANCE=150;
				break;
			case PINCE_H:
				Pince_Horizontal->Set(frc::DoubleSolenoid::kReverse);
				frc::Wait(0.5);
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
		Pince_Horizontal->Set(frc::DoubleSolenoid::kForward);
		Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
		Bac->Set(frc::DoubleSolenoid::kReverse);

	}

	void AutonomousPeriodic() {


		if(Tableau_Actions[etape_actuelle].type==AVANCER||Tableau_Actions[etape_actuelle].type==TOURNER)
				{

					if(BR.effectuerConsigne(gyro->GetAngle())==1)
						etapeSuivante();
				}
		//Plaque_Zeppelin->SetAngle(48);

	}

	void TeleopInit() {
		std::cout<<" DÃ©but tÃ©lÃ©opÃ©rÃ©"<<std::endl;
				BR.reset();
				BR.SetVitesseMax(30.0); // m/s
				prefs->PutBoolean("LED_PINCEH_OUVERTE",false);
				prefs->PutBoolean("LED_PINCEV_MONTE",true);


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
					{
						Pince_Horizontal->Set(frc::DoubleSolenoid::kReverse);

						prefs->PutBoolean("LED_PINCEH_OUVERTE",true);
					}

					if (Joystick1->GetRawButton(4))
					{
						Pince_Horizontal->Set(frc::DoubleSolenoid::kForward);

						prefs->PutBoolean("LED_PINCEH_OUVERTE",false);
					}

					if (Joystick1->GetRawButton(5)){
						Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
						prefs->PutBoolean("LED_PINCEV_MONTE",true);
					}

					if (Joystick1->GetRawButton(6)){
						Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);
						prefs->PutBoolean("LED_PINCEV_MONTE",false);
					}

					if (Joystick1->GetRawButton(7))
							Bac->Set(frc::DoubleSolenoid::kForward);

					if (Joystick1->GetRawButton(8))
							Bac->Set(frc::DoubleSolenoid::kReverse);

					if((throttle=(Joystick1->GetThrottle()-1))<=0)
						//Treuil->Set(throttle);
					if (Joystick1->GetRawButton(11))
						Mode_Servo=0;
					if (Joystick1->GetRawButton(12))
						Mode_Servo=1;

					if(Mode_Servo==0)
					{
						// servo min/max : 48/150
						Plaque_Zeppelin->SetAngle(48);

					}
					else
					{
						Plaque_Zeppelin->SetAngle(150);
					}

			BR.setRobotMode(MODE_TANK);
			Wait(1);
			BR.setRobotMode(MODE_MECA);
			Wait(1);
			BR.setRobotMode(MODE_TANK);
			Wait(1);
			BR.setRobotMode(MODE_MECA);
			Wait(1);
	}



};
START_ROBOT_CLASS(Robot)
