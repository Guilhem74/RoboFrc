#include <iostream>
#include <memory>
#include <string>
#include <LiveWindow/LiveWindow.h>

#include <ADXRS450_Gyro.h>
#include <BaseRoulante.h>
#include <constantes.h>
#include <CameraServer.h>
#include <math.h>

#include <thread>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
using namespace cv;
using namespace std;
RNG rng(12345);
int x=0;
float Centre_bandes=-1;
float Perimetre_bandes=-1;
float largeur_bande=-1;
float longueur_bande=-1;
float angle=0;
extern float distance_vision;

#include "WPILib.h"
extern float P_COEFF_A;//0.017
extern int TOLERANCE;

enum type_etape {AUCUN, FIN, AVANCER, TOURNER, ATTENDRE,PINCE_H,BAC,PINCE_V};

struct etape{
	float param;
	float param2;
	enum type_etape type;
};
#define MILLIEU true
#define GAUCHE true
#define BLEU true
#define ROUGE false

struct etape Tableau_Actions[] {
		{100,0, AVANCER},
		{0,0,FIN}
};
/*#elif MILLIEU==false && GAUCHE == true && BLEU ==true && ROUGE ==false//Gauche Bleu
struct etape Tableau_Actions[] {
		{187.56*34,0, AVANCER},
		{0,60, TOURNER},
		{152*34,60, AVANCER},
		{0,0,PINCE_H},
		{-50*34,60, AVANCER},
		{0,0,PINCE_V},
		{0,0,FIN}
};
#elif MILLIEU==false && GAUCHE == false && BLEU ==true && ROUGE ==false//Droite Bleu
struct etape Tableau_Actions[] {
		{188*34,0, AVANCER},
		{0,-60, TOURNER},
		{159*34,-60, AVANCER},
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


class Robot: public frc::IterativeRobot {
public:


	// dï¿½claration des capteurs et actionneurs
	Joystick* Joystick1;
	ADXRS450_Gyro* gyro;
	BaseRoulante BR;
	Servo* Plaque_Zeppelin; // servo min/max : 48/150
	DoubleSolenoid* Pince_Vertical;

	VictorSP* Treuil;
	VictorSP* Pince_Roue;
	Ultrasonic *Ultrason_Avant; // creates the ultra object
	Preferences *prefs;
	//P
	int Mode_Servo=0;
	double throttle=0;
	int robotMode;
	double angleini = 0;
	double erreuranglemax = 5;
	int etape_actuelle;
	int etape_suivante;
	double ecart_roues_largeur_mm = 1100;  //740
	static void VisionThread() {
			// Get the USB camera from CameraServer

			cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
			// Set the resolution
			camera.SetResolution(640, 480);
			camera.SetFPS(20);
			camera.SetExposureManual(3);
			/*cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
						camera2.SetResolution(160, 120);
						camera2.SetFPS(5);*/
			// Get a CvSink. This will capture Mats from the Camera
			cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
			// Setup a CvSource. This will send images back to the Dashboard

			cs::CvSource outputStream = CameraServer::GetInstance()->
							PutVideo("Test", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			cv::Mat mat;
			cv::Mat mat2;
			while (true) {
				cv::Mat Erode_Kernel;
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.GrabFrame(mat) == 0) {
					// Send the output the error.
					//outputStream.NotifyError(cvSink.GetError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				//rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
				//cv::Scalar(255, 255, 255), 5);
				cv::cvtColor(mat,mat2,cv::COLOR_BGR2RGB);
				cv::inRange(mat2,cv::Scalar(0.0,18.0,0.0),cv::Scalar(33.0,205.0,128.0),mat);
				outputStream.PutFrame(mat);
				cv::erode(mat,mat2,Erode_Kernel,cv::Point(-1, -1),4.0,cv::BORDER_CONSTANT,cv::Scalar(-1));
				cv::dilate(mat2,mat,Erode_Kernel,cv::Point(-1,-1),2.0, cv::BORDER_CONSTANT,cv::Scalar(-1));
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;

				findContours(mat,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0, 0));
				Mat drawing = Mat::zeros( mat.size(), CV_8UC3 );
				vector<Point2f> mc( contours.size() );
				vector<Moments> mu(contours.size() );
				float centre1=-1,centre2=-1;
				float hauteur=-1, largeur=-1, rapport=-1;
				float perimetre_min=30;
				largeur_bande=-1;
				longueur_bande=-1;

				for(int i = 0; i< contours.size(); i++)
				{
										Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
										drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
										mu[i] = moments( contours[i], false );
										mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
										cv::Rect rect=boundingRect(contours[i]);
										hauteur=float(rect.height);
										largeur=float(rect.width);
										rapport=hauteur/largeur;
										//cout<<"hauteur : \n"<<rect.height<<endl;
										//cout<<"width : \n"<<rect.width<<endl;
										//cout<<"perimetre: \n"<<arcLength(contours[i],true)<<endl;

										if(i==0 && arcLength(contours[0],true)>perimetre_min && rapport<3.5 && rapport>1.5){
											centre1=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}

										if(i==1 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<3.5 && rapport>1.5){
											centre2=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
										}
										else if(i==1 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<3.5 && rapport>1.5){
											centre1=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}

										if(i==2 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<3.5 && rapport>1.5){
											centre2=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}
										else if(i==2 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<3.5 && rapport>1.5){
											centre1=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}

										if(i==3 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<3.5 && rapport>1.5){
											centre2=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}
										else if(i==3 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<3.5 && rapport>1.5){
											centre1=mu[i].m10/mu[i].m00;
											Perimetre_bandes=arcLength(contours[i],true);
											largeur_bande=rect.width;
											longueur_bande=rect.height;
										}

										if(centre1!=-1 && centre2!=-1){
											Centre_bandes=(centre1+centre2)/2;
										}
										else{
											Centre_bandes=-1;
											Perimetre_bandes=-1;
										}
										//cout<<"Perimetre_bandes: "<<Perimetre_bandes<<endl;


				}
				//cout<<"\nhauteur: "<<longueur_bande<<endl;
				//cout<<"hauteur, largeur, rapport: \n"<<longueur_bande<<", "<<largeur_bande<<", "<<rapport<<endl;
				if(longueur_bande<320){
					//bande 13cm:
					distance_vision=7502.7*pow(longueur_bande,-0.973);
					//bande 12cm:
					//distance=5502.9*pow(longueur_bande,-0.915);
				}
				//cout<<"\ndistance= "<<distance_vision<<endl;




				//outputStream.PutFrame(drawing);
			}

	}




	void RobotInit() {

		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro

		Pince_Vertical= new DoubleSolenoid(1,0);
		Treuil=new VictorSP(4);
		Treuil->Set(0);
		Pince_Roue=new VictorSP(5);
		Pince_Roue->Set(0);
		robotMode = MODE_TANK; // on dï¿½marre en mode TANK par dï¿½faut
		Joystick1 = new Joystick(0);								// ï¿½ connecter sur port USB0
		std::thread visionThread(VisionThread);
		visionThread.detach();
		Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);
		prefs = Preferences::GetInstance();

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
				TOLERANCE=150;
				break;
			case TOURNER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				P_COEFF_A=0.017;
				TOLERANCE=100;
				break;
			case PINCE_H:

				frc::Wait(0.2);
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
		Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);

		angleini = gyro->GetAngle();

		angle=gyro->GetAngle();



	}

	void AutonomousPeriodic() {



				/*if(Tableau_Actions[etape_actuelle].type==AVANCER||Tableau_Actions[etape_actuelle].type==TOURNER)*/

		/*if(Tableau_Actions[etape_actuelle].type==AVANCER||Tableau_Actions[etape_actuelle].type==TOURNER)

				{

					if(BR.effectuerConsigne(gyro->GetAngle())==1)
						etapeSuivante();
				}*/
		std::cout<<"gyro: "<<gyro->GetAngle()<<std:endl;
				//BR.setRobotMode(MODE_MECA);
				/*angle = gyro->GetAngle();
				if(Centre_bandes<270 && Centre_bandes!=-1){
					//BR.meca_gauche(0.5);
					cout<<"gauche"<<endl;
				}
				else if(Centre_bandes>370) {
					//BR.meca_droite(0.5);
					cout<<"droite"<<endl;
				}
				else if(Centre_bandes==-1){
					cout<<"Erreur d�tection"<<endl;
					BR.meca_droite(0);
					BR.meca_gauche(0);
				}
				else if(largeur_bande<20){
					//BR.meca_avancer(0.5);
					cout<<"avancer"<<endl;
				}
				else if(gyro->GetAngle()>angle+5){
					//BR.meca_tourne_droite(0.6);
					cout<<"d�calage gauche"<<endl;
				}
				else if(gyro->GetAngle()<angle-5){
					//BR.meca_tourne_gauche(0.6);
					cout<<"d�calage droite"<<endl;
				}
				else{
					BR.meca_droite(0);
					BR.meca_gauche(0);
					cout<<"Elles sont au milieu et bonne distance"<<endl;
				}

				/*if(angle-angleini > erreuranglemax){
						BR.meca_tourne_droite(0.7);
						cout<<"tourne_droite"<<endl;
				}
				else if(angle-angleini < -erreuranglemax){
						BR.meca_tourne_gauche(0.7);
						cout<<"tourne_gauche"<<endl;

				if(gyro->GetAngle()>angle+5){
					//BR.meca_tourne_droite(0.6);
				}
				if(gyro->GetAngle()<angle-5){
					//BR.meca_tourne_gauche(0.6);

				}
				/*if(Perimetre_bandes<500 && Perimetre_bandes!=-1 && Centre_bandes<370 && Centre_bandes>270){
					BR.meca_avancer(0.7);
					cout<<"avancer"<<endl;

				}*/




	}

	void TeleopInit() {
		std::cout<<" DÃ©but tÃ©lÃ©opÃ©rÃ©"<<std::endl;
				BR.reset();
				BR.SetVitesseMax(30.0); // m/s
		angleini = gyro->GetAngle();


	}

	void TeleopPeriodic() {



		if(Joystick1->GetRawButton(BTN_TANK))
					{
						BR.setRobotMode(MODE_TANK);
					}

					if(Joystick1->GetRawButton(BTN_MECA))
					{
						BR.setRobotMode(MODE_MECA);
						BR.anglevoulu = gyro->GetAngle();
					}

					if (Joystick1->GetX() || Joystick1->GetY() || Joystick1->GetZ() )
					{
						BR.mvtJoystick(Joystick1,gyro, angleini);
					}

					if (Joystick1->GetRawButton(3))
					{
						Pince_Roue->Set(0.7);
						Wait(0.3);
						Pince_Roue->Set(0.0);
					}

					if (Joystick1->GetRawButton(4)){
						Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
						prefs->PutBoolean("LED_PINCEV_MONTE",true);
					}

					if (Joystick1->GetRawButton(5)){
						Pince_Vertical->Set(frc::DoubleSolenoid::kReverse);
						prefs->PutBoolean("LED_PINCEV_MONTE",false);
						Pince_Roue->Set(-0.7);
						Wait(0.3);
						Pince_Roue->Set(0.0);
					}


					if((throttle=(Joystick1->GetThrottle()-1))<=0)
						Treuil->Set(throttle);


	}



};
START_ROBOT_CLASS(Robot)
