#include <iostream>
#include <memory>
#include <string>
#include <LiveWindow/LiveWindow.h>
#include <chrono>
#include <unistd.h>

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
int reconnaissance_visuelle=1;
#include "WPILib.h"
extern float P_COEFF_A;//0.017
extern float P_COEFF_L;
extern int TOLERANCE;
extern float somme_erreur;
extern float delta_erreur;
enum type_etape {AUCUN, FIN, AVANCER, RECULER, TOURNER, ATTENDRE,PINCE_H,BAC,PINCE_V};

struct etape{
	float param;
	float param2;
	enum type_etape type;
};

struct etape Tableau_Actions[] {
		{20,0, AVANCER},
		//{0,0, PINCE_V},
		{-190,0, RECULER},
		{0,0,FIN}
};
/*struct etape Tableau_Actions[] {
		{110,0, AVANCER},
		{0,19, TOURNER},
		{40,0, AVANCER},
		{0,0,FIN}
};*/




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
			cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
			camera.SetResolution(640, 480);
			camera.SetFPS(20);
			camera.SetExposureManual(3);
					cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
					cs::CvSource outputStream = CameraServer::GetInstance()->
					PutVideo("Test", 640, 480);
					cv::Mat mat;
					cv::Mat mat2;
					while (reconnaissance_visuelle==1) {
						cv::Mat Erode_Kernel;
						if (cvSink.GrabFrame(mat) == 0) continue;
						cv::cvtColor(mat,mat2,cv::COLOR_BGR2RGB);
						cv::inRange(mat2,cv::Scalar(0.0,44.0,0.0),cv::Scalar(100.0,240.0,190.0),mat);

						cv::erode(mat,mat2,Erode_Kernel,cv::Point(-1, -1),3.0,cv::BORDER_CONSTANT,cv::Scalar(-1));
						cv::dilate(mat2,mat,Erode_Kernel,cv::Point(-1,-1),3.0, cv::BORDER_CONSTANT,cv::Scalar(-1));
						outputStream.PutFrame(mat);
						vector<vector<Point> > contours;
						vector<Vec4i> hierarchy;

						findContours(mat,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0, 0));
						Mat drawing = Mat::zeros( mat.size(), CV_8UC3 );
						vector<Point2f> mc( contours.size() );
						vector<Moments> mu(contours.size() );
						float centre1=-1,centre2=-1;
						float hauteur=-1, largeur=-1, rapport=-1;
						float perimetre_min=30;
						float contours_acceptes=0;
						largeur_bande=-1;
						longueur_bande=-1;
						float rapport_min=1.9;
						float rapport_max=4.0;
						for(unsigned int i = 0; i< contours.size(); i++)
						{
												Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
												drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
												mu[i] = moments( contours[i], false );
												mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
												cv::Rect rect=boundingRect(contours[i]);
												hauteur=float(rect.height);
												largeur=float(rect.width);
												rapport=hauteur/largeur;
												if(i==0 && arcLength(contours[0],true)>perimetre_min && rapport<rapport_max && rapport>rapport_min){
													centre1=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;
													contours_acceptes++;
												}
												if(i==1 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<rapport_max && rapport>rapport_min){
													centre2=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													longueur_bande=rect.height;
													largeur_bande=rect.width;
													contours_acceptes++;
												}
												else if(i==1 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<rapport_max && rapport>rapport_min){
													centre1=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;
													contours_acceptes++;
												}
												if(i==2 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<rapport_max && rapport>rapport_min){
													centre2=mu[i].m10/mu[i].m00;
													contours_acceptes++;
													centre2=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;

												}
												else if(i==2 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<rapport_max && rapport>rapport_min){
													centre1=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;
													contours_acceptes++;
												}
												if(i==3 && arcLength(contours[0],true)>perimetre_min && centre1!=-1 && rapport<rapport_max && rapport>rapport_min){
													centre2=mu[i].m10/mu[i].m00;
													contours_acceptes++;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;
													contours_acceptes++;
												}
												else if(i==3 && arcLength(contours[0],true)>perimetre_min && centre1==-1 && rapport<rapport_max && rapport>rapport_min){
													centre1=mu[i].m10/mu[i].m00;
													Perimetre_bandes=arcLength(contours[i],true);
													largeur_bande=rect.width;
													longueur_bande=rect.height;
													contours_acceptes++;
												}
						}
						if(centre1!=-1 && centre2!=-1){
									Centre_bandes=(centre1+centre2)/2;
						}
						else if(centre1!=-1){
									//cout<<"une seule bande visible"<<endl;
									Centre_bandes=-1;
						}
						else{
									Centre_bandes=-1;
									Perimetre_bandes=-1;
						}
						if(longueur_bande<320 && longueur_bande!=-1){
							distance_vision=7502.7*pow(longueur_bande,-0.973)-23;
						}
						else distance_vision=-1;
			}
					camera.SetExposureAuto();
					/*cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(1);
					camera2.SetResolution(160, 120);
					camera2.SetFPS(5);
					camera2.SetExposureAuto();*/
	}

	void RobotInit() {
		// initialisation des objets et donnï¿½es
		gyro = new ADXRS450_Gyro(); 								// ï¿½ connecter sur SPI
		gyro->Calibrate(); // initialisation de la position 0 du gyro

		Pince_Vertical= new DoubleSolenoid(2,3);
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
				P_COEFF_A=0.07;
				P_COEFF_L=0.00132;

				TOLERANCE=400;
				break;
			case RECULER:
				cout<<"\nReculer"<<std::endl;
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				P_COEFF_A=0.07;
				P_COEFF_L=0.0020;
				TOLERANCE=200;
				break;
			case TOURNER:
				BR.setConsigne(Tableau_Actions[etape_actuelle].param,Tableau_Actions[etape_actuelle].param2);
				etape_suivante++;
				P_COEFF_A=0.027;
				TOLERANCE=200;
				break;
			case PINCE_H:

				frc::Wait(0.2);
				etape_suivante++;
				etapeSuivante();
				break;
			case PINCE_V:
				frc::Wait(0.2);
				Pince_Roue->Set(0.7);
				Pince_Vertical->Set(frc::DoubleSolenoid::kForward);
				frc::Wait(0.5);
				Pince_Roue->Set(-0.5);
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

		//angleini = gyro->GetAngle();
		gyro->Reset();
		angle=gyro->GetAngle();
		somme_erreur=0;
		delta_erreur=0;
		reconnaissance_visuelle=1;



	}

	void AutonomousPeriodic() {


		reconnaissance_visuelle=1;


				if(Tableau_Actions[etape_actuelle].type==AVANCER||Tableau_Actions[etape_actuelle].type==TOURNER||Tableau_Actions[etape_actuelle].type==RECULER)
				{
					if(BR.effectuerConsigne(gyro->GetAngle())==1)
						etapeSuivante();
				}
				else BR.meca_avancer(0);



	}

	void TeleopInit() {
		std::cout<<" DÃ©but tÃ©lÃ©opÃ©rÃ©"<<std::endl;
		BR.reset();
		BR.SetVitesseMax(30.0); // m/s
		reconnaissance_visuelle=0;
	}

	void TeleopPeriodic() {

		reconnaissance_visuelle=0;
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
