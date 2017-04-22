/*
 * BaseRoulante.cppS
 *
 *  Created on: 27 d�c. 2016
 *      Author: REBAUDET Thomas
 */

#include "WPILib.h"
#include <RobotDrive.h>
#include <BaseRoulante.h>
#include <DoubleSolenoid.h>
#include <constantes.h>
float P_COEFF_A=0.009;//0.017
float I_COEFF=0.0000000050*0;
float D_COEFF=0.00007*0;
int TOLERANCE=10000;


BaseRoulante::BaseRoulante():
mecaFrontLeft(0,0,1,1),mecaBackLeft(1,3,2,1),mecaFrontRight(3,6,7,0),mecaBackRight(2,4,5,0), verins_BASE(0,1)
{
		// arr�t des moteurs
		mecaFrontLeft.Set(0.0);
		mecaFrontRight.Set(0.0);
		mecaBackRight.Set(0.0);
		mecaBackLeft.Set(0.0);
		// configuration mode TANK

		verins_BASE.Set(frc::DoubleSolenoid::kReverse);
		Ultrason=new AnalogInput(3);
		robotMode = MODE_TANK;
		mode_auto = MODE_ALIGN;

		// configuration du robotDrive
		//R2D2 = new RobotDrive(mecaFrontLeft.getVictorSP(),mecaBackLeft.getVictorSP(),mecaFrontRight.getVictorSP(),mecaBackRight.getVictorSP());
		approach_speed = 0.3;
		align_dist = 500; // en mm
		align_marge = 20; // en mm
		rot_marge = 10; // en mm
		rot_speed = 0.3; // entre -1 et 1
}

void BaseRoulante::SetPID(double P_val,double I_val, double D_val)
{
	P=P_val;
	I=I_val;
	D=D_val;
}

void BaseRoulante::SetVitesseMax(double max)
{
	mecaFrontLeft.SetVitesseMax(max);
	mecaFrontRight.SetVitesseMax(max);
	mecaBackLeft.SetVitesseMax(max);
	mecaBackRight.SetVitesseMax(max);
}

void BaseRoulante::reset()
{
	mecaFrontLeft.Reset();
	mecaFrontRight.Reset();
	mecaBackLeft.Reset();
	mecaFrontRight.Reset();
}

void BaseRoulante::setConsigne(double Longueur, double Angle)
{//Met a jour les valeurs de consigne + raz les valeurs d'int�gration de l'assert
	Consigne_Dist=Longueur;
	Consigne_Ang=Angle;
	sommeErreursG=0;
	sommeErreursD=0;
	Erreur_Precedente_G=0;
	Erreur_Precedente_D=0;
	reset();
}

double BaseRoulante::PID_ANGLE(double Angle, double Angle_gyro)
{//Met a jour les valeurs de consigne + raz les valeurs d'int�gration de l'assert
	double erreur=Angle-Angle_gyro;
	return P_COEFF_A*erreur;

}
double BaseRoulante::PID_DISTANCE(double consigne_L, double valeur_Encodeur)
{//Met a jour les valeurs de consigne + raz les valeurs d'int�gration de l'assert
	double erreur=consigne_L-valeur_Encodeur;
	sommeErreur +=erreur;
	diff_erreur=erreur-erreur_prec;
	erreur_prec=erreur;
	return (P_COEFF_L*erreur+I_COEFF*sommeErreur+D_COEFF*diff_erreur);

}

int BaseRoulante::effectuerConsigne(double Angle_gyro)
{
	counteur_Fin++;
	double moyenneGauche = GetCmUltrason();
	double moyenneDroite = moyenneGauche;// mecaBackRight.GetDistance() ;
	std::cout<<"gauche :"<<mecaFrontLeft.GetDistance()<<std::endl;

	std::cout<<"angle :"<<Angle_gyro<<std::endl;

	powerRight=PID_DISTANCE(Consigne_Dist,moyenneDroite)-PID_ANGLE(Consigne_Ang,Angle_gyro);
	powerLeft=-(PID_DISTANCE(Consigne_Dist,moyenneGauche)+PID_ANGLE(Consigne_Ang,Angle_gyro));
	/*if(counteur_Fin>TOLERANCE)
		return 1;*/

	std::cout<<"Power: "<<powerLeft<<std::endl;
	mecaFrontLeft.Set(powerLeft);
	mecaBackLeft.Set(powerLeft);
	mecaFrontRight.Set(powerRight);
	mecaBackRight.Set(powerRight);
	return 0;
}

/*double BaseRoulante::Getdistance(int droite)
{
	if(droite==1)
	{
		//std::cout<<"mecaBackRight.GetDistance()"<<mecaBackRight.GetDistance()<<std::endl;
		return ((mecaBackRight.GetDistance())*(100*3.1415)/1300);
	}
	else
	{			//std::cout<<"mecaFrontLeft.GetDistance()"<<mecaFrontLeft.GetDistance()<<std::endl;

		return(mecaFrontLeft.GetDistance()*(100*3.1415)/230);
	}
}*/
double BaseRoulante::GetCmUltrason()
{
	double x=Ultrason->GetAverageValue();
	double y=0.1253*x-11.881;

	std::cout<<"y "<<y<<std::endl;

	return y;
}

void BaseRoulante::TestEncodeurs()
{
	std::cout<<"avant gauche :"<<mecaFrontLeft.GetDistance()<<std::endl;
	std::cout<<"avant droite :"<<mecaFrontRight.GetDistance()<<std::endl;
	std::cout<<"arriere gauche :"<<mecaBackLeft.GetDistance()<<std::endl;
	std::cout<<"arriere droite :"<<mecaBackRight.GetDistance()<<std::endl;
}


void BaseRoulante::setRobotMode(int mode){
	if(mode == MODE_TANK){
		// rentrer les verins

		verins_BASE.Set(frc::DoubleSolenoid::kReverse);
	}
	if(mode == MODE_MECA){
		// pousser les verins
		verins_BASE.Set(frc::DoubleSolenoid::kForward);

	}
	// store robot mode
	robotMode = mode;
}

int BaseRoulante::getRobotMode(){
	return(robotMode);
}


void BaseRoulante::mvtJoystick(Joystick *joystick, ADXRS450_Gyro* gyro)
{
	if(robotMode == MODE_TANK){
		//R2D2->ArcadeDrive(	-joystick->GetZ(),-joystick->GetY(),true);
		float x= -((float)joystick->GetX());
		float y= -((float)joystick->GetY());
		float z= -((float)joystick->GetZ());

		if (x>=-0.2 && x<=0.2)
			x=0;

		if (y>=-0.2 && y<=0.2)
					y=0;

		if (z>=-0.3 && z<=0.3)
					z=0;
		std::cout<<"gauche :"<<mecaBackLeft.GetDistance()<<std::endl;

		mecaFrontRight.Set(y +zCoeff *z);
		mecaBackRight.Set(y+zCoeff *z);
		mecaFrontLeft.Set(-y+ zCoeff *z);
		mecaBackLeft.Set(-y+ zCoeff *z);

		//R2D2->ArcadeDrive(joystick);
		//R2D2->TankDrive(y+1 *z,y- 1 *z,true);
	}

	if(robotMode == MODE_MECA){

		float x= ((float)joystick->GetX());
		float y= -((float)joystick->GetY());
		float z= -((float)joystick->GetZ());

		/*if (x>=-0.2 && x<=0.2)
			x=0;

		if (y>=-0.2 && y<=0.2)
					y=0;

		if (z>=-0.3 && z<=0.3)
					z=0;

		mecaFrontRight.Set(y +zCoeff *z);
		mecaBackRight.Set(y+zCoeff *z);
		mecaFrontLeft.Set(-y+ zCoeff *z);
		mecaBackLeft.Set(-y+ zCoeff *z);*/

		if(x>=-0.2 && x<=0.2)
			x=0;
		if(y>=-0.2 && y<=0.2)
			y=0;
		if(z>=-0.3 && z<=0.3)
			z=0;

		x=x*coeff;
		y*=coeff;
		/*static float angle_voulu=0;
		angle_voulu+=-((float)joystick->GetZ());
		std::cout<<"Angle_Voulu : "<<angle_voulu<<" "<<std::endl;
		float angle=angle_voulu-gyro->GetAngle();
		angle=-angle/90;
		std::cout<<"gyro->GetAngle() : "<<gyro->GetAngle()<<" "<<std::endl;
		std::cout<<"Angle : "<<angle<<" "<<std::endl;*/
		mecaFrontRight.Set(+y+ -x+y);
		mecaBackRight.Set(y+x+y);
		mecaFrontLeft.Set(-y -x +y );
		mecaBackLeft.Set(-y+x+y );

		//R2D2->HolonomicDrive(y,x,z);
		//R2D2->MecanumDrive_Cartesian(x,y,z,angle);
	}
}
void BaseRoulante::meca_droite(double val)
{

				mecaFrontRight.Set(-val);
				mecaBackRight.Set(val);
				mecaFrontLeft.Set(val);
				mecaBackLeft.Set(-val);
}
void BaseRoulante::meca_gauche(double val)
{

				mecaFrontRight.Set(val);
				mecaBackRight.Set(-val);
				mecaFrontLeft.Set(-val);
				mecaBackLeft.Set(val);
}
void BaseRoulante::meca_avancer(double val)
{
				mecaFrontRight.Set(val);
				mecaBackRight.Set(val);
				mecaFrontLeft.Set(val);
				mecaBackLeft.Set(val);
}
void BaseRoulante::resetModeAuto(){
	mode_auto=MODE_APPROACH;
}



BaseRoulante::~BaseRoulante() {
	// TODO Auto-generated destructor stub

}

