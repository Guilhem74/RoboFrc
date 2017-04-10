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
float I_COEFF=0.0000000050;
float D_COEFF=0.00007;
int TOLERANCE=10000;


BaseRoulante::BaseRoulante():
mecaFrontLeft(0,0,1,1),mecaBackLeft(1,2,3,1),mecaFrontRight(3,4,5,0),mecaBackRight(2,6,7,0), verins_BASE(4,5)
{
		// arr�t des moteurs
		mecaFrontLeft.Set(0.0);
		mecaFrontRight.Set(0.0);
		mecaBackRight.Set(0.0);
		mecaBackLeft.Set(0.0);
		// configuration mode TANK

		verins_BASE.Set(frc::DoubleSolenoid::kReverse);

		robotMode = MODE_TANK;
		mode_auto = MODE_ALIGN;
		// configuration du robotDrive
		//R2D2 = new RobotDrive(mecaFrontLeft,mecaBackLeft,mecaFrontRight,mecaBackRight);
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
	double moyenneGauche = mecaBackLeft.GetDistance();
	double moyenneDroite = mecaFrontRight.GetDistance() ;

	powerRight=PID_DISTANCE(Consigne_Dist,moyenneDroite)-PID_ANGLE(Consigne_Ang,Angle_gyro);
	powerLeft=-(PID_DISTANCE(Consigne_Dist,moyenneGauche)+PID_ANGLE(Consigne_Ang,Angle_gyro));
	if(counteur_Fin>TOLERANCE)
		return 1;
	mecaFrontLeft.Set(powerLeft);
	mecaBackLeft.Set(powerLeft);
	mecaFrontRight.Set(powerRight);
	mecaBackRight.Set(powerRight);
	return 0;
}



void BaseRoulante::setRobotMode(int mode){
	if(mode == MODE_TANK){
		// rentrer les verins

		verins_BASE.Set(frc::DoubleSolenoid::kForward);
	}
	if(mode == MODE_MECA){
		// pousser les verins
		verins_BASE.Set(frc::DoubleSolenoid::kReverse);

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

		mecaFrontRight.Set(y +zCoeff *z);
		mecaBackRight.Set(y+zCoeff *z);
		mecaFrontLeft.Set(-y+ zCoeff *z);
		mecaBackLeft.Set(-y+ zCoeff *z);

		//R2D2->ArcadeDrive(joystick);
		//R2D2->ArcadeDrive(joystick,frc::Joystick::AxisType::kZAxis,joystick,frc::Joystick::AxisType::kYAxis);
	}

	if(robotMode == MODE_MECA){

		float x= ((float)joystick->GetX());
		float y= -((float)joystick->GetY());
		float z= -((float)joystick->GetZ());

		if(x>=-0.2 && x<=0.2)
			x=0;
		if(y>=-0.2 && y<=0.2)
			y=0;
		if(z>=-0.3 && z<=0.3)
			z=0;

		x=x*coeff;
		y*=coeff;

		mecaFrontRight.Set(+y+ -x+z*zCoeff);
		mecaBackRight.Set(y+x+z*zCoeff);
		mecaFrontLeft.Set(-y -x +z*zCoeff );
		mecaBackLeft.Set(-y+x+z*zCoeff );


		//R2D2->MecanumDrive_Cartesian(x,y,z,angle);
	}
}

void BaseRoulante::resetModeAuto(){
	mode_auto=MODE_APPROACH;
}



BaseRoulante::~BaseRoulante() {
	// TODO Auto-generated destructor stub

}

