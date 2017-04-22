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

float P_COEFF_A=0.04;//0.017
int TOLERANCE=150;


BaseRoulante::BaseRoulante():
mecaFrontLeft(0,0,1,1),mecaBackLeft(1,2,3,1),mecaFrontRight(3,4,5,0),mecaBackRight(2,6,7,0), verins_BASE(2,3)
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
	return P_COEFF_L*erreur;

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


void BaseRoulante::mvtJoystick(Joystick *joystick, ADXRS450_Gyro* gyro, double angleini)
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
/*
	if(robotMode == MODE_MECA){

		float x= -((float)joystick->GetX());
		float y= -((float)joystick->GetY());
		float z= -((float)joystick->GetZ());

		if(x>=-0.2 && x<=0.2)
			x=0;
		if(y>=-0.2 && y<=0.2)
			y=0;


		double angle = gyro->GetAngle();
		anglevoulu +=z;
		double anglecalc = anglevoulu + angle;
		anglecalc = (anglecalc/10);
		std::cout<<"gyro"<<angle<<std::endl;
		std::cout<<"angle_voulu"<<anglevoulu<<std::endl;
		std::cout<<"angle_calc"<<anglecalc<<std::endl;
		std::cout<<"FrontRight"<<y+ -x + anglecalc<<std::endl;
		std::cout<<"BackRight"<<y+x + anglecalc<<std::endl;
		std::cout<<"FrontLeft"<<-y -x + anglecalc<<std::endl;
		std::cout<<"BackLeft"<<-y+x+anglecalc<<std::endl;
*//*		mecaFrontRight.Set(y+ -x + anglecalc);
		mecaBackRight.Set(y+x + anglecalc);
		mecaFrontLeft.Set(-y -x + anglecalc);
		mecaBackLeft.Set(-y+x+anglecalc);*/
/*		(mecaFrontRight.getVictorSP())->Set(y+ -x + anglecalc);
		(mecaBackRight.getVictorSP())->Set(y +x + anglecalc);
		(mecaFrontLeft.getVictorSP())->Set(-y -x + anglecalc);
		(mecaBackLeft.getVictorSP())->Set(-y +x +anglecalc);

*/
		//R2D2->MecanumDrive_Cartesian(x,y,z,angle);
/*	}
}
*/





if(robotMode == MODE_MECA){

		/*float x= ((float)joystick->GetX());
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
		*/
		//R2D2->MecanumDrive_Cartesian(x,y,z,angle);

		float x= ((float)joystick->GetX());
		float y= ((float)joystick->GetY());
		float z= ((float)joystick->GetZ());

		ConvertJoystick(0.4,y,z+0.1);




	}
}


double BaseRoulante::Ecrete(double val)
{
	val = val > 1 ? 1 : val;
	return val;
}




void BaseRoulante::ConvertJoystick(double x, double y, double z)
{

	//Dead Space
	double xyDeadSpace = 0.1;
	double zDeadSpace = 0.25;

        //Check that the position is outside the deadspace
	double newx = (fabs(x) - xyDeadSpace) * (xyDeadSpace+1);
	if (newx < 0) newx = 0;
	x = (x < 0) ? -newx : newx;
	double newy = (fabs(y) - xyDeadSpace) * (xyDeadSpace + 1);
	if (newy < 0) newy = 0;
	y = (y < 0) ? -newy : newy;
	double newz = (fabs(z) - zDeadSpace) * (zDeadSpace + 1);
	if (newz < 0) newz = 0;
	z = (z < 0) ? -newz : newz;

	x = Ecrete(x);
	y = Ecrete(y);
	z = Ecrete(z);

	double magnitude = sqrt(x * x + y * y);
	double direction = atan2(x, y);
	double rotation = z;

	if (magnitude == 0)
		direction = 0;

	//Into degrees
	direction = direction * 180.0 / 3.1415;

	/*if (gyroLocked)
	{
		direction -= heading;
		direction += headingLockPoint;
	}*/

	if (direction < 0)
		direction += 360;
	if (direction > 360)
		direction -= 360;
	std::cout<<"magitude"<<magnitude<<"direction"<<direction<<"rotation"<<rotation<<std::endl;
	NewMecanumDrive(magnitude, direction, rotation);
}





void BaseRoulante::NewMecanumDrive(double magnitude, double direction, double rotation)
{
	//Limit limits magnitude to 1.0
	magnitude = Ecrete(magnitude);

	// Normalized for full power along the Cartesian axes.
	magnitude = magnitude * sqrt(2.0);

	// The rollers are at 45 degree angles.
	double dirInRad = (direction + 45.0) * 3.1415 / 180.0;
	double cosD = cos(dirInRad);
	double sinD = sin(dirInRad);
	double wheelSpeeds [4];

	wheelSpeeds[0] = sinD * magnitude + rotation;
	wheelSpeeds[1] = cosD * magnitude - rotation;
	wheelSpeeds[2] = cosD * magnitude + rotation;
	wheelSpeeds[3] = sinD * magnitude - rotation;


	wheelSpeeds[0] = Ecrete(wheelSpeeds[0]);
	wheelSpeeds[1] = Ecrete(wheelSpeeds[1]);
	wheelSpeeds[2] = Ecrete(wheelSpeeds[2]);
	wheelSpeeds[3] = Ecrete(wheelSpeeds[3]);

	mecaFrontLeft.Set(wheelSpeeds[0]);
	mecaFrontRight.Set(-wheelSpeeds[1]);
	mecaBackLeft.Set(wheelSpeeds[2]);
	mecaBackRight.Set(-wheelSpeeds[3]);

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
void BaseRoulante::meca_tourne_droite(double val)
{
				mecaFrontRight.Set(-val);
				mecaBackRight.Set(-val);
				mecaFrontLeft.Set(-val);
				mecaBackLeft.Set(-val);
}
void BaseRoulante::meca_tourne_gauche(double val)
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

