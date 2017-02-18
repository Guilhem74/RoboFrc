/*
 * BaseRoulante.cppS
 *
 *  Created on: 27 déc. 2016
 *      Author: REBAUDET Thomas
 */

#include "WPILib.h"
#include <RobotDrive.h>
#include <BaseRoulante.h>
#include <DoubleSolenoid.h>
#include <constantes.h>

BaseRoulante::BaseRoulante():
mecaFrontLeft(0,0,1),mecaBackLeft(1,2,3),mecaFrontRight(2,4,5),mecaBackRight(3,6,7),
verins_AV(6,7)
{
		// arrêt des moteurs
		mecaFrontLeft.Set(0.0);
		mecaFrontRight.Set(0.0);
		mecaBackRight.Set(0.0);
		mecaBackLeft.Set(0.0);
		// configuration mode TANK
		verins_AV.Set(frc::DoubleSolenoid::kReverse);
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

int BaseRoulante::Rampe(int x)
{
	 auto t1 = Time::now();
	 deltaT deltaT=t1-t0;
	 t0=t1;
	 ms d = std::chrono::duration_cast<ms>(deltaT);
	 float deltaMillis=float(d.count());

	 int deltaX=x-xprecedent;
	 xprecedent=x;

	 float increment = (deltaMillis*coeffAcceleration);


	 if (deltaX >0)
	 {
		  powerActuel=previousPower+increment;
	 }
	 else if (deltaX<0)
	 {
		  powerActuel=previousPower-increment;
	 }

	 previousPower=powerActuel;

	 return powerActuel;
}

void BaseRoulante::setRobotMode(int mode){
	if(mode == MODE_TANK){
		// rentrer les verins
		verins_AV.Set(frc::DoubleSolenoid::kReverse);
	}
	if(mode == MODE_MECA){
		// pousser les verins
		verins_AV.Set(frc::DoubleSolenoid::kForward);
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

		mecaFrontRight.Set(-y- zCoeff *z);
		mecaBackRight.Set(-y- zCoeff *z);
		mecaFrontLeft.Set(-y+ zCoeff *z);
		mecaBackLeft.Set(-y+ zCoeff *z);

		//R2D2->ArcadeDrive(joystick);
		//R2D2->ArcadeDrive(joystick,frc::Joystick::AxisType::kZAxis,joystick,frc::Joystick::AxisType::kYAxis);
	};

	if(robotMode == MODE_MECA){
		double angle=gyro->GetAngle();
		float x= -((float)joystick->GetX());
		float y= -((float)joystick->GetY());
		float z= -((float)joystick->GetZ());

		if(x>=-0.2 && x<=0.2)
			x=0;
		if(y>=-0.2 && y<=0.2)
			y=0;
		if(z>=-0.3 && z<=0.3)
			z=0;

		mecaFrontRight.Set(-y- x-z);
		mecaBackRight.Set(-y+x-z);
		mecaFrontLeft.Set(-y +x +z);
		mecaBackLeft.Set(-y-x+z);





		std::cout<<"x: "<<x<<std::endl;
		std::cout<<"y: "<<y<<std::endl;
		std::cout<<"z: "<<z<<std::endl;

		//R2D2->MecanumDrive_Cartesian(x,y,z,angle);
	};
}

void BaseRoulante::resetModeAuto(){
	mode_auto=MODE_APPROACH;
}


void BaseRoulante::deposeRoueAuto(Joystick* joystick, ADXRS450_Gyro*gyro, Ultrasonic* ultrason_G, Ultrasonic* ultrason_D){

	switch (mode_auto){

	case MODE_APPROACH :
		// tant que pas assez pres du mur: on avance
		if((ultrason_G->GetRangeMM() > (align_dist+align_marge)) && (ultrason_D->GetRangeMM() > (align_dist+align_marge)))
			R2D2->SetLeftRightMotorOutputs(approach_speed,-approach_speed);
		else{
		// sinon on coupe les moteurs et on passe en mode alignement
			R2D2->StopMotor();
			mode_auto=MODE_ALIGN;
		}
		break;

	case MODE_ALIGN :
		// on passe en mode MECANUM si pas dejà fait
		if(robotMode == MODE_TANK)
			setRobotMode(MODE_MECA);
		// Si le capteur gauche est supérieur à capteur droit: on tourne à droite
		if (ultrason_G->GetRangeMM() > (ultrason_D->GetRangeMM() + rot_marge))
			R2D2->MecanumDrive_Cartesian(0,0,rot_speed);
		else
			//Si le capteur gauche est inferieur à capteur droit: on tourne à gauche
			if (ultrason_G->GetRangeMM() < (ultrason_D->GetRangeMM() - rot_marge))
				R2D2->MecanumDrive_Cartesian(0,0,-rot_speed);
			else{
				// si on est dans la zone de marge: on passe en mode centrage
				R2D2->StopMotor();
				mode_auto=MODE_CENTER;
			}
		break;

	case MODE_CENTER :
		// Si pas de cible dans image: afficher message pour centrage manuel
		// if(target.lenght()==0)

		break;
	}
}

BaseRoulante::~BaseRoulante() {
	// TODO Auto-generated destructor stub

}

