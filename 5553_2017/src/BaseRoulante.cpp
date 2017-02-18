/*
 * BaseRoulante.cpp
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
mecaFrontLeft(0),mecaFrontRight(1),mecaBackRight(2),mecaBackLeft(3),verins_AV(0,1),verins_AR(2,3)
{
		// arrêt des moteurs
		mecaFrontLeft.Set(0.0);
		mecaFrontRight.Set(0.0);
		mecaBackRight.Set(0.0);
		mecaBackLeft.Set(0.0);
		// configuration mode TANK
		verins_AV.Set(frc::DoubleSolenoid::kReverse);
		verins_AR.Set(frc::DoubleSolenoid::kReverse);
		robotMode = MODE_TANK;
		mode_auto = MODE_ALIGN;
		// configuration du robotDrive
		R2D2 = new RobotDrive(mecaFrontLeft,mecaBackLeft,mecaFrontRight,mecaBackRight);
		approach_speed = 0.5;
		align_dist = 100; // en mm
		align_marge = 20; // en mm
		rot_marge = 10; // en mm
		rot_speed = 0.3; // entre -1 et 1

		// pour le centrage automatique avec camera
		timeout = 0;
		position_robot_camera = 0;

}


void BaseRoulante::setRobotMode(int mode){
	if(mode == MODE_TANK){
		// rentrer les verins
		verins_AV.Set(frc::DoubleSolenoid::kReverse);
		verins_AR.Set(frc::DoubleSolenoid::kReverse);
	}
	if(mode == MODE_MECA){
		// pousser les verins
		verins_AV.Set(frc::DoubleSolenoid::kForward);
		verins_AR.Set(frc::DoubleSolenoid::kForward);
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
		R2D2->ArcadeDrive(joystick);
		//R2D2->ArcadeDrive(joystick,frc::Joystick::AxisType::kYAxis,joystick,frc::Joystick::AxisType::kZAxis);
	};

	if(robotMode == MODE_MECA){
		double angle=gyro->GetAngle();
		float x= (float)joystick->GetX();
		float y= (float)joystick->GetY();
		float z= (float)joystick->GetZ();
		R2D2->MecanumDrive_Cartesian(x,y,z,angle);
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
			R2D2->SetLeftRightMotorOutputs(approach_speed,approach_speed);
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
		if (!(timeout > TIMEOUT || TIMEOUT_ACTIF))
		{
			if (nb_lignes > 2)
				{
					//traitement pour trouver les deux rectangles principaux


				}
				else if (nb_lignes == 2)
				{
					// On centre le robot par rapport aux deux rectangles
					// calcul de la position du robot par rapport aux bandes
					position_robot_camera = (rect[0].x1+rect[1].x1)/2;
					if (position_robot_camera > TAILLE_IMAGE_CAMERA/2)
					{
						DeplaceAutoDroite();
					}
					else
					{
						DeplaceAutoGauche();
					}

				}
				else if (nb_lignes == 1)
				{
					// un seul rectangle, on essaye de trouver le deuxième
					// on regarde la position du rectangle par rapport au milieu de l'image
					// TODO optimiser en prennant en compte l'épaisseur du du rectangle
					if (rect[0].x1 > TAILLE_IMAGE_CAMERA/2)
					{
						DeplaceAutoDroite();
						timeout += 1;
					}
					else
					{
						DeplaceAutoGauche();
						timeout +=1;
					}
				}
				else
				{
					printf("Pas de rectangles repérés, passage en mode manuel ?");
				}
			}
		break;
	}
}


void BaseRoulante::DeplaceAutoDroite(){
	//TODO prendre en compte le nombre de tick de codeuse
	//TODO conversion en cm ?
	R2D2->MecanumDrive_Cartesian(VITESSE_AUTO,-1,0);//ajuster le -1 pour le deplacement à droite ou a gauche
}

void BaseRoulante::DeplaceAutoGauche(){
	//TODO prendre en compte le nombre de tick de codeuse
	//TODO conversion en cm ?
	R2D2->MecanumDrive_Cartesian(VITESSE_AUTO,1,0);//ajuster le 1 pour le deplacement à droite ou a gauche
}




BaseRoulante::~BaseRoulante() {
	// TODO Auto-generated destructor stub

}

