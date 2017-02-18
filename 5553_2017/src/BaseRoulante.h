/*
 * BaseRoulante.h
 *
 *  Created on: 27 déc. 2016
 *      Author: REBAUDET Thomas
 */
#include "WPILib.h"#ifndef SRC_BASEROULANTE_H_
#define SRC_BASEROULANTE_H_

class BaseRoulante {
public:	BaseRoulante();
	virtual ~BaseRoulante();	void setRobotMode(int);	int getRobotMode();
	void mvtJoystick(Joystick*, ADXRS450_Gyro* );	void deposeRoueAuto(Joystick* , ADXRS450_Gyro*, Ultrasonic*,Ultrasonic*);	void resetModeAuto();	void DeplaceAutoDroite();	void DeplaceAutoGauche();    VictorSP mecaFrontLeft;	VictorSP mecaFrontRight;	VictorSP mecaBackRight;	VictorSP mecaBackLeft;	RobotDrive* R2D2;	DoubleSolenoid verins_AV;	DoubleSolenoid verins_AR;	double approach_speed;	double align_dist;	double align_marge;	double rot_speed;	double rot_marge;	// Pour la fonction de centrage automatique avec camera	int timeout = 0;	int nb_rectangles;	int position_robot_camera;
private:	int robotMode;
	float zCoeff = 0.5;	int mode_auto;
};

#endif /* SRC_BASEROULANTE_H_ */
