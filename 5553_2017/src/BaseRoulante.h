/*

 * BaseRoulante.h

 *

 *  Created on: 27 d�c. 2016

 *      Author: REBAUDET Thomas

 */


#include "WPILib.h"
#include <chrono>
#include "VictorSPRampe.h"
#include <unistd.h>

#ifndef SRC_BASEROULANTE_H_


#define SRC_BASEROULANTE_H_


class BaseRoulante {

public:
	BaseRoulante();
	virtual ~BaseRoulante();
	void setRobotMode(int);
	int getRobotMode();

	void mvtJoystick(Joystick*, ADXRS450_Gyro* );
	void deposeRoueAuto(Joystick* , ADXRS450_Gyro*, Ultrasonic*,Ultrasonic*);
	void resetModeAuto();

	void parcourirDistance(double distanceGauche, double distanceDroite);
	double effectuerConsigne(double P, double I, double D);
	void SetVitesseMax(double max);
	void reset();

	int Rampe(int x);
	VictorSP_Rampe mecaFrontLeft;
	VictorSP_Rampe mecaBackLeft;
	VictorSP_Rampe mecaFrontRight;
	VictorSP_Rampe mecaBackRight;
	DoubleSolenoid verins_BASE;
	double approach_speed;
	double align_dist;
	double align_marge;
	double rot_speed;
	double rot_marge;
	int count;
	static const int Nintegration=80;
	int indiceIntegration=0;
	double consigneD=0, consigneG=0;
	double erreursD[Nintegration]={0};
	double erreursG[Nintegration]={0};
	double PerreurD, PerreurG; // erreurs précédentes
	float powerLeft;
	float powerRight;


private:
	typedef std::chrono::high_resolution_clock Time;
	typedef std::chrono::duration<float> deltaT;
	typedef std::chrono::milliseconds ms;
	int robotMode;
	float zCoeff = 0.5;
	int mode_auto;
	std::chrono::_V2::system_clock::time_point t0 = Time::now();
	float previousPower = 0.0f;
	float xprecedent=0.0f;
	float coeffAcceleration=0.003f;
	int powerActuel;
};


#endif /* SRC_BASEROULANTE_H_ */

