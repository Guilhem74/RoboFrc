/*
 * BaseRoulante.h
 *
 *  Created on: 27 déc. 2016
 *      Author: REBAUDET Thomas
 */
#include "WPILib.h"#include <chrono>#include "VictorSPRampe.h"#include <unistd.h>#ifndef SRC_BASEROULANTE_H_
#define SRC_BASEROULANTE_H_

class BaseRoulante {
public:	BaseRoulante();
	virtual ~BaseRoulante();	void setRobotMode(int);	int getRobotMode();
	void mvtJoystick(Joystick*, ADXRS450_Gyro* );	void deposeRoueAuto(Joystick* , ADXRS450_Gyro*, Ultrasonic*,Ultrasonic*);	void resetModeAuto();	void mvtTreuil(	Joystick* joystick);	int Rampe(int x);	VictorSP mecaFrontLeft;	VictorSP mecaFrontRight;	VictorSP mecaBackRight;	VictorSP mecaBackLeft;	VictorSP treuil;	DoubleSolenoid verins_BASE;	Joystick* joystick;	double approach_speed;	double align_dist;	double align_marge;	double rot_speed;	double rot_marge;
private:	typedef std::chrono::high_resolution_clock Time;	typedef std::chrono::duration<float> deltaT;	typedef std::chrono::milliseconds ms;	int robotMode;
	float zCoeff = 0.5;	int mode_auto;	std::chrono::_V2::system_clock::time_point t0 = Time::now();	float previousPower = 0.0f;	float xprecedent=0.0f;	float coeffAcceleration=0.003f;	int powerActuel;
};

#endif /* SRC_BASEROULANTE_H_ */
