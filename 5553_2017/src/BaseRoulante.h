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

	void mvtJoystick(Joystick*, ADXRS450_Gyro*, double angleini );
	void deposeRoueAuto(Joystick* , ADXRS450_Gyro*, Ultrasonic*,Ultrasonic*);
	void resetModeAuto();
	void getEnc();
	void Avancenul();
	void ReculeNul();
	void meca_droite(double val);
	void meca_gauche(double val);
	void meca_avancer(double val);

	void meca_tourne_gauche(double val);
	void meca_tourne_droite(double val);



	void setConsigne(double Longueur, double Angle);
	int effectuerConsigne(double Angle_gyro);
	void SetVitesseMax(double max);
	void SetPID(double P_val,double I_val, double D_val);
	void reset();
	double PID_ANGLE(double Angle, double Angle_gyro);
	double PID_DISTANCE(double consigne_L, double valeur_Encodeur);
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
	 int counteur_Fin=0;

	static const int Nintegration=80;
	int indiceIntegration=0;
	double Consigne_Dist=0, Consigne_Ang=0;
	double erreursD=0;
	double erreursG=0;
	double sommeErreursG=0;
		double sommeErreursD=0;
	double P=0.00015, I=0.0, D=0.0;
	double Erreur_Precedente_G=0, Erreur_Precedente_D=0; // erreurs précédentes
	float powerLeft=0;
	float powerRight=0;
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
};


#endif /* SRC_BASEROULANTE_H_ */

