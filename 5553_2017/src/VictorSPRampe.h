/*
 * VictorSPRampe.h
 *
 *  Created on: 20 févr. 2016
 *      Author: Mayeul
 */

#ifndef SRC_VICTORSPRAMPE_H_
#define SRC_VICTORSPRAMPE_H_

#include <chrono>
#include "WPILib.h"
#include <unistd.h>
#include <math.h>

#define incrementMax 0.2f // acceleration maximale en cas de problème

#define diametreMillimetre 101 // diamètre en mm, (calcul du pas par tick dans BaseRoulante.cpp)


class VictorSP_Rampe{
public:
	VictorSP_Rampe(uint32_t channel_VictorSP, uint32_t A_encode, uint32_t B_encode, bool invert);
	void Set(float power);
	void SetVitesse(double vitesse);
	void setCoeffAcceleration(float coeff);
	float getCoeffAcceleration();
	virtual ~VictorSP_Rampe();
	double GetDistance();
	void Reset(){codeur.Reset();} // réinitialise l'encodeur
	void SetVitesseMax(double max){vitesseMax=max;}

private:
	void mettreAJourVitesse();

	typedef std::chrono::high_resolution_clock Time;
	typedef std::chrono::milliseconds ms;
	typedef std::chrono::duration<float> fsec;
	typedef std::chrono::_V2::system_clock::time_point time_point;

	int signe(int delta);
	float signe(float delta);
	double signe(double delta);

	VictorSP moteur;
	Encoder codeur;
	float coeffAcceleration = 0.003f; // = 1/temps_pour_atteindre_la_vitesse_max_ms
	float previousPower = 0.0f; // Puissance précédemment envoyée aux moteurs pour le calcul de la rampe
	time_point t0 = Time::now(); // Temps précédent pour le calcul de la rampe

	time_point previousTime = Time::now(); // Temps précédent pour a rampe de vitesse ou de puissance
	double P=1.0, D=0.1; // Coefficients pour l'asservissement en vitesse
	double speed = 0; //vitesse en millimetre par milliseconde (m/s)
	double ecartVitessePrecedent=0;
	double coeffAccelerationVitesse = 0.5; // m/s/s ou m/s²
	double vitesseMax = 10; // m/s


};

#endif /* SRC_VICTORSPRAMPE_H_ */
