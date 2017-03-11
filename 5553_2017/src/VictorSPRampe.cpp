/*
 * VictorSPRampe.cpp
 *
 *  Created on: 20 févr. 2016
 *      Author: Mayeul
 */

#include <VictorSPRampe.h>
#include "Encoder.h"

VictorSP_Rampe::VictorSP_Rampe(
		uint32_t channel_VictorSP,
		uint32_t A_encode,
		uint32_t B_encode,
		bool invert)
: moteur(channel_VictorSP),
  codeur(A_encode, B_encode,invert,Encoder::k4X)
{
	codeur.SetSamplesToAverage(5);
	codeur.SetMinRate(1.0);
	codeur.SetDistancePerPulse(diametreMillimetre*M_PI/360);
}

VictorSP_Rampe::~VictorSP_Rampe() {
	// TODO Auto-generated destructor stub
}

//retourne le signe de delta 1 si positif, 0 sinon
int VictorSP_Rampe::signe(int delta)
{
	if(delta>=0)return 1;
	return -1;
}

double VictorSP_Rampe::signe(double delta)
{
	if(delta>=0)return 1;
	return -1;
}

double VictorSP_Rampe::GetDistance()
{
	return codeur.GetDistance();
}

void VictorSP_Rampe::mettreAJourVitesse()
{
	time_point currentTime = Time::now();
	fsec fs = currentTime - previousTime;
	previousTime=currentTime;
	ms d = std::chrono::duration_cast<ms>(fs);
	double deltaSecondes=float(d.count());

	speed = (codeur.GetDistance())/(deltaSecondes);
}

float VictorSP_Rampe::signe(float delta)
{
	if(delta>=0)return 1;
	return -1;
}

//#define RAMPE_DESACTIVER
void VictorSP_Rampe::SetVitesse(double vitesseConsigne)
{
/*#ifdef RAMPE_DESACTIVER
	static bool message_printed=false;
	if(!message_printed)
	{
		std::cerr<<"ATTENTION, Rampe désactivée"<<std::endl;
		std::cout<<"ATTENTION, Rampe désactivée"<<std::endl;
		message_printed=true;
	}
	moteur.Set(power);
	return;
#endif*/
	double vitessePrecedente=speed;
	mettreAJourVitesse();

	auto t1 = Time::now();
	fsec fs = t1 - t0;
	t0=t1;
	ms d = std::chrono::duration_cast<ms>(fs);

	/* rampe de vitesse et écrêtage*/
	if(std::abs(vitesseConsigne) > vitesseMax)
		vitesseConsigne = vitesseMax*signe(vitesseConsigne);
	double incrementVitesse=double(d.count())*coeffAccelerationVitesse/1000;
	if(std::abs(vitesseConsigne-vitessePrecedente)>incrementVitesse)
		vitesseConsigne = vitessePrecedente + incrementVitesse*double(signe(vitesseConsigne-vitessePrecedente));

	/*PID*/
	double erreur=vitesseConsigne-speed;
	double deltaErreurs=(erreur-ecartVitessePrecedent)/double(d.count());

	ecartVitessePrecedent=erreur;
	double Power=P*erreur+D*deltaErreurs;

	moteur.Set(float(Power));
}


void VictorSP_Rampe::Set(float power)
{
/*#ifdef RAMPE_DESACTIVER
	static bool message_printed=false;
	if(!message_printed)
	{
		std::cerr<<"ATTENTION, Rampe désactivée"<<std::endl;
		std::cout<<"ATTENTION, Rampe désactivée"<<std::endl;
		message_printed=true;
	}
	moteur.Set(power);
	return;
#endif*/
	mettreAJourVitesse();
	auto t1 = Time::now();

	fsec fs = t1 - t0;
	t0=t1;
	ms d = std::chrono::duration_cast<ms>(fs);
	float deltaMillis=float(d.count());

	float delta = (power-previousPower);
	float increment = coeffAcceleration*deltaMillis*signe(delta);

	if (std::abs(increment)>fabs(delta))
		increment=delta;

	if(std::abs(increment) > incrementMax) // Ecretage en cas de pépin
		increment=incrementMax*signe(increment);

	previousPower += increment;

	moteur.Set(previousPower);
}

void VictorSP_Rampe::setCoeffAcceleration(float coeff)
{
	if(coeff<0)
		return;
	coeffAcceleration=coeff;
}

float VictorSP_Rampe::getCoeffAcceleration()
{
	return coeffAcceleration;
}
