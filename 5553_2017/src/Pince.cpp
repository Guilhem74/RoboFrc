/*
 * Pince.cpp
 *
 *  Created on: 18 févr. 2017
 *      Author: REBAUDET Thomas
 */

#include "WPILib.h"
#include <RobotDrive.h>
#include <DoubleSolenoid.h>
#include "constantes.h"
#include "Pince.h"
#include "Bac.h"

Pince::Pince(): m_verinLev(4,5),m_verinSer(6,7)
{
	// TODO Auto-generated constructor stub
	m_verinLev.Set(frc::DoubleSolenoid::kReverse);
	m_verinSer.Set(frc::DoubleSolenoid::kReverse);
	limitSwitch = new DigitalInput(1);
}
Pince::~Pince()
{

}

void Pince::serrerPince()
{
	    m_verinSer.Set(frc::DoubleSolenoid::kForward);
}

void Pince::desserrerPince()
{
		m_verinSer.Set(frc::DoubleSolenoid::kReverse);

}

void Pince::leverPince(Bac *monBac)
{


	if(monBac->m_statut==BAC_HAUT)

		m_verinLev.Set(frc::DoubleSolenoid::kReverse);
	else
		std::cout<<"attention lever le bac avant la pince "<<std::endl;

}

void Pince::abaisserPince()
{
		m_verinLev.Set(frc::DoubleSolenoid::kReverse);

}

