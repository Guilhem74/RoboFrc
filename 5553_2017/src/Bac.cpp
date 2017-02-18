/*
 * Bac.cpp
 *
 *  Created on: 18 févr. 2017
 *      Author: REBAUDET Thomas
 */
#include "WPILib.h"
#include <DoubleSolenoid.h>
#include <RobotDrive.h>
#include "Bac.h"
#include "constantes.h"


Bac::Bac(): m_verinBac(2,3)
{
	// TODO Auto-generated constructor stub
	//m_verinBac.Set(frc::DoubleSolenoid::kReverse);
	//m_statut = 0;//BTN_BAC_DOWN;
}


void Bac::leverBac()
{

	m_verinBac.Set(frc::DoubleSolenoid::kForward);
	m_statut=BTN_BAC_UP;

}
void Bac::rentrerBac()
{

	m_verinBac.Set(frc::DoubleSolenoid::kReverse);
	m_statut = BTN_BAC_DOWN;
}
