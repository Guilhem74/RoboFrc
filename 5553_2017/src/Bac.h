/*
 * Bac.h
 *
 *  Created on: 18 févr. 2017
 *      Author: REBAUDET Thomas
 */
#include "WPILib.h"
#include <DoubleSolenoid.h>

#ifndef BAC_H_
#define BAC_H_



class Bac {

public:
	Bac();
	virtual ~Bac();
	int m_statut;
	DoubleSolenoid m_verinBac;
	void leverBac();
	void rentrerBac();
};



#endif /* BAC_H_ */
