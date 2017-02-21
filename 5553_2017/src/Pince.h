/*
 * Pince.h
 *
 *  Created on: 18 févr. 2017
 *      Author: REBAUDET Thomas
 */
#include "WPILib.h"
#include "Bac.h"

#ifndef SRC_PINCE_H_
#define SRC_PINCE_H_



class Pince {

public:
	Pince();
	virtual ~Pince();
	DoubleSolenoid m_verinLev;
	DoubleSolenoid m_verinSer;
	void serrerPince();
	void desserrerPince();
	void leverPince();
    void abaisserPince();
    AnalogInput* limitSwitch;




};



#endif /* SRC_PINCE_H_ */
