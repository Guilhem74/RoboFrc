#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include "WPILib.h"

#include <Encoder.h>




#include <thread>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>



class Robot: public frc::IterativeRobot {
public:

	// déclaration des capteurs et actionneurs
	Joystick* Joystick1;
	VictorSP* M1;
	VictorSP* M2;
	VictorSP* M3;
	VictorSP* M4;
	VictorSP* M5;
	DoubleSolenoid* verins_1;
	DoubleSolenoid* verins_2;
	DoubleSolenoid* verins_3;
	DoubleSolenoid* verins_4;

	//bidon

	void RobotInit() {

		// initialisation des objets et données

		Joystick1 = new Joystick(0);								// à connecter sur port USB0
		M1= new VictorSP(0);
		M2= new VictorSP(1);
		M3= new VictorSP(2);
		M4= new VictorSP(3);
		M5= new VictorSP(4);
		verins_1= new DoubleSolenoid(0,1);
		verins_2= new DoubleSolenoid(2,3);
		verins_3= new DoubleSolenoid(4,5);
		verins_4= new DoubleSolenoid(6,7);

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
/*M1= new VictorSP(0);
		M2= new VictorSP(1);
		M3= new VictorSP(2);
		M4= new VictorSP(3);
		M5= new VictorSP(4);*/
		M1->Set(0.9);
		if(Joystick1->GetRawButton(1)){
			M1->Set(0.6);
			M2->Set(0.6);
			M3->Set(0.6);
			M4->Set(0.6);

		}
		else
		{
			M1->Set(0);
			M2->Set(0);
			M3->Set(0);
			M4->Set(0);

		}
		if(Joystick1->GetRawButton(2)){
			M4->Set(0.9);
		}
		else
		{
			M4->Set(0);
		}
		if(Joystick1->GetRawButton(3)){
			verins_1->Set(frc::DoubleSolenoid::kForward);
			verins_2->Set(frc::DoubleSolenoid::kForward);
			verins_3->Set(frc::DoubleSolenoid::kForward);
			verins_4->Set(frc::DoubleSolenoid::kForward);



		}
		if(Joystick1->GetRawButton(4)){
			verins_1->Set(frc::DoubleSolenoid::kReverse);
			verins_2->Set(frc::DoubleSolenoid::kReverse);
			verins_3->Set(frc::DoubleSolenoid::kReverse);
			verins_4->Set(frc::DoubleSolenoid::kReverse);
		}



	}

	void TestPeriodic() {


	}



};

START_ROBOT_CLASS(Robot)
