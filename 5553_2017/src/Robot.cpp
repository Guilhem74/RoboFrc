#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

#include "WPILib.h"









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
	Servo* Sai;
	AnalogInput *Ai;
	int etat1;
	int etat2;
	int etat3;
	int etat4;
	//bidon

	void RobotInit() {
		Ai= new AnalogInput(1);//Fin de course

		// initialisation des objets et données
		Sai =new Servo(5);//Servo moteur
		 etat1=1;
		 etat2=1;
		 etat3=1;
		 etat4=1;
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
		Sai->SetAngle(-20);
		float x= -Joystick1->GetX();
		float y= -Joystick1->GetY();
		float z= -Joystick1->GetZ();
		if(etat1==0)
		{
			M1->Set(-y+0.5*z);
			M2->Set(-y+0.5*z);
			M3->Set(y+0.5*z);
			M4->Set(y+0.5*z);
		}
		else
		{
			M1->Set(-y-x+0.5*z);
			M2->Set(-y+x+0.5*z);
			M3->Set(x+y+0.5*z);
			M4->Set(-x+y+0.5*z);
		}


		if(Joystick1->GetRawButton(3)){//Mecanum
			etat1=1-etat1;
			if(etat1==0)
				verins_1->Set(frc::DoubleSolenoid::kForward);
			else
				verins_1->Set(frc::DoubleSolenoid::kReverse);

			std::cout<<"Verin1"<<std::endl;
			std::cout<<etat1<<std::endl;
			Wait(1);
		}
		if(Joystick1->GetRawButton(4)){//Lever pince
			etat2=1-etat2;
			if(etat2==0)
			{
				verins_2->Set(frc::DoubleSolenoid::kForward);

			}
			else
			{
				verins_2->Set(frc::DoubleSolenoid::kReverse);

			}

			std::cout<<"Verin2"<<std::endl;
			std::cout<<etat2<<std::endl;
			Wait(1);

		}
		if(Joystick1->GetRawButton(5)){//open close pince
			etat3=1-etat3;
			if(etat3==0)
			{

				verins_3->Set(frc::DoubleSolenoid::kForward);


			}
			else
			{
				verins_3->Set(frc::DoubleSolenoid::kReverse);

			}
			std::cout<<"Verin3"<<std::endl;
			std::cout<<etat3<<std::endl;
			Wait(1);

		}
		if(Joystick1->GetRawButton(6)){//bac
			etat4=1-etat4;
			if(etat4==0)
			{
				verins_4->Set(frc::DoubleSolenoid::kForward);

			}


			{
				verins_4->Set(frc::DoubleSolenoid::kReverse);

			}

			std::cout<<"Verin4"<<std::endl;
			std::cout<<etat4<<std::endl;
			Wait(1);
		}
		if(Joystick1->GetRawButton(7)){
			M5->Set(-0.8);



		}
		if(Joystick1->GetRawButton(8)){
			M5->Set(0);

		}


	}

	void TestPeriodic() {


	}



};

START_ROBOT_CLASS(Robot)
