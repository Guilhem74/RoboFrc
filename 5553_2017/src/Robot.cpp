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
	Encoder* enc1;
	Encoder* enc2;
	Encoder* enc3;
	Encoder* enc4;
	//Ultrasonic *ultra; // creates the ultra object

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
		enc1 = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
				enc1->SetMaxPeriod(.1);
				enc1->SetMinRate(10);
				enc1->SetDistancePerPulse(5);
				enc1->SetReverseDirection(true);
				enc1->SetSamplesToAverage(7);
				enc2 = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
				enc2->SetMaxPeriod(.1);
				enc2->SetMinRate(10);
				enc2->SetDistancePerPulse(5);
				enc2->SetReverseDirection(true);
				enc2->SetSamplesToAverage(7);
				enc3 = new Encoder(4, 5, false, Encoder::EncodingType::k4X);
				enc3->SetMaxPeriod(.1);
				enc3->SetMinRate(10);
				enc3->SetDistancePerPulse(5);
				enc3->SetReverseDirection(true);
				enc3->SetSamplesToAverage(7);
				enc4 = new Encoder(6, 7, false, Encoder::EncodingType::k4X);
				enc4->SetMaxPeriod(.1);
				enc4->SetMinRate(10);
				enc4->SetDistancePerPulse(5);
				enc4->SetReverseDirection(true);
				enc4->SetSamplesToAverage(7);
	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		//Sai->SetAngle(180);
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


		if(Joystick1->GetRawButton(1)){//Mecanum
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
		if(Joystick1->GetRawButton(3)){//open close pince
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
			else
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
						//		int range = ultra->GetRangeInches(); // reads the range on the ultrasonic sensor
		//		std::cout<<"Range:";
		//		std::cout<<range<<std::endl;
		std::cout<<"Encodeur01: ";
						std::cout<<enc1->Get();
						std::cout<<"  Encodeur02: ";
						std::cout<<enc2->Get();
						std::cout<<"  Encodeur03: ";
						std::cout<<enc3->Get();
						std::cout<<"  Encodeur04: ";
						std::cout<<enc4->Get()<<std::endl;


	}

	void TestPeriodic() {


	}



};

START_ROBOT_CLASS(Robot)
