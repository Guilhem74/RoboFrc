#ifndef CONSTANTEH
#define CONSTANTEH
#define MODE_TANK 0
#define MODE_MECA 1


#define BTN_TANK 1
#define BTN_MECA 2
#define BTN_DEPOSE_ROUE_AUTO 7


#define MODE_APPROACH 0
#define MODE_ALIGN 12
#define MODE_CENTER 11

#define BAC_HAUT 1
#define BAC_BAS 0

#define P_COEFF_L 0.00015
#define TOLERANCE 150
/* TABLEAU DE CABLAGE du ROBORIO
 *
 * 	RS-232	->
 *
 * 	DIO-0	->	ULTRASON AV-Gauche (ping)
 * 	DIO-1	->	ULTRASON AV-Gauche (echo)
 * 	DIO-2	->	ULTRASON AV-Droite (ping)
 * 	DIO-3	->	ULTRASON AV-Droite (echo)
 * 	DIO-4	->
 * 	DIO-5	->
 * 	DIO-6	->
 * 	DIO-7	->
 * 	DIO-8	->
 * 	DIO-9	->
 *
 *  RELAY-0	->
 *  RELAY-1	->
 *  RELAY-2	->
 *  RELAY-3	->
 *
 *  ANALOG IN -0 	->
 *  ANALOG IN -1 	->
 *  ANALOG IN -2 	->
 *  ANALOG IN -3 	->
 *
 *
 * 	PWM-0	-> victorSP AV-Gauche
 * 	PWM-1	-> victorSP AV-Droite
 * 	PWM-2	-> victorSP AR-Gauche
 * 	PWM-3	-> victorSP AR-Droite
 * 	PWM-4	->
 * 	PWM-5	->
 * 	PWM-6	->
 * 	PWM-7	->
 * 	PWM-8	->
 * 	PWM-9	->
 *
 */


/* TABLEAU DE CABLAGE du PCM (Pneumatic Control Module)
 *

 * PWM-0	-> verrin BAC_UP
 * PWM-1	-> verrin BAC_DOWN
 * PWM-2	->
 * PWM-3	->
 * PWM-4
 * PWM-5

 * PWM-6
 * PWM-7
 *
 */
#endif

