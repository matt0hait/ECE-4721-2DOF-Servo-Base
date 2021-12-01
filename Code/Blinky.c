/* 	Matthew Hait
		ECE 4721
		F 2021
		Final Project */
		
#include "MKL25Z4.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
 
#define MASK(x) (1UL << (x))
#define DOF		2							// @brief Degrees of Freedom
#define PWM_PERIOD (48000) 	// @brief Period of PWM
#define F_TPM_CLOCK 48000000// @brief Running frequency of the MCU [Hz]
#define F_TPM_OVFLW	250			// @brief Frequency to run TP2 interrupt [Hz]
#define TPM2_PERIOD	4000		// @brief Frequency to run TP2 interrupt [us]

// PID Constants
#define tau  8000

// Encoder Inputs 		(PORT A):
#define AXIS_1_ENC_A		1		// @brief Port A1
#define AXIS_1_ENC_B		2		// @brief Port A2
#define AXIS_2_ENC_A		4		// @brief Port A4
#define AXIS_2_ENC_B		12	// @brief Port A12

// Hall Effect Inputs	(PORT D):
#define AXIS_1_HALL			1		// @brief Port D1
#define AXIS_2_HALL			2		// @brief Port D2

// Motor Outputs			(PORT B):
#define AXIS_1_MTR_A		2 		// @brief Port B2
#define AXIS_1_MTR_B		1 		// @brief Port B1
#define AXIS_2_MTR_A		3 		// @brief Port B3
#define AXIS_2_MTR_B		8 		// @brief Port B8

// Motor Speeds				(PORT C):
#define AXIS_1_MTR_PWM	2	 		// @brief TPM 0.1
#define AXIS_2_MTR_PWM	8 		// @brief TPM 0.4

// Functional Outputs
#define STATUS_LED			1 		// @brief D1
#define OPT_CONTROL			2 		// @brief TBD

// @brief Structure for storing axis logic
struct axis {
	uint32_t	Motor_A;				// Motor Input Direction Pin A
	uint32_t	Motor_B;				// Motor Input Direction Pin B
	uint32_t	hall_sense;			// Hall Effect Sensor Pin A
    // @brief Quadrature Encoder values for calculating steps
	struct		encoder {
		uint32_t	Encoder_A;			// Encoder Input Pin A
		uint32_t	Encoder_B;			// Encoder Input Pin B
		bool			old_b;					// Encoder B previous state
		bool			old_a;					// Encoder A previous state
		bool			new_a;					// Encoder A current state
		bool			new_b;					// Encoder B current state
		size_t		old_count;			// Old count from home
		size_t		new_count;			// Current count from home
		size_t		backlash;				// Encoder counts from center of home to the edge of area of effect
	} encoder;
	struct		PID {
		float		KP;								// Proportional Gain
		float		KI;								// Integral Gain
		float		KD;								// Derivative Gain
		float		p_error;					// Previous Error
		float		integrator;				// PID integrating error
		float		differentiator;		// PID differential error
		float		PID_LIM_MIN;			// Max Negative Jerk [deg/sec]
		float		PID_LIM_MAX;			// Max Jerk [deg/sec]
		float		PID_LIM_MIN_INT;	// Max Initial Negative Jerk [deg/sec]
		float		PID_LIM_MAX_INT;	// Max Initial Jerk [deg/sec]
	} PID;
	struct		PWM{
		uint32_t	PWM;						// Motor PWM Pin A
		uint8_t		TPM_CH;					// TPM CH
		uint8_t		TPM_ALT;					// TPM Alt Number
		size_t		Pulse_Per_Rev;	// Encoder counts per 360 deg
		float		deg_per_pulse;		// Deg per encoder count
		float		v_b;							// 2rd Deg poly: b
		float		v_x1;							// 2rd Deg poly: x
		float		v_x2;							// 2rd Deg poly: x^2
	} PWM;
	bool		calibrated;				// Calibrate from home point
	float		velocity;					// [Deg per second]
	float		p_velocity;				// Previous Vel [Deg per second] (For PID)
  float		target_velocity;	// Deg per second
};

struct axis dof_axes[DOF];	// @brief Array of DOF Axes
const int enc_LUT[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // @brief Lookup table for encoder step from interrupts
uint8_t PORTD_ISR_Mode = 0; // @brief Controls what the PORTD interrupt does

	
void motor_control(struct axis* inAxis, bool clockwise, bool brake, uint16_t PWM_Duty);

/* @brief	Calculates angular velocity [deg/sec] from step count traversed over a period of sample time.
 *          Sets velocity value in axis structure.
 * @param		inAxis      Axis to calculate
 * @param		sample_time  Time between old_count and new_count [us] */
void calculate_speed(size_t i, size_t sample_time) {
		dof_axes[i].p_velocity = dof_axes[i].velocity;	//Pass old velocity
    dof_axes[i].velocity = (((float)dof_axes[i].encoder.new_count - (float)(dof_axes[i].encoder.old_count)) * dof_axes[i].PWM.deg_per_pulse) / (float)sample_time;
    dof_axes[i].velocity *= 1000000;   // Convert us to s
    dof_axes[i].encoder.old_count = dof_axes[i].encoder.new_count; // NOTE: Updating count HERE. IMPORTANT!!!
}

void Delay(volatile unsigned int time_del) {
	while (time_del--) {
		;
	}
}

/* @brief	Rotate Axis to home and calculate offsets
 *			Can only home one axis at a time. While homing, home detection for other axes is halted.
 * @param	inAxis	Axis to locate home */
void home_axis(struct axis* inAxis) {
	// TODO: Use goto_motor_control_function instead
	PORTD_ISR_Mode = 0;	// Set PORTD ISR to calibration mode
	// TODO: Set PWM speed below to 20% maximum
	motor_control(inAxis, true, false, 20);
	while(!inAxis->calibrated) {
		;	// Wait until PORTD ISR sets calibration
			// TODO: Add watchdog incase calibration cannot be set
	}
	// TODO: Turn clockwise 10deg, turn counterclockwise until rising edge, calculate backlash
	motor_control(inAxis, false, false, 5);
	while(inAxis->encoder.new_count != 0) {
		;	// Wait until home is reached
			// TODO: Add watchdog
	}
	PORTD_ISR_Mode = 1;	// Set PORTD ISR to normal mode
	// Apply Brake
	PTB->PSOR = MASK(inAxis->Motor_A);
	PTB->PSOR = MASK(inAxis->Motor_A);
}
// @brief Intialize TPM2 Timer for velocity function
void initalize_TPM2() {
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	//SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // Don't need to do this twice
	TPM2->MOD = (F_TPM_CLOCK/(F_TPM_OVFLW*32))-1;
	TPM2->SC	= TPM_SC_CMOD(1) | TPM_SC_PS(5) | TPM_SC_TOIE_MASK;
	// Run with interrupt
	NVIC_SetPriority(TPM2_IRQn,3);
	NVIC_ClearPendingIRQ(TPM2_IRQn);
	NVIC_EnableIRQ(TPM2_IRQn);
}

// @brief Intialize GPIO
void initalize_GPIO() {
	SystemCoreClockUpdate();
	SysTick_Config(10000);
	struct axis a_axis = {
					.Motor_A = AXIS_1_MTR_A,
					.Motor_B = AXIS_1_MTR_B,
					.hall_sense = AXIS_1_HALL,
					.encoder = {
						.Encoder_A = AXIS_1_ENC_A, 
						.Encoder_B = AXIS_1_ENC_B,
						.old_count = 0,
						.new_count = 0,
						.backlash = 0
					},
					.PID = {
						.KP = 2,
						.KI = 0.5,
						.KD = 0.25,
						.p_error = 0,
						.integrator = 0,
						.differentiator = 0,
						.PID_LIM_MIN = -250,
						.PID_LIM_MAX = 250,
						.PID_LIM_MIN_INT = -100,
						.PID_LIM_MAX_INT = 100
					},
					.PWM = {
						.PWM = AXIS_1_MTR_PWM,
						.TPM_CH = 1,
						.TPM_ALT = 4,
						.Pulse_Per_Rev = 4267,
						.deg_per_pulse = 0.084375,
						.v_b = 3579.8,
						.v_x1 = -34.371,
						.v_x2 = 0.1704
					},
					.calibrated = false,
					.velocity = 0,
					.p_velocity = 0
				};
	struct axis b_axis = {
					.Motor_A = AXIS_2_MTR_A,
					.Motor_B = AXIS_2_MTR_B,
					.hall_sense = AXIS_2_HALL,
					.encoder = {
						.Encoder_A = AXIS_2_ENC_A, 
						.Encoder_B = AXIS_2_ENC_B,
						.old_count = 0,
						.new_count = 0,
						.backlash = 0
					},
					.PID = {
						.KP = 2,
						.KI = 0.75,
						.KD = 3,
						.p_error = 0,
						.integrator = 0,
						.differentiator = 0,
						.PID_LIM_MIN = -250,
						.PID_LIM_MAX = 250,
						.PID_LIM_MIN_INT = -100,
						.PID_LIM_MAX_INT = 100
					},
					.PWM = {
						.PWM = AXIS_2_MTR_PWM,
						.TPM_CH = 4,
						.TPM_ALT = 3,
						.Pulse_Per_Rev = 6533,
						.deg_per_pulse = 0.055103,
						.v_b = 6215.6,
						.v_x1 = 33.121,
						.v_x2 = 0.0247
					},
					.calibrated = false,
					.velocity = 0,
					.p_velocity = 0
				};
	struct axis temp_dof_axes[DOF] = {a_axis, b_axis}; 				// Declare here for clean look
	memcpy(dof_axes, temp_dof_axes, sizeof(temp_dof_axes));		// Copy to global variable (compiler should remove this from code)
	// Setup Clock PLL
	MCG->C1 |= MCG_C1_CLKS(0);  // Select PLL/FLL as clock source
	MCG->C1 |= MCG_C1_IREFS(1); // Select Inernal Reference clock
	MCG->C4 |= MCG_C4_DRST_DRS(1); // Select DCO range as Mid range
	MCG->C4 |= MCG_C4_DMX32(1);    // Select DCO frequency as 48Mhz
	// Initalize Port Clocks:
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;												// Enable clock on Port A
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;												// Enable clock on Port B
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;												// Enable clock on Port C
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;												// Enable clock on Port D
	// Initalize TPMs:
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;												// Enable clock on TPM0
	// Configure TPM clocks
	//SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);	// Set TPM input freq to 48 MHz // Alexander G. Dean is an idiot.
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM0->MOD = PWM_PERIOD-1;						// Load the counter and MOD
	TPM0->SC = TPM_SC_PS(1);						// Set TPM count direction and prescale
	//TPM0->CONF |= TPM_CONF_DBGMODE(3);	// Continue in debug mode // TODO: Is this needed?
	// Initalize Axes GPIO
	for (size_t i = 0; i < DOF; i++) {
		// All motor directions must be on PORTB
		PORTB->PCR[dof_axes[i].Motor_A] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[dof_axes[i].Motor_A] |= PORT_PCR_MUX(1);
		PORTB->PCR[dof_axes[i].Motor_B] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[dof_axes[i].Motor_B] |= PORT_PCR_MUX(1);
		// All encoders must be on PORTA
		PORTA->PCR[dof_axes[i].encoder.Encoder_A] &= ~PORT_PCR_MUX_MASK;
		PORTA->PCR[dof_axes[i].encoder.Encoder_A] |= PORT_PCR_MUX(1);
		PORTA->PCR[dof_axes[i].encoder.Encoder_A] |= PORT_PCR_IRQC(11);	// Interrupt on rising and falling edge
		PORTA->PCR[dof_axes[i].encoder.Encoder_B] &= ~PORT_PCR_MUX_MASK;
		PORTA->PCR[dof_axes[i].encoder.Encoder_B] |= PORT_PCR_MUX(1);
		PORTA->PCR[dof_axes[i].encoder.Encoder_B] |= PORT_PCR_IRQC(11);	// Interrupt on rising and falling edge
		// All hall-effect sensors must be on PORTD
		PORTD->PCR[dof_axes[i].hall_sense] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[dof_axes[i].hall_sense] |= PORT_PCR_MUX(1);
		PORTD->PCR[dof_axes[i].hall_sense] |= PORT_PCR_IRQC(0x0B);	// Interrupt on rising and falling edge
		PORTD->PCR[STATUS_LED] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[STATUS_LED] |= PORT_PCR_MUX(1);
		// All PWM must be on PORTC
		// L298N H bridge accepts up to 40 kHz PWM frequency
		PORTC->PCR[dof_axes[i].PWM.PWM] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[dof_axes[i].PWM.PWM] |= PORT_PCR_MUX(dof_axes[i].PWM.TPM_ALT); // Alt# changes between pins
		// Set Outputs
		PTB->PDDR |= MASK(dof_axes[i].Motor_A) | MASK(dof_axes[i].Motor_B);
		// Set Inputs
		PTA->PDDR &= ~(MASK(dof_axes[i].encoder.Encoder_A) | MASK(dof_axes[i].encoder.Encoder_B));
		PTD->PDDR &= ~MASK(dof_axes[i].hall_sense);
		PTD->PDDR |= MASK(STATUS_LED);
		// Apply brakes on startup
		PTB->PSOR = MASK(dof_axes[i].Motor_A);
		PTB->PSOR = MASK(dof_axes[i].Motor_B);
		// Grab current encoder value
		dof_axes[i].encoder.old_a = PTA->PDIR & MASK(dof_axes[i].encoder.Encoder_A);
		dof_axes[i].encoder.old_b = PTA->PDIR & MASK(dof_axes[i].encoder.Encoder_B);
		// Configure TPM channels
		//TPM0->CONTROLS[dof_axes[i].TPM_CH].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;	// Set CH of TPM0 to edge-aligned low-true PWM
		TPM0->CONTROLS[dof_axes[i].PWM.TPM_CH].CnSC = (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(0) | TPM_CnSC_MSB(1)  | TPM_CnSC_MSA(0));
		TPM0->CONTROLS[dof_axes[i].PWM.TPM_CH].CnV = 0;					// Set initial speed to 0;
		TPM0->SC |= TPM_SC_CMOD(dof_axes[i].PWM.TPM_CH);					// Start TPM0
	}
	// Enable interrupts for encoders
	NVIC_SetPriority(PORTA_IRQn, 2);
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
	// Enable interrupts for hall effect sensor
	NVIC_SetPriority(PORTD_IRQn, 3);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	//NVIC_EnableIRQ(PORTD_IRQn);
	PTD->PSOR = MASK(STATUS_LED);
}

void motor_control(struct axis* inAxis, bool clockwise, bool brake, uint16_t PWM_Duty) {
	// Short motor windings for hard brake
	if (brake) {
		PTB->PSOR = MASK(inAxis->Motor_A);
		PTB->PSOR = MASK(inAxis->Motor_B);		
		TPM0->CONTROLS[inAxis->PWM.TPM_CH].CnV = 0;
	} else {
		// Output Direction
		// TODO: Check if input direction is different from currently set. Add backlash offset if true.
		if (clockwise) {
			PTB->PCOR = MASK(inAxis->Motor_B);	// Clear first to prevent brake if interrupt occurs after
			PTB->PSOR = MASK(inAxis->Motor_A);
		} else {
			PTB->PCOR = MASK(inAxis->Motor_A);
			PTB->PSOR = MASK(inAxis->Motor_B);
		}
		TPM0->CONTROLS[inAxis->PWM.TPM_CH].CnV = PWM_Duty;
	}
}

// @brief PORTA Interrupt Handler (Encoder Inputs)
void PORTA_IRQHandler(void) {
	//DEBUG_PORT->PSOR = MASK(DBG_ISR_POS); // Is this needed?
	PTD->PCOR = MASK(STATUS_LED);
	size_t i = 0;
	// Loop through axes and set new input
	for ( ; i < DOF; i++) {
		// Check A of encoder
		if (PORTA->ISFR & MASK(dof_axes[i].encoder.Encoder_A)) {
			dof_axes[i].encoder.new_a = PTA->PDIR & MASK(dof_axes[i].encoder.Encoder_A);
			break;	// Exit, no need to iterate through rest of axes.
		}
		// Check B of encoder
		if (PORTA->ISFR & MASK(dof_axes[i].encoder.Encoder_B)) {
			dof_axes[i].encoder.new_b = PTA->PDIR & MASK(dof_axes[i].encoder.Encoder_B);
			break;
		}
	}
	// Calculate count of interrupt axis
	int old_sum = (dof_axes[i].encoder.old_a << 1) + dof_axes[i].encoder.old_b;
	int new_sum = (dof_axes[i].encoder.new_a << 1) + dof_axes[i].encoder.new_b;
	dof_axes[i].encoder.new_count +=  enc_LUT[old_sum*4+new_sum];
	// TODO: Check if LUT = 2 (encoder skipped a step) then reboot, rehome, or count errors.
	// Check for count overflows
	if(dof_axes[i].encoder.new_count > dof_axes[i].PWM.Pulse_Per_Rev) {
		// Axis has completed a full revolution; reset count
		dof_axes[i].encoder.new_count = 0;
	}
	if(dof_axes[i].encoder.new_count == ((size_t)-1)) {
		// Axis has turned past 0, rest count to max-1
		dof_axes[i].encoder.new_count = dof_axes[i].PWM.Pulse_Per_Rev;
	}
	// Update old values for next interrupt
	dof_axes[i].encoder.old_a = dof_axes[i].encoder.new_a;
	dof_axes[i].encoder.old_b = dof_axes[i].encoder.new_b;
	// clear status flags
	PORTA->ISFR = 0xffffffff;
	//DEBUG_PORT->PCOR = MASK(DBG_ISR_POS); // Need this?
	PTD->PSOR = MASK(STATUS_LED);		
}

// @brief PORTD Interrupt Handler (Hall Effect Sensors)
void PORTD_IRQHandler(void) {
	if (PORTD_ISR_Mode == 0) {
		// Uncalibrated mode
		bool hit_rising_edge = false;
		// Loop through axes and set new input
		size_t i = 0;
		for ( ; i < DOF; i++) {
			if (PORTD->ISFR & MASK(dof_axes[i].hall_sense)) {
				break;	// Exit, no need to iterate through rest of axes.
			}
		}
		// Check for rising edge case
		if (PTD->PDIR & MASK(dof_axes[i].hall_sense)) {
			dof_axes[i].encoder.new_count = 0;
			hit_rising_edge = true;
			return;
		}
		// Check for falling edge case after traveling through home
		if (!(PTD->PDIR & MASK(dof_axes[i].hall_sense) && hit_rising_edge)) {
			dof_axes[i].encoder.new_count /= 2;		// Divide by 2 to grab center of area of effect
			dof_axes[i].calibrated = true;
			return;
		}
		return; // Return to catch rising edge case without traveling through home
	}
	if (PORTD_ISR_Mode == 1) {
		// Normal calibrated operation
	}
	PORTD ->ISFR = 0xffffffff;
}


void speed_PID(size_t i);
// @brief TPM2 Interrupt Handler for measuring velocity
void TPM2_IRQHandler() {
	//`	PTD->PCOR = MASK(STATUS_LED);
	TPM2->SC |= TPM_SC_TOIE_MASK; // Reset overflow flag
	// Calculate vel for all axes
	for (size_t i = 0; i < DOF; i++) {
		//calculate_speed(i, TPM2_PERIOD);
		speed_PID(i);
	}
	//PTD->PSOR = MASK(STATUS_LED);
}

void speed_PID(size_t i) {
		// Thank you pms67: https://github.com/pms67/PID
    calculate_speed(i, TPM2_PERIOD);
    float error = dof_axes[i].target_velocity - dof_axes[i].velocity;
		// P
			float proportional = dof_axes[i].PID.KP * error;
		// I
			dof_axes[i].PID.integrator = dof_axes[i].PID.integrator + 0.5f * dof_axes[i].PID.KI * TPM2_PERIOD * (dof_axes[i].PID.p_error + error);
			// Compute I Limits (Limit the "Limit" on distance away from P
				/*
						float lowILimInt, maxILimInt;
						if (lowILim < proportional) {
							lowILimInt = lowILim - proportional;
						} else {
							lowILimInt = 0.0f;
						}
						if (maxILim > proportional) {
							maxILimInt = maxILim - proportional;
						} else {
							maxILimInt = 0.0f;
						}
			*/
			// Clamp Integrator
			if(dof_axes[i].PID.integrator < dof_axes[i].PID.PID_LIM_MIN_INT) {
				dof_axes[i].PID.integrator = dof_axes[i].PID.PID_LIM_MIN_INT;
			} else if (dof_axes[i].PID.integrator > dof_axes[i].PID.PID_LIM_MAX_INT) {
				dof_axes[i].PID.integrator = dof_axes[i].PID.PID_LIM_MAX_INT;
			}
		// D
			dof_axes[i].PID.differentiator =	-(2.0f * dof_axes[i].PID.KD * (dof_axes[i].velocity - dof_axes[i].p_velocity)
																				+ (2.0f * tau - TPM2_PERIOD) * dof_axes[i].PID.differentiator)
																				/ (2.0f * tau + TPM2_PERIOD);
			
		// P,I,D
			float out = proportional + dof_axes[i].PID.integrator + dof_axes[i].PID.differentiator;
			// Check sign and turn positive if neg. Rotate anti-clkwise if neg.
			bool clk; if(out > 0) { clk = true; } else { clk = false; out *= -1;}
		// Convert [deg/s] => PWM
			size_t PWM_temp = ((out*out)*dof_axes[i].PWM.v_x2) + (out*dof_axes[i].PWM.v_x1) + dof_axes[i].PWM.v_b;
    // Upper Clamp PWM
			 if (PWM_temp > PWM_PERIOD) PWM_temp = PWM_PERIOD;
		// Store
			dof_axes[i].PID.p_error = error;
    // Execute motor control function
			motor_control(&dof_axes[i],clk,false,PWM_temp);
}

int main (void) {
	initalize_GPIO();
	initalize_TPM2();
	// Optional homing function
	//for (size_t i = 0; i < DOF; i++) {
	//	home_axis(&dof_axes[i]);
	//}
	volatile int32_t delay;
	for (;;) {
			float i=0;
			for ( ; i < 594; i+=0.075) {
				dof_axes[0].target_velocity = i;
				dof_axes[1].target_velocity = i;
				Delay(500);
			}
			for ( ; i > 0; i-=0.075) {
				dof_axes[0].target_velocity = i;
				dof_axes[1].target_velocity = i;
				Delay(500);
			}
				Delay(500);
			i=0;
			for ( ; i > -594; i-=0.075) {
				dof_axes[0].target_velocity = i;
				dof_axes[1].target_velocity = i;
				Delay(500);
			}
			for ( ; i < 0; i+=0.075) {
				dof_axes[0].target_velocity = i;
				dof_axes[1].target_velocity = i;
				Delay(500);
			}
				Delay(500);
	}
}
