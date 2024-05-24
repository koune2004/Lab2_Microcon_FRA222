/*
 * Kalman_lib.c
 *
 *  Created on: May 22, 2024
 *      Author: WARITT
 */
#include "Kalman_lib.h"

// INITIALIZE KALMAN STRUCTURE FUNCTION
void Kalman_Init(Kalman* Kal, float32_t* A_val, float32_t* B_val, float32_t* C_val,
		float32_t* D_val, float32_t* G_val, float32_t* Q_val, float32_t* R_val,
		float32_t* P_val, float32_t* K_val, float32_t* S_val,float32_t* x_val,
		float32_t* temp1_val, float32_t* temp2_val, float32_t* temp3_val,
		float32_t* temp4_val,float32_t* temp5_val,float32_t* temp6_val){

	// MAIN KALMAN STRUCTURE
	arm_mat_init_f32(&Kal->A, 3, 3, A_val); //INITIALIZE STATE TRANSITTION MATRIX [3X3]
	arm_mat_init_f32(&Kal->B, 3, 1, B_val); //INITIALIZE CONTROL INPUT MATRIX [3X1]
	arm_mat_init_f32(&Kal->C, 1, 3, C_val); //INITIALIZE OBSERVATION MATRIX [1X3]
	arm_mat_init_f32(&Kal->D, 1, 1, D_val); //INITIALIZE CONTROL INPUT MATRIX TO MEASUREMENT [1X1]
	arm_mat_init_f32(&Kal->G, 3, 1, G_val); //INITIALIZE PROCESS NOISE GAIN MATRIX [3X1]
	arm_mat_init_f32(&Kal->Q, 1, 1, Q_val); //INITIALIZE PROCCESS NOISE COVARIANCE [1X1]
	arm_mat_init_f32(&Kal->R, 1, 1, R_val); //INITIALIZE MEASUREMENT NOISE COVARIANCE [1X1]
	arm_mat_init_f32(&Kal->P, 3, 3, P_val); //INITIALIZE PREDICTION ERROR COVARIANCE [3X3]
	arm_mat_init_f32(&Kal->K, 3, 1, K_val); //INITIALIZE KALMAN GAIN [3X1]
	arm_mat_init_f32(&Kal->S, 1, 1, S_val); //INITIALIZE INNOVATION COVARIANCE [1X1]
	arm_mat_init_f32(&Kal->x, 3, 1, x_val); //INITIALIZE STATE ESTIMSATE [3X1]

	// BUFFER MATRIX KALMAN STRUCTURE
	arm_mat_init_f32(&Kal->temp1, 3, 1, temp1_val); //INITIALIZE BUFFER MATRIX [3X1]
	arm_mat_init_f32(&Kal->temp2, 3, 1, temp2_val); //INITIALIZE BUFFER MATRIX [3X1]
	arm_mat_init_f32(&Kal->temp3, 3, 3, temp3_val); //INITIALIZE BUFFER MATRIX [3X3]
	arm_mat_init_f32(&Kal->temp4, 3, 3, temp4_val); //INITIALIZE BUFFER MATRIX [3X3]
	arm_mat_init_f32(&Kal->temp5, 1, 3, temp5_val); //INITIALIZE BUFFER MATRIX [1X3]
	arm_mat_init_f32(&Kal->temp6, 1, 1, temp6_val); //INITIALIZE BUFFER MATRIX [1X1]
}

// PREDICTION STATE FUNCTION
void Kalman_Predict(Kalman* Kal, float32_t* u_val, float32_t* w_val){
	arm_mat_init_f32(&Kal->w, 1, 1, w_val); //INITIALIZE MEASUREMENT VALUE [1X1]
	arm_mat_init_f32(&Kal->u, 1, 1, u_val); //INITIALIZE STATE ESTIMSATE [3X1]

	// PREDICTED STATE ESTIMATE
	// x(k) = A*x(k-1) + B*u(k-1) +G*w
	arm_mat_mult_f32(&Kal->A, &Kal->x, &Kal->temp1);	// OUT PUT MATRIZ [3x1]
	arm_mat_mult_f32(&Kal->B, &Kal->u, &Kal->temp2);	// OUT PUT MATRIZ [3x1]
	arm_mat_add_f32(&Kal->temp1, &Kal->temp2, &Kal->x);	// OUT PUT MATRIZ [3x1]
	arm_mat_mult_f32(&Kal->G, &Kal->w, &Kal->temp1);	// OUT PUT MATRIZ [3x1]
	arm_mat_add_f32(&Kal->x, &Kal->temp1, &Kal->x);		// OUT PUT MATRIZ [3x1]

	// PREDICTED ERROR COVARIANCE
	// P(k) = A*P(k-1)*A_t + G*Q*G_t
	arm_mat_mult_f32(&Kal->A, &Kal->P, &Kal->temp3);		// OUT PUT MATRIZ [3x3]
	arm_mat_trans_f32(&Kal->A, &Kal->temp4);				// OUT PUT MATRIZ [3x3]
	arm_mat_mult_f32(&Kal->temp3, &Kal->temp4, &Kal->P);	// OUT PUT MATRIZ [3x3]
	arm_mat_mult_f32(&Kal->G, &Kal->Q, &Kal->temp1);		// OUT PUT MATRIZ [3x1]
	arm_mat_trans_f32(&Kal->G, &Kal->temp5);				// OUT PUT MATRIZ [1x3]
	arm_mat_mult_f32(&Kal->temp1, &Kal->temp5, &Kal->temp2);// OUT PUT MATRIZ [3x1]
	arm_mat_add_f32(&Kal->P, &Kal->temp2, &Kal->P);			// OUT PUT MATRIZ [3x3]
}

// UPDATE STATE FUNCTION
void Kalman_Update(Kalman* Kal, float32_t* u_val, float32_t* y_val){
	arm_mat_init_f32(&Kal->u, 1, 1, u_val); //INITIALIZE STATE ESTIMSATE [3X1]
	arm_mat_init_f32(&Kal->y, 1, 1, y_val); //INITIALIZE MEASUREMENT VALUE [1X1]

	// INNOVATION RESIDUAL
	// y(k) = y(k) - C*x(k) - D*u(k)
	arm_mat_mult_f32(&Kal->C, &Kal->x, &Kal->temp6);	// OUT PUT MATRIZ [1x1]
	arm_mat_sub_f32(&Kal->y, &Kal->temp6, &Kal->y);		// OUT PUT MATRIZ [1x1]
	arm_mat_mult_f32(&Kal->D, &Kal->u, &Kal->temp6);	// OUT PUT MATRIZ [1x1]
	arm_mat_sub_f32(&Kal->y, &Kal->temp6, &Kal->y);		// OUT PUT MATRIZ [1x1]

	// INNOVATION COVARIANCE
	// S(k) = C*P(k)*C_t + R
	arm_mat_mult_f32(&Kal->C, &Kal->P, &Kal->temp5);	// OUT PUT MATRIZ [1x3]
	arm_mat_trans_f32(&Kal->C, &Kal->temp1);			// OUT PUT MATRIZ [3x1]
	arm_mat_mult_f32(&Kal->temp5, &Kal->temp1, &Kal->S);// OUT PUT MATRIZ [1x1]
	arm_mat_sub_f32(&Kal->S, &Kal->R, &Kal->S);			// OUT PUT MATRIZ [1x1]

	// OPTIMAL KALMAN GAIN
	// K(k) = P(k)*C_t*S(k)^-1
	arm_mat_mult_f32(&Kal->P, &Kal->temp1, &Kal->temp2);// OUT PUT MATRIZ [3x1]
	arm_mat_inverse_f32(&Kal->S, &Kal->temp6);			// OUT PUT MATRIZ [1x1]
	arm_mat_mult_f32(&Kal->temp2, &Kal->temp6, &Kal->K);// OUT PUT MATRIZ [3x1]

	// CORRECTED STATE ESTIMATE
	// x(k) = x(k) + K(k)*y(k)
	arm_mat_mult_f32(&Kal->K, &Kal->y, &Kal->temp1);// OUT PUT MATRIZ [3x1]
	arm_mat_add_f32(&Kal->x, &Kal->temp1, &Kal->x);	// OUT PUT MATRIZ [3x3]

	// CORRECTED ESTIMATE COVARIANCE
	// P(k) = (I_3x3 - K(k)*C)*P(k)

	// INITIALIZE IDENTITY MATRIX I 3x3
	arm_matrix_instance_f32 I;
	float32_t I_3x3[9] = {	1, 0, 0,
							0, 1, 0,
							0, 0, 1	};
	arm_mat_init_f32(&I, 3, 3, I_3x3); //INITIALIZE IDENTITY MATRIX I [3x3]
	// CALCULATE
	arm_mat_mult_f32(&Kal->K, &Kal->C, &Kal->temp3);// OUT PUT MATRIZ [3x3]
	arm_mat_sub_f32(&I, &Kal->temp3, &Kal->temp4);	// OUT PUT MATRIZ [3x3]
	arm_mat_mult_f32(&Kal->temp4, &Kal->P, &Kal->P);// OUT PUT MATRIZ [3x3]
}
