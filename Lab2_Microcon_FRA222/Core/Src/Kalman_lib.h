/*
 * Kalman_lib.h
 *
 *  Created on: May 22, 2024
 *      Author: WARITT
 */

#ifndef SRC_KALMAN_LIB_H_
#define SRC_KALMAN_LIB_H_

#include "arm_math.h"

// KALMAN STRUCTURE
typedef struct{
    arm_matrix_instance_f32 A;   //STATE TRANSITTION MATRIX
    arm_matrix_instance_f32 B;   //CONTROL INPUT MATRIX
    arm_matrix_instance_f32 C;   //OBSERVATION MATRIX
    arm_matrix_instance_f32 D;   //CONTROL INPUT MATRIX TO MEASUREMENT
    arm_matrix_instance_f32 G;   //PROCESS NOISE GAIN MATRIX
    arm_matrix_instance_f32 Q;   //PROCCESS NOISE COVARIANCE
    arm_matrix_instance_f32 R;   //MEASUREMENT NOISE COVARIANCE
    arm_matrix_instance_f32 P;   //PREDICTION ERROR COVARIANCE
    arm_matrix_instance_f32 K;   //KALMAN GAIN
    arm_matrix_instance_f32 S;   //INNOVATION COVARIANCE
    arm_matrix_instance_f32 w;   //PROCCESS NOISE
    arm_matrix_instance_f32 u;   //CONTROL INPUT
    arm_matrix_instance_f32 y;   //MEASUREMENT VALUE
    arm_matrix_instance_f32 x;   //STATE ESTIMSATE
    arm_matrix_instance_f32 temp1; //BUFFER MATRIX
    arm_matrix_instance_f32 temp2; //BUFFER MATRIX
    arm_matrix_instance_f32 temp3; //BUFFER MATRIX
    arm_matrix_instance_f32 temp4; //BUFFER MATRIX
    arm_matrix_instance_f32 temp5; //BUFFER MATRIX
    arm_matrix_instance_f32 temp6; //BUFFER MATRIX
} Kalman;

// INITIALIZE KALMAN STRUCTURE
void Kalman_Init(Kalman* Kal, float32_t* A_val, float32_t* B_val, float32_t* C_val,
		float32_t* D_val, float32_t* G_val, float32_t* Q_val, float32_t* R_val,
		float32_t* P_val, float32_t* K_val, float32_t* S_val,float32_t* x_val,
		float32_t* temp1_val, float32_t* temp2_val, float32_t* temp3_val,
		float32_t* temp4_val,float32_t* temp5_val,float32_t* temp6_val);

// PREDICTION STATE
void Kalman_Predict(Kalman* Kal, float32_t* u_val, float32_t* w_val);

// UPDATE STATE
void Kalman_Update(Kalman* Kal, float32_t* u_val, float32_t* y_val);

#endif /* SRC_KALMAN_LIB_H_ */
