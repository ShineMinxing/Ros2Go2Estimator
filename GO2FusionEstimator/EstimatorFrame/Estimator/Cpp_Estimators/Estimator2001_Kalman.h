/*
% ===========================================================================
% Kalman Estimator ID 2001
% Filename: Estimator2001_Kalman.cpp  Estimator2001_Kalman.h
% Description:
% This function estimator performs linear estimation based on the structure provided in EstimatorPortN.h and the specific state-space model provided in StateSpaceModelN.c
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 16, 2024
% 
% Updates:
% Unit:
% Email:
% Date:
% ===========================================================================
*/
#ifndef _ESTIMATOR2001_KALMAN_H_
#define _ESTIMATOR2001_KALMAN_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "../EstimatorPortN.h"

void Estimator2001_Init(EstimatorPortN *estimator);
void Estimator2001_Estimation(EstimatorPortN *estimator);
void Estimator2001_Termination();

#ifdef __cplusplus
}
#endif

#endif