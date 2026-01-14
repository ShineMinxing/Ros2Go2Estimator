/*
% ===========================================================================
% Unscented Kalman Estimator ID 2002
% Filename: Estimator2002_UnscentedKalmanEstimator.cpp  Estimator2002_UnscentedKalmanEstimator.h
% Description:
% This estimator performs nonlinear estimation based on the structure provided in EstimatorPortN.h and the specific state-space model provided in StateSpaceModelN.c
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
#ifndef _Estimator2002_H_
#define _Estimator2002_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "../EstimatorPortN.h"

void Estimator2002_Init(EstimatorPortN *estimator);
void Estimator2002_Estimation(EstimatorPortN *estimator);
void Estimator2002_Termination();

#ifdef __cplusplus
}
#endif

#endif
