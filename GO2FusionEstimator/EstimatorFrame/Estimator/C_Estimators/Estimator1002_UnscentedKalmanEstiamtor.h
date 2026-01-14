/*
% ===========================================================================
% Unscented Kalman Estimator ID 1002
% Filename: Estimator1002_UnscentedKalmanEstimator.c  Estimator1002_UnscentedKalmanEstimator.h
% Description:
% This function estimator performs nonlinear estimation based on the structure provided in EstimatorPortN.h and the specific state-space model provided in StateSpaceModelN.c
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

#ifndef _Estimator1002_H_
#define _Estimator1002_H_

#include "matrix.h"
#include "../EstimatorPortN.h"

void Estimator1002_Init(EstimatorPortN *estimator);
void Estimator1002_Input(EstimatorPortN *estimator);
void Estimator1002_Estimation(EstimatorPortN *estimator);
void Estimator1002_Output(EstimatorPortN *estimator);
void Estimator1002_Termination();

MATRIX *E1002_Q, *E1002_R, *E1002_Xe, *E1002_Xp, *E1002_Z, *E1002_Ze, *E1002_P, *E1002_P_pre, *E1002_K;
MATRIX *E1002_Alpha, *E1002_Beta, *E1002_Ki, *E1002_Lambda, *E1002_A, *E1002_Xsigma, *E1002_Xsgm0, *E1002_Xmean;
MATRIX *E1002_Wm, *E1002_Wc, *E1002_DiagWc, *E1002_X2X1, *E1002_Z2X1, *E1002_Zmean, *E1002_X1, *E1002_X2, *E1002_X2T, *E1002_Z1, *E1002_Z2, *E1002_Z2T, *E1002_Pxz, *E1002_PxzT, *E1002_Pzz;
MATRIX *E1002_X11, *E1002_X12, *E1002_Z11, *E1002_Z12, *E1002_ZZ1, *E1002_XX1;

#endif
