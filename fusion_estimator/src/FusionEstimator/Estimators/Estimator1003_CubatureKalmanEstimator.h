/*
% ===========================================================================
% Cubature Kalman Estimator ID 1003
% Filename: Estimator1003_CubatureKalmanEstimator.c  Estimator1003_CubatureKalmanEstimator.h
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

#ifndef _Estimator1003_H_
#define _Estimator1003_H_

#include "matrix.h"
#include "EstimatorPortN.h"

void Estimator1003_Init(EstimatorPortN *estimator);
void Estimator1003_Input(EstimatorPortN *estimator);
void Estimator1003_Estimation(EstimatorPortN *estimator);
void Estimator1003_Output(EstimatorPortN *estimator);
void Estimator1003_Termination();

extern MATRIX *E1003_Q, *E1003_R, *E1003_Xe, *E1003_Xp, *E1003_Z, *E1003_Ze, *E1003_P, *E1003_P_pre, *E1003_K;
extern MATRIX *E1003_A, *E1003_Xsigma, *E1003_Xsgm0, *E1003_Xmean;
extern MATRIX *E1003_Wm, *E1003_Wc, *E1003_DiagWc, *E1003_X2X1, *E1003_Z2X1, *E1003_Zmean, *E1003_X1, *E1003_X2, *E1003_X2T, *E1003_Z1, *E1003_Z2, *E1003_Z2T, *E1003_Pxz, *E1003_PxzT, *E1003_Pzz;
extern MATRIX *E1003_X11, *E1003_X12, *E1003_Z11, *E1003_Z12, *E1003_ZZ1, *E1003_XX1;

#endif
