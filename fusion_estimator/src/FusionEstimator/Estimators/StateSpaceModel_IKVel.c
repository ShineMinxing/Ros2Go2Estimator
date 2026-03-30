// StateSpaceModel_IKVel.c
// CKF(Estimator1003) for IK velocity estimation (hip-foot relative)
// Transition/Observation follow the old CKF6d usage (SensorHip::Hip_CKF6dFunTransition / Observation)
// IMPORTANT: kinematic params are READ from estimator->Double_Par, NOT written here.

#include "EstimatorPortN.h"
#include "Estimator1003_CubatureKalmanEstimator.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Double_Par usage (external fills):
// Double_Par[0] = SideSign (+1 left / -1 right)
// Double_Par[1] = HIP
// Double_Par[2] = THIGH
// Double_Par[3] = CALF
// Double_Par[4] = FOOT

EstimatorPortN StateSpaceModel_IKVel_;

// ---------------------------------------------------------
// State (Nx = 6):
//   x = [x, y, z, vx, vy, vz]^T   (hip->foot relative position & velocity)
//
// Observation (Nz = 6):
//   z = [theta1, theta2, theta3, dtheta1, dtheta2, dtheta3]^T
// ---------------------------------------------------------

// -----------------------------
// Transition (copy your CKF6d transition logic)
// Output = Input
// pos += dt * vel
// y = SideSign * abs(y)
// -----------------------------
void StateSpaceModel_IKVel_StateTransitionFunction(double *In_State,
                                                   double *Out_State,
                                                   struct EstimatorPortN *estimator)
{
    double SideSign = estimator->Double_Par[0];
    if (!(SideSign == 1.0 || SideSign == -1.0))
        SideSign = 1.0;

    double dt = estimator->CurrentTimestamp - estimator->StateUpdateTimestamp;

    // same spirit as your CKF6d: if dt abnormal, kill dt
    if (!(fabs(dt) < 0.02))
        dt = 0.0;

    // Output = Input
    for (int i = 0; i < estimator->Nx; i++)
        Out_State[i] = In_State[i];

    // integrate
    Out_State[0] = Out_State[0] + dt * Out_State[3];
    Out_State[1] = Out_State[1] + dt * Out_State[4];
    Out_State[2] = Out_State[2] + dt * Out_State[5];

    // enforce side sign on y
    Out_State[1] = SideSign * fabs(Out_State[1]);
}

// -----------------------------
// Observation (copy your CKF6d observation logic)
// from (x,y,z,vx,vy,vz) -> (theta1,theta2,theta3, Vtheta1,Vtheta2,Vtheta3)
// -----------------------------
void StateSpaceModel_IKVel_ObservationFunction(double *In_State,
                                               double *Out_Observation,
                                               EstimatorPortN *estimator)
{
    double SideSign = estimator->Double_Par[0];
    double Lhip     = estimator->Double_Par[1];
    double Lthigh   = estimator->Double_Par[2];
    double Lcalf    = estimator->Double_Par[3];
    double Lfoot    = estimator->Double_Par[4];

    if (!(SideSign == 1.0 || SideSign == -1.0))
        SideSign = 1.0;

    const double L2 = (Lcalf + Lfoot);

    double x  = In_State[0];
    double y  = In_State[1];
    double z  = In_State[2];
    double vx = In_State[3];
    double vy = In_State[4];
    double vz = In_State[5];

    double theta1, theta2, theta3;
    double Vtheta1, Vtheta2, Vtheta3;

    // ---- theta1 (same branching as your code) ----
    double y_use = y;

    if (!((SideSign == 1.0 && y_use > SideSign * Lhip) ||
          (SideSign == -1.0 && y_use < SideSign * Lhip)))
    {
        if (!((SideSign == 1.0 && y_use > 0.0) ||
              (SideSign == -1.0 && y_use < 0.0)))
        {
            y_use = 0.0;
        }
    }

    double denom = 2.0 * (z * z + y_use * y_use);

    double disc = 0.000001
                + 4.0 * Lhip * Lhip * z * z
                - 4.0 * (z * z + y_use * y_use) * (Lhip * Lhip - y_use * y_use);

    double numer = 2.0 * Lhip * z + sqrt(disc);
    theta1 = SideSign * asin(numer / denom);

    // ---- theta2 / theta3 ----
    double z_bar = z - SideSign * sin(theta1) * Lhip;
    double y_bar = y_use - SideSign * cos(theta1) * Lhip;

    double r_bar = sqrt(z_bar * z_bar + y_bar * y_bar);
    double r = sqrt(r_bar * r_bar + x * x);

    theta3 = -M_PI + acos((Lthigh * Lthigh + L2 * L2 - r * r) / (2.0 * Lthigh * L2));

    double alpha = atan2(x, r_bar);
    double beta  = acos((r * r + Lthigh * Lthigh - L2 * L2) / (2.0 * r * Lthigh));
    theta2 = alpha + beta;

    // ---- Jacobian J (same formulas as your code) ----
    double s1 = sin(theta1), c1 = cos(theta1);
    double s2 = sin(theta2), c2 = cos(theta2);
    double s3 = sin(theta3), c3 = cos(theta3);

    double c23 = c2 * c3 - s2 * s3; // cos(theta2+theta3)
    double s23 = s2 * c3 + c2 * s3; // sin(theta2+theta3)

    double J[3][3];

    J[0][0] = 0.0;
    J[0][1] = L2 * c23 + Lthigh * c2;
    J[0][2] = L2 * c23;

    J[1][0] = -Lhip * SideSign * s1 + L2 * c1 * c23 + Lthigh * c2 * c1;
    J[1][1] = L2 * s1 * (-s23) + Lthigh * s1 * (-s2);
    J[1][2] = L2 * s1 * (-s23);

    J[2][0] =  Lhip * SideSign * c1 + L2 * s1 * c23 + Lthigh * c2 * s1;
    J[2][1] = -L2 * c1 * (-s23) - Lthigh * c1 * (-s2);
    J[2][2] = -L2 * c1 * (-s23);

    double detJ =
        J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
        J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
        J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);

    double invJ[3][3] = {
        {(J[1][1] * J[2][2] - J[1][2] * J[2][1]) / detJ,
         -(J[0][1] * J[2][2] - J[0][2] * J[2][1]) / detJ,
         (J[0][1] * J[1][2] - J[0][2] * J[1][1]) / detJ},

        {-(J[1][0] * J[2][2] - J[1][2] * J[2][0]) / detJ,
         (J[0][0] * J[2][2] - J[0][2] * J[2][0]) / detJ,
         -(J[0][0] * J[1][2] - J[0][2] * J[1][0]) / detJ},

        {(J[1][0] * J[2][1] - J[1][1] * J[2][0]) / detJ,
         -(J[0][0] * J[2][1] - J[0][1] * J[2][0]) / detJ,
         (J[0][0] * J[1][1] - J[0][1] * J[1][0]) / detJ}
    };

    Vtheta1 = invJ[0][0] * vx + invJ[0][1] * vy + invJ[0][2] * vz;
    Vtheta2 = invJ[1][0] * vx + invJ[1][1] * vy + invJ[1][2] * vz;
    Vtheta3 = invJ[2][0] * vx + invJ[2][1] * vy + invJ[2][2] * vz;

    Out_Observation[0] = theta1;
    Out_Observation[1] = theta2;
    Out_Observation[2] = theta3;
    Out_Observation[3] = Vtheta1;
    Out_Observation[4] = Vtheta2;
    Out_Observation[5] = Vtheta3;
}

// Prediction = same as Transition but using estimator->PredictTime
void StateSpaceModel_IKVel_PredictionFunction(double *In_State,
                                              double *Out_PredictedState,
                                              EstimatorPortN *estimator)
{
    double SideSign = estimator->Double_Par[0];
    if (!(SideSign == 1.0 || SideSign == -1.0))
        SideSign = 1.0;

    double dt = estimator->PredictTime;

    for (int i = 0; i < estimator->Nx; i++)
        Out_PredictedState[i] = In_State[i];

    Out_PredictedState[0] = Out_PredictedState[0] + dt * Out_PredictedState[3];
    Out_PredictedState[1] = Out_PredictedState[1] + dt * Out_PredictedState[4];
    Out_PredictedState[2] = Out_PredictedState[2] + dt * Out_PredictedState[5];

    Out_PredictedState[1] = SideSign * fabs(Out_PredictedState[1]);
}

EXPORT void StateSpaceModel_IKVel_EstimatorPort(double *In_Observation,
                                               double In_Observation_Timestamp,
                                               struct EstimatorPortN *estimator)
{
    estimator->CurrentTimestamp = In_Observation_Timestamp;
    estimator->ObservationTimestamp = In_Observation_Timestamp;

    for (int i = 0; i < estimator->Nz; i++)
        estimator->CurrentObservation[i] = In_Observation[i];

    Estimator1003_Estimation(estimator);

    estimator->StateUpdateTimestamp = estimator->CurrentTimestamp;
}

EXPORT void StateSpaceModel_IKVel_EstimatorPortTermination(struct EstimatorPortN *estimator)
{
    Estimator1003_Termination();

    free(estimator->PortName);             estimator->PortName = NULL;
    free(estimator->PortIntroduction);     estimator->PortIntroduction = NULL;

    free(estimator->EstimatedState);       estimator->EstimatedState = NULL;
    free(estimator->PredictedState);       estimator->PredictedState = NULL;
    free(estimator->CurrentObservation);   estimator->CurrentObservation = NULL;
    free(estimator->PredictedObservation); estimator->PredictedObservation = NULL;

    free(estimator->Matrix_F);             estimator->Matrix_F = NULL;
    free(estimator->Matrix_G);             estimator->Matrix_G = NULL;
    free(estimator->Matrix_B);             estimator->Matrix_B = NULL;
    free(estimator->Matrix_H);             estimator->Matrix_H = NULL;

    free(estimator->Matrix_P);             estimator->Matrix_P = NULL;
    free(estimator->Matrix_Q);             estimator->Matrix_Q = NULL;
    free(estimator->Matrix_R);             estimator->Matrix_R = NULL;

    free(estimator->Int_Par);              estimator->Int_Par = NULL;
    free(estimator->Double_Par);           estimator->Double_Par = NULL;

    printf("EstimatorPort terminated.\n");
}

EXPORT void StateSpaceModel_IKVel_Initialization(EstimatorPortN *estimator)
{
    char *PortNameTemp = "IKVel CKF Estimator v0.00";
    char *PortIntroductionTemp = "CKF(1003): x=[hip->foot pos/vel], z=[joint angles/vel] (IK observation)";

    estimator->Nx = 6;
    estimator->Nz = 6;

    estimator->PredictStep = 0;
    estimator->Intervel = 0.004;
    estimator->PredictTime = 0.0;

    estimator->CurrentTimestamp = 0.0;
    estimator->StateUpdateTimestamp = 0.0;
    estimator->ObservationTimestamp = 0.0;

    estimator->PortName = (char *)malloc(100);
    estimator->PortIntroduction = (char *)malloc(1000);

    estimator->EstimatedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->PredictedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->CurrentObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->PredictedObservation = (double *)malloc(estimator->Nz * sizeof(double));

    estimator->Matrix_F = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_G = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_B = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->Matrix_H = (double *)malloc(estimator->Nz * estimator->Nx * sizeof(double));

    estimator->Matrix_P = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_Q = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_R = (double *)malloc(estimator->Nz * estimator->Nz * sizeof(double));

    estimator->Int_Par  = (int *)malloc(100 * sizeof(int));
    estimator->Double_Par = (double *)malloc(100 * sizeof(double));  // you only need [0..4]

    if (estimator->PortName == NULL || estimator->PortIntroduction == NULL ||
        estimator->EstimatedState == NULL || estimator->PredictedState == NULL ||
        estimator->CurrentObservation == NULL || estimator->PredictedObservation == NULL ||
        estimator->Matrix_F == NULL || estimator->Matrix_G == NULL ||
        estimator->Matrix_B == NULL || estimator->Matrix_H == NULL ||
        estimator->Matrix_P == NULL || estimator->Matrix_Q == NULL ||
        estimator->Matrix_R == NULL || estimator->Int_Par == NULL ||
        estimator->Double_Par == NULL)
    {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }

    strcpy(estimator->PortName, PortNameTemp);
    strcpy(estimator->PortIntroduction, PortIntroductionTemp);

    for (int i = 0; i < estimator->Nx; i++)
    {
        estimator->EstimatedState[i] = 0.0;
        estimator->PredictedState[i] = 0.0;
    }
    for (int i = 0; i < estimator->Nz; i++)
    {
        estimator->CurrentObservation[i] = 0.0;
        estimator->PredictedObservation[i] = 0.0;
    }

    // F/G/H identity (CKF uses nonlinear functions; matrices just need valid sizes)
    for (int r = 0; r < estimator->Nx; r++)
        for (int c = 0; c < estimator->Nx; c++)
            estimator->Matrix_F[r * estimator->Nx + c] = (r == c) ? 1.0 : 0.0;

    for (int r = 0; r < estimator->Nx; r++)
        for (int c = 0; c < estimator->Nx; c++)
            estimator->Matrix_G[r * estimator->Nx + c] = (r == c) ? 1.0 : 0.0;

    for (int i = 0; i < estimator->Nx; i++)
        estimator->Matrix_B[i] = 0.0;

    for (int r = 0; r < estimator->Nz; r++)
        for (int c = 0; c < estimator->Nx; c++)
            estimator->Matrix_H[r * estimator->Nx + c] = (r == c) ? 1.0 : 0.0;

    // P/Q/R default values (can be overwritten externally)
    for (int r = 0; r < estimator->Nx; r++)
        for (int c = 0; c < estimator->Nx; c++)
            estimator->Matrix_P[r * estimator->Nx + c] = (r == c) ? 1e-4 : 0.0;

    for (int r = 0; r < estimator->Nx; r++)
        for (int c = 0; c < estimator->Nx; c++)
            estimator->Matrix_Q[r * estimator->Nx + c] = 0.0;
    for (int i = 0; i < 3; i++)
        estimator->Matrix_Q[i * estimator->Nx + i] = 1e-4;
    for (int i = 3; i < 6; i++)
        estimator->Matrix_Q[i * estimator->Nx + i] = 1e-3;

    for (int r = 0; r < estimator->Nz; r++)
        for (int c = 0; c < estimator->Nz; c++)
            estimator->Matrix_R[r * estimator->Nz + c] = 0.0;
    for (int i = 0; i < 3; i++)
        estimator->Matrix_R[i * estimator->Nz + i] = 0.01;
    for (int i = 3; i < 6; i++)
        estimator->Matrix_R[i * estimator->Nz + i] = 1.0;

    for (int i = 0; i < 100; i++)
        estimator->Int_Par[i] = 0;

    // only clear the needed Double_Par slots
    for (int i = 0; i < 5; i++)
        estimator->Double_Par[i] = 0.0;

    estimator->StateTransitionEquation = StateSpaceModel_IKVel_StateTransitionFunction;
    estimator->ObservationEquation     = StateSpaceModel_IKVel_ObservationFunction;
    estimator->PredictionEquation      = StateSpaceModel_IKVel_PredictionFunction;

    printf("%s is initialized\n", estimator->PortName);

    Estimator1003_Init(estimator);
}
