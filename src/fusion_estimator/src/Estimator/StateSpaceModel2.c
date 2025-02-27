// If you want to test another Estimator, substitue EstimatorXXXX_ to your estimator

#include "EstimatorPortN.h"
#include "Cpp_Estimators/Estimator2001_Kalman.h"
#include "Cpp_Estimators/Estimator2002_UnscentedKalmanEstiamtor.h"

EstimatorPortN StateSpaceModel2_;

void StateSpaceModel2_StateTransitionFunction(double *In_State, double *Out_State, struct EstimatorPortN *estimator) {

    estimator->Intervel = estimator->CurrentTimestamp - estimator->StateUpdateTimestamp;

	for (int i = 0; i < 3; i++)
	{
		Out_State[i * 3 + 0] = In_State[i * 3 + 0] + In_State[i * 3 + 1] * estimator->Intervel + In_State[i * 3 + 2] * estimator->Intervel * estimator->Intervel /2;
		Out_State[i * 3 + 1] = In_State[i * 3 + 1] + In_State[i * 3 + 2] * estimator->Intervel;
		Out_State[i * 3 + 2] = In_State[i * 3 + 2];
	}
}

void StateSpaceModel2_ObservationFunction(double *In_State, double *Out_Observation, EstimatorPortN *estimator) {
        
    Out_Observation[0] = In_State[0];
    Out_Observation[1] = In_State[3];
    Out_Observation[2] = In_State[6];
}

void StateSpaceModel2_PredictionFunction(double *In_State, double *Out_PredictedState, EstimatorPortN *estimator) {
	for (int i = 0; i < 3; i++)
	{
		Out_PredictedState[i * 3 + 0] = In_State[i * 3 + 0] + In_State[i * 3 + 1] * estimator->PredictTime + In_State[i * 3 + 2] * estimator->PredictTime * estimator->PredictTime /2;
		Out_PredictedState[i * 3 + 1] = In_State[i * 3 + 1] + In_State[i * 3 + 2] * estimator->PredictTime;
		Out_PredictedState[i * 3 + 2] = In_State[i * 3 + 2];
	}
}

EXPORT void StateSpaceModel2_EstimatorPort(double *In_Observation, double In_Observation_Timestamp, struct EstimatorPortN *estimator) {
	for (int i = 0; i < estimator->Nz; i++)
	{
		estimator->CurrentObservation[i] = In_Observation[i];
	}
    estimator->ObservationTimestamp = In_Observation_Timestamp;
    Estimator2002_Estimation(estimator);
}

EXPORT void StateSpaceModel2_EstimatorPortTermination(struct EstimatorPortN *estimator) {
    Estimator2002_Termination();

    free(estimator->PortName);
    estimator->PortName = NULL;
    free(estimator->PortIntroduction);
    estimator->PortIntroduction = NULL;
    free(estimator->EstimatedState);
    estimator->EstimatedState = NULL;
    free(estimator->PredictedState);
    estimator->PredictedState = NULL;
    free(estimator->CurrentObservation);
    estimator->CurrentObservation = NULL;
    free(estimator->PredictedObservation);
    estimator->PredictedObservation = NULL;
    free(estimator->Matrix_F);
    estimator->Matrix_F = NULL;
    free(estimator->Matrix_G);
    estimator->Matrix_G = NULL;
    free(estimator->Matrix_B);
    estimator->Matrix_B = NULL;
    free(estimator->Matrix_H);
    estimator->Matrix_H = NULL;
    free(estimator->Matrix_P);
    estimator->Matrix_P = NULL;
    free(estimator->Matrix_Q);
    estimator->Matrix_Q = NULL;
    free(estimator->Matrix_R);
    estimator->Matrix_R = NULL;
    free(estimator->Int_Par);
    estimator->Int_Par = NULL;
    free(estimator->Double_Par);
    estimator->Double_Par = NULL;
    printf("EstimatorPort terminated.\n");
}


EXPORT void StateSpaceModel2_Initialization(EstimatorPortN *estimator) 
{
    char *PortNameTemp = "Tested Estimator v0.00";
    char *PortIntroductionTemp = "For Reference";

    #define StateSpaceModel2_NX 9
    #define StateSpaceModel2_NZ 3
    #define StateSpaceModel2_PredictStep 0
    #define StateSpaceModel2_Interval 0.0015
    #define StateSpaceModel2_PredictTime 0
    #define StateSpaceModel2_ObservationTimestamp 0

    double X0[StateSpaceModel2_NX] = {\
    0,0,0, 0,0,0, 0,0,0\
    };
    double F[StateSpaceModel2_NX*StateSpaceModel2_NX] = {\
    1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, 0,0,0, 0,0,0,\
    0,1,StateSpaceModel2_Interval, 0,0,0, 0,0,0,\
    0,0,1, 0,0,0, 0,0,0,\
    0,0,0, 1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, 0,0,0, \
    0,0,0, 0,1,StateSpaceModel2_Interval, 0,0,0,\
    0,0,0, 0,0,1, 0,0,0,\
    0,0,0, 0,0,0, 1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, \
    0,0,0, 0,0,0, 0,1,StateSpaceModel2_Interval,\
    0,0,0, 0,0,0, 0,0,1,\
    };
    double G[StateSpaceModel2_NX*StateSpaceModel2_NX] = {\
    1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, 0,0,0, 0,0,0,\
    0,1,StateSpaceModel2_Interval, 0,0,0, 0,0,0,\
    0,0,1, 0,0,0, 0,0,0,\
    0,0,0, 1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, 0,0,0, \
    0,0,0, 0,1,StateSpaceModel2_Interval, 0,0,0,\
    0,0,0, 0,0,1, 0,0,0,\
    0,0,0, 0,0,0, 1,StateSpaceModel2_Interval,StateSpaceModel2_Interval*StateSpaceModel2_Interval/2, \
    0,0,0, 0,0,0, 0,1,StateSpaceModel2_Interval,\
    0,0,0, 0,0,0, 0,0,1,\
    };
    double B[StateSpaceModel2_NX] = {\
    0,0,0,0,0,0,0,0,0\
    };
    double H[StateSpaceModel2_NZ*StateSpaceModel2_NX] = {\
    0,0,0,0,0,0,0,0,0,\
    0,0,0,0,0,0,0,0,0,\
    0,0,0,0,0,0,0,0,0\
    };
    double P[StateSpaceModel2_NX*StateSpaceModel2_NX] = {\
    1,0,0,0,0,0,0,0,0,\
    0,1,0,0,0,0,0,0,0,\
    0,0,1,0,0,0,0,0,0,\
    0,0,0,1,0,0,0,0,0,\
    0,0,0,0,1,0,0,0,0,\
    0,0,0,0,0,1,0,0,0,\
    0,0,0,0,0,0,1,0,0,\
    0,0,0,0,0,0,0,1,0,\
    0,0,0,0,0,0,0,0,1,\
    };
    double Q[StateSpaceModel2_NX*StateSpaceModel2_NX] = {\
    1,0,0,0,0,0,0,0,0,\
    0,1,0,0,0,0,0,0,0,\
    0,0,1,0,0,0,0,0,0,\
    0,0,0,1,0,0,0,0,0,\
    0,0,0,0,1,0,0,0,0,\
    0,0,0,0,0,1,0,0,0,\
    0,0,0,0,0,0,1,0,0,\
    0,0,0,0,0,0,0,1,0,\
    0,0,0,0,0,0,0,0,1\
    };
    double R[StateSpaceModel2_NZ*StateSpaceModel2_NZ] = {\
    1,0,0,\
    0,1,0,\
    0,0,1\
    };
    double Int_Par[6] = {0,0,0,0,0,0};
    double Double_Par[6] = {0,0,0,0,0,0};

    estimator->Nx = StateSpaceModel2_NX;
    estimator->Nz = StateSpaceModel2_NZ;
    estimator->PredictStep = StateSpaceModel2_PredictStep;
    estimator->Intervel = StateSpaceModel2_Interval;
    estimator->PredictTime = StateSpaceModel2_PredictTime;
    estimator->ObservationTimestamp = StateSpaceModel2_ObservationTimestamp;

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
    estimator->Int_Par = (int *)malloc(100 * sizeof(int));
    estimator->Double_Par = (double *)malloc(100 * sizeof(double));
    if (estimator->PortName == NULL || estimator->PortIntroduction == NULL ||\
        estimator->EstimatedState == NULL || estimator->PredictedState == NULL ||\
        estimator->CurrentObservation == NULL || estimator->PredictedObservation == NULL ||\
        estimator->Matrix_F == NULL || estimator->Matrix_G == NULL ||\
        estimator->Matrix_B == NULL || estimator->Matrix_H == NULL ||\
        estimator->Matrix_P == NULL || estimator->Matrix_Q == NULL ||\
        estimator->Matrix_R == NULL || estimator->Int_Par == NULL ||\
        estimator->Double_Par == NULL) {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }

    strcpy(estimator->PortName, PortNameTemp);
    strcpy(estimator->PortIntroduction, PortIntroductionTemp);
    memcpy(estimator->EstimatedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->PredictedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_F, F, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_G, G, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_B, B, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_H, H, estimator->Nz * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_P, P, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_Q, Q, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_R, R, estimator->Nz * estimator->Nz * sizeof(double));
    memcpy(estimator->Int_Par, Int_Par, 1 * sizeof(int));
    memcpy(estimator->Double_Par, Double_Par, 1 * sizeof(double));

    // Initiate pointer
    estimator->StateTransitionEquation = StateSpaceModel2_StateTransitionFunction;
    estimator->ObservationEquation = StateSpaceModel2_ObservationFunction;
    estimator->PredictionEquation = StateSpaceModel2_PredictionFunction;

    printf("%s is initialized\n", estimator->PortName);

    Estimator2002_Init(estimator);
}