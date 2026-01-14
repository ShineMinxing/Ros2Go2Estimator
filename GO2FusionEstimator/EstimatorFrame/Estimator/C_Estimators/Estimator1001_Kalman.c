#include "Estimator1001_Kalman.h"

void Estimator1001_Init(EstimatorPortN *estimator)
{
    init_stack(&S);

    // Model parameter initialization
    E1001_F  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_FT = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_G  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_H  = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1001_HT = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_Q  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_R  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_Xe = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Xp = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Z  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_Ze = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_K  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_P  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_P_pre  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);

    // Intermediate variables
    E1001_XX1 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_XX2 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_ZX1 = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1001_XZ1 = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_ZZ1 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_ZZ2 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_X11 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Z11 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_Z12 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_eyeX = creat_eye_matrix(estimator->Nx, &errorID, &S);

    for (int i = 0; i < estimator->Nx; i++)
    {
        E1001_Xe->p[i] = estimator->EstimatedState[i];
        E1001_Xp->p[i] = estimator->PredictedState[i];
    }
    for (int i = 0; i < estimator->Nx * estimator->Nx; i++)
    {
        E1001_P->p[i] = estimator->Matrix_P[i];
        E1001_F->p[i] = estimator->Matrix_F[i];
        E1001_G->p[i] = estimator->Matrix_G[i];
        E1001_Q->p[i] = estimator->Matrix_Q[i];
    }
    for (int i = 0; i < estimator->Nz * estimator->Nz; i++)
    {
        E1001_R->p[i] = estimator->Matrix_R[i];
    }
    for (int i = 0; i < estimator->Nx * estimator->Nz; i++)
    {
        E1001_H->p[i] = estimator->Matrix_H[i];
    }

    printf("Linear Kalman estimator is initialized\n");
}

void Estimator1001_Input(EstimatorPortN *estimator)
{
    for (int i = 0; i < estimator->Nz; i++)
    {
        E1001_Z->p[i] = estimator->CurrentObservation[i];
    }
    for (int i = 0; i < estimator->Nx; i++)
    {
        E1001_Xe->p[i] = estimator->EstimatedState[i];
    }
    for (int i = 0; i < estimator->Nx * estimator->Nx; i++)
    {
        E1001_P->p[i] = estimator->Matrix_P[i];
        E1001_F->p[i] = estimator->Matrix_F[i];
        E1001_G->p[i] = estimator->Matrix_G[i];
        E1001_Q->p[i] = estimator->Matrix_Q[i];
    }
    for (int i = 0; i < estimator->Nz * estimator->Nz; i++)
    {
        E1001_R->p[i] = estimator->Matrix_R[i];
    }
    for (int i = 0; i < estimator->Nx * estimator->Nz; i++)
    {
        E1001_H->p[i] = estimator->Matrix_H[i];
    }
}

void Estimator1001_Estimation(EstimatorPortN *estimator)
{
    Estimator1001_Input(estimator);

    // State prediction E1001_Xe
    matrix_multiplication(E1001_F, E1001_Xe, E1001_X11);
    matrix_multiplication(E1001_eyeX, E1001_X11, E1001_Xe);

    // Covariance prediction E1001_P_pre
    matrix_transpose(E1001_F, E1001_FT);
    matrix_multiplication(E1001_F, E1001_P, E1001_XX1);
    matrix_multiplication(E1001_XX1, E1001_FT, E1001_XX2);
    matrix_add(E1001_XX2, E1001_Q, E1001_P_pre);

    // Compute Kalman gain E1001_K
    matrix_transpose(E1001_H, E1001_HT);
    matrix_multiplication(E1001_H, E1001_P_pre, E1001_ZX1);
    matrix_multiplication(E1001_ZX1, E1001_HT, E1001_ZZ1);
    matrix_add(E1001_ZZ1, E1001_R, E1001_ZZ1);
    matrix_inverse(E1001_ZZ1, E1001_ZZ2);
    matrix_multiplication(E1001_P_pre, E1001_HT, E1001_XZ1);
    matrix_multiplication(E1001_XZ1, E1001_ZZ2, E1001_K);

    // State update E1001_Xe
    matrix_multiplication(E1001_H, E1001_Xe, E1001_Z11);
    matrix_subtraction(E1001_Z, E1001_Z11, E1001_Z12);
    matrix_multiplication(E1001_K, E1001_Z12, E1001_X11);
    matrix_add(E1001_Xe, E1001_X11, E1001_Xe);

    // Covariance update E1001_P
    matrix_multiplication(E1001_K, E1001_H, E1001_XX1);
    matrix_subtraction(E1001_eyeX, E1001_XX1, E1001_XX2);
    matrix_multiplication(E1001_XX2, E1001_P_pre, E1001_P);

    // State and observation prediction E1001_Z
    matrix_valuation(E1001_Xe, E1001_Xp, 0, 0);
    for (int i = 0; i < estimator->PredictStep; i++)
    {
        matrix_multiplication(E1001_F, E1001_Xp, E1001_X11);
        matrix_valuation(E1001_X11, E1001_Xp, 0, 0);
    }
    matrix_multiplication(E1001_H, E1001_Xp, E1001_Ze);
    
    Estimator1001_Output(estimator);
}

void Estimator1001_Output(EstimatorPortN *estimator)
{
    for (int i = 0; i < estimator->Nz; i++)
    {
        estimator->PredictedObservation[i] = E1001_Ze->p[i];
    }
    for (int i = 0; i < estimator->Nx; i++)
    {
        estimator->EstimatedState[i] = E1001_Xe->p[i];
        estimator->PredictedState[i] = E1001_Xp->p[i];
    }
    for (int i = 0; i < estimator->Nx * estimator->Nx; i++)
    {
        estimator->Matrix_P[i] = E1001_P->p[i];
    }
}

void Estimator1001_Termination()
{
    free_stack(&S);
}
