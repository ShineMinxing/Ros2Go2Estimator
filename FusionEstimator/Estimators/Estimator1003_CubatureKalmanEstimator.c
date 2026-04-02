#include "Estimator1003_CubatureKalmanEstimator.h"

MATRIX *E1003_Q, *E1003_R, *E1003_Xe, *E1003_Xp, *E1003_Z, *E1003_Ze, *E1003_P, *E1003_P_pre, *E1003_K;
MATRIX *E1003_A, *E1003_Xsigma, *E1003_Xsgm0, *E1003_Xmean;
MATRIX *E1003_Wm, *E1003_Wc, *E1003_DiagWc, *E1003_X2X1, *E1003_Z2X1, *E1003_Zmean, *E1003_X1, *E1003_X2, *E1003_X2T, *E1003_Z1, *E1003_Z2, *E1003_Z2T, *E1003_Pxz, *E1003_PxzT, *E1003_Pzz;
MATRIX *E1003_X11, *E1003_X12, *E1003_Z11, *E1003_Z12, *E1003_ZZ1, *E1003_XX1;

void Estimator1003_Init(EstimatorPortN *estimator)
{
    init_stack(&S);

    int n = 2 * estimator->Nx;

    // Model parameter initialization
    E1003_Q  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1003_R  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1003_Xe = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1003_Xp = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1003_Z  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1003_Ze = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1003_P  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1003_P_pre = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1003_K  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1003_A  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);

    E1003_Wm = creat_zero_matrix(1, n, &errorID, &S);
    E1003_Wc = creat_zero_matrix(1, n, &errorID, &S);
    E1003_Xsgm0  = creat_zero_matrix(estimator->Nx, n, &errorID, &S);
    E1003_Xsigma = creat_zero_matrix(estimator->Nx, n, &errorID, &S);
    E1003_Xmean  = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1003_Zmean  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);

    E1003_X1 = creat_zero_matrix(estimator->Nx, n, &errorID, &S);
    E1003_X2 = creat_zero_matrix(estimator->Nx, n, &errorID, &S);
    E1003_X2T  = creat_zero_matrix(n, estimator->Nx, &errorID, &S);
    E1003_Z1   = creat_zero_matrix(estimator->Nz, n, &errorID, &S);
    E1003_Z2   = creat_zero_matrix(estimator->Nz, n, &errorID, &S);
    E1003_Z2T  = creat_zero_matrix(n, estimator->Nz, &errorID, &S);

    E1003_X2X1 = creat_zero_matrix(estimator->Nx, n, &errorID, &S);
    E1003_Z2X1 = creat_zero_matrix(estimator->Nz, n, &errorID, &S);
    E1003_Pxz  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1003_PxzT = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1003_Pzz  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);

    // Intermediate variables
    E1003_X11 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1003_X12 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1003_Z11 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1003_Z12 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1003_ZZ1 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1003_XX1 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);

    for(int i = 0; i < estimator->Nx; i++)
    {
        E1003_Xe->p[i] = estimator->EstimatedState[i];
        E1003_Xp->p[i] = estimator->PredictedState[i];
    }
    for(int i = 0; i < estimator->Nx*estimator->Nx; i++)
    {
        E1003_P->p[i] = estimator->Matrix_P[i];
        E1003_Q->p[i] = estimator->Matrix_Q[i];
    }
    for(int i = 0; i < estimator->Nz*estimator->Nz; i++)
    {
        E1003_R->p[i] = estimator->Matrix_R[i];
    }

    for (int i = 0; i < n; i++)
    {
        E1003_Wm->p[i] = 1.0 / (double)n;
        E1003_Wc->p[i] = E1003_Wm->p[i];
    }
    E1003_DiagWc = creat_diag_matrix(E1003_Wc->p, n, &errorID, &S);

    Cholesky_decomposition(E1003_P, E1003_P_pre);
    matrix_numbermulti(sqrt((double)estimator->Nx), E1003_P_pre, E1003_A);
    matrix_valuation(E1003_A, E1003_Xsgm0, 0, 0);
    matrix_numbermulti(-1, E1003_A, E1003_A);
    matrix_valuation(E1003_A, E1003_Xsgm0, 0, estimator->Nx);

    printf("Cubature Kalman estimator is initialized\n");
}

void Estimator1003_Input(EstimatorPortN *estimator)
{
    for(int i = 0; i < estimator->Nz; i++)
    {
        E1003_Z->p[i] = estimator->CurrentObservation[i];
    }
    for(int i = 0; i < estimator->Nx; i++)
    {
        E1003_Xe->p[i] = estimator->EstimatedState[i];
    }
    for(int i = 0; i < estimator->Nx*estimator->Nx; i++)
    {
        E1003_P->p[i] = estimator->Matrix_P[i];
        E1003_Q->p[i] = estimator->Matrix_Q[i];
    }
    for(int i = 0; i < estimator->Nz*estimator->Nz; i++)
    {
        E1003_R->p[i] = estimator->Matrix_R[i];
    }
}

void Estimator1003_Estimation(EstimatorPortN *estimator)
{
    int i, j, n = 2 * estimator->Nx;

    Estimator1003_Input(estimator);

    Cholesky_decomposition(E1003_P, E1003_P_pre);
    matrix_numbermulti(sqrt((double)estimator->Nx), E1003_P_pre, E1003_A);
    matrix_numbermulti(0, E1003_Xsgm0, E1003_Xsgm0);
    matrix_valuation(E1003_A, E1003_Xsgm0, 0, 0);
    matrix_numbermulti(-1, E1003_A, E1003_A);
    matrix_valuation(E1003_A, E1003_Xsgm0, 0, estimator->Nx);

    matrix_numbermulti(0, E1003_Xmean, E1003_Xmean);
    matrix_numbermulti(0, E1003_Zmean, E1003_Zmean);

    for (i = 0; i < n; i++)
    {
        // Get cubature point set
        for (j = 0; j < estimator->Nx; j++)
        {
            E1003_Xsigma->p[i + j * n] = E1003_Xe->p[j] + E1003_Xsgm0->p[i + j * n];
        }

        // State cubature transformation
        matrix_extraction(E1003_Xsigma, E1003_X11, 0, i);
        estimator->StateTransitionEquation(E1003_X11->p, E1003_X12->p, estimator);
        matrix_valuation(E1003_X12, E1003_X1, 0, i);

        matrix_numbermulti(E1003_Wm->p[i], E1003_X12, E1003_X11);
        matrix_add(E1003_Xmean, E1003_X11, E1003_Xmean);
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < estimator->Nx; j++)
        {
            E1003_X2->p[i + j * n] = E1003_X1->p[i + j * n] - E1003_Xmean->p[j];
        }
    }

    // State covariance
    matrix_multiplication(E1003_X2, E1003_DiagWc, E1003_X2X1);
    matrix_transpose(E1003_X2, E1003_X2T);
    matrix_multiplication(E1003_X2X1, E1003_X2T, E1003_P_pre);
    matrix_add(E1003_P_pre, E1003_Q, E1003_P_pre);

    for (i = 0; i < n; i++)
    {
        // Observation cubature transformation
        matrix_extraction(E1003_X1, E1003_X11, 0, i);
        estimator->ObservationEquation(E1003_X11->p, E1003_Z11->p, estimator);
        matrix_valuation(E1003_Z11, E1003_Z1, 0, i);

        matrix_numbermulti(E1003_Wm->p[i], E1003_Z11, E1003_Z12);
        matrix_add(E1003_Zmean, E1003_Z12, E1003_Zmean);
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < estimator->Nz; j++)
        {
            E1003_Z2->p[i + j * n] = E1003_Z1->p[i + j * n] - E1003_Zmean->p[j];
        }
    }

    // Covariance
    matrix_multiplication(E1003_Z2, E1003_DiagWc, E1003_Z2X1);
    matrix_transpose(E1003_Z2, E1003_Z2T);
    matrix_multiplication(E1003_Z2X1, E1003_Z2T, E1003_Pzz);
    matrix_add(E1003_Pzz, E1003_R, E1003_Pzz);

    matrix_multiplication(E1003_X2, E1003_DiagWc, E1003_X2X1);
    matrix_multiplication(E1003_X2X1, E1003_Z2T, E1003_Pxz);

    // Kalman gain
    matrix_inverse(E1003_Pzz, E1003_ZZ1);
    matrix_multiplication(E1003_Pxz, E1003_ZZ1, E1003_K);

    // State and covariance update
    matrix_subtraction(E1003_Z, E1003_Zmean, E1003_Z11);
    matrix_multiplication(E1003_K, E1003_Z11, E1003_X11);
    matrix_add(E1003_Xmean, E1003_X11, E1003_Xe);

    matrix_transpose(E1003_Pxz, E1003_PxzT);
    matrix_multiplication(E1003_K, E1003_PxzT, E1003_XX1);
    matrix_subtraction(E1003_P_pre, E1003_XX1, E1003_P);

    estimator->PredictionEquation(E1003_Xe->p, E1003_Xp->p, estimator);
    estimator->ObservationEquation(E1003_Xp->p, E1003_Ze->p, estimator);

    Estimator1003_Output(estimator);
}

void Estimator1003_Output(EstimatorPortN *estimator)
{
    for(int i = 0; i < estimator->Nz; i++)
    {
        estimator->PredictedObservation[i] = E1003_Ze->p[i];
    }
    for(int i = 0; i < estimator->Nx; i++)
    {
        estimator->EstimatedState[i] = E1003_Xe->p[i];
        estimator->PredictedState[i] = E1003_Xp->p[i];
    }
    for(int i = 0; i < estimator->Nx * estimator->Nx; i++)
    {
        estimator->Matrix_P[i] = E1003_P->p[i];
    }
}

void Estimator1003_Termination()
{
    free_stack(&S);
}
