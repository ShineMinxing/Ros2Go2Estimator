#include "Estimator1002_UnscentedKalmanEstiamtor.h"

void Estimator1002_Init(EstimatorPortN *estimator)
{
    init_stack(&S);

    // Model parameter initialization
    E1002_Q  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1002_R  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1002_Xe = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1002_Xp = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1002_Z  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1002_Ze = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1002_P  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1002_P_pre = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1002_K  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1002_A  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1002_Wm = creat_zero_matrix(1, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Wc = creat_zero_matrix(1, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_X1 = creat_zero_matrix(estimator->Nx, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_X2 = creat_zero_matrix(estimator->Nx, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_X2T  = creat_zero_matrix(2 * estimator->Nx + 1, estimator->Nx, &errorID, &S);
    E1002_Z1   = creat_zero_matrix(estimator->Nz, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Z2   = creat_zero_matrix(estimator->Nz, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Z2T  = creat_zero_matrix(2 * estimator->Nx + 1, estimator->Nz, &errorID, &S);
    E1002_X2X1 = creat_zero_matrix(estimator->Nx, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Z2X1 = creat_zero_matrix(estimator->Nz, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Pxz  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1002_PxzT = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1002_Pzz  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1002_Ki   = creat_zero_matrix(1, 1, &errorID, &S);
    E1002_Alpha  = creat_zero_matrix(1, 1, &errorID, &S);
    E1002_Beta   = creat_zero_matrix(1, 1, &errorID, &S);
    E1002_Xsgm0  = creat_zero_matrix(estimator->Nx, 2 * estimator->Nx + 1, &errorID, &S);
    E1002_Xmean  = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1002_Zmean  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1002_Lambda = creat_zero_matrix(1, 1, &errorID, &S);
    E1002_Xsigma = creat_zero_matrix(estimator->Nx, 2 * estimator->Nx + 1, &errorID, &S);

    // Intermediate variables
    E1002_X11 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1002_X12 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1002_Z11 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1002_Z12 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1002_ZZ1 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1002_XX1 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);

    for(int i = 0; i < estimator->Nx; i++)
    {
        E1002_Xe->p[i] = estimator->EstimatedState[i];
        E1002_Xp->p[i] = estimator->PredictedState[i];
    }
    for(int i = 0; i < estimator->Nx*estimator->Nx; i++)
    {
        E1002_P->p[i] = estimator->Matrix_P[i];
        E1002_Q->p[i] = estimator->Matrix_Q[i];
    }
    for(int i = 0; i < estimator->Nz*estimator->Nz; i++)
    {
        E1002_R->p[i] = estimator->Matrix_R[i];
    }

    E1002_Alpha->p[0] = 0.01;
    E1002_Beta->p[0] = 2.0;
    E1002_Ki->p[0] = 1.0;
    E1002_Lambda->p[0] = (E1002_Alpha->p[0] * E1002_Alpha->p[0])*(estimator->Nx + E1002_Ki->p[0]) - estimator->Nx;
    E1002_Wm->p[0] = E1002_Lambda->p[0] / (estimator->Nx + E1002_Lambda->p[0]);
    E1002_Wc->p[0] = E1002_Wm->p[0] + 1 - (E1002_Alpha->p[0] * E1002_Alpha->p[0]) + E1002_Beta->p[0];
    for (int i = 1; i < (2 * estimator->Nx + 1); i++)
    {
        E1002_Wm->p[i] = 0.5 / (estimator->Nx + E1002_Lambda->p[0]);
        E1002_Wc->p[i] = E1002_Wm->p[i];
    }
    E1002_DiagWc = creat_diag_matrix(E1002_Wc->p, 2 * estimator->Nx + 1, &errorID, &S);
    Cholesky_decomposition(E1002_P, E1002_P_pre);
    matrix_numbermulti(sqrt(estimator->Nx + E1002_Lambda->p[0]), E1002_P_pre, E1002_A);
    matrix_valuation(E1002_A, E1002_Xsgm0, 0, 1);
    matrix_numbermulti(-1, E1002_A, E1002_A);
    matrix_valuation(E1002_A, E1002_Xsgm0, 0, estimator->Nx + 1);

    printf("Unscented Kalman estimator is initialized\n");
}

void Estimator1002_Input(EstimatorPortN *estimator)
{
    for(int i = 0; i < estimator->Nz; i++)
    {
        E1002_Z->p[i] = estimator->CurrentObservation[i];
    }
    for(int i = 0; i < estimator->Nx; i++)
    {
        E1002_Xe->p[i] = estimator->EstimatedState[i];
    }
    for(int i = 0; i < estimator->Nx*estimator->Nx; i++)
    {
        E1002_P->p[i] = estimator->Matrix_P[i];
        E1002_Q->p[i] = estimator->Matrix_Q[i];
    }
    for(int i = 0; i < estimator->Nz*estimator->Nz; i++)
    {
        E1002_R->p[i] = estimator->Matrix_R[i];
    }
}

void Estimator1002_Estimation(EstimatorPortN *estimator)
{
    int i, j, n = 2 * estimator->Nx + 1;

    Estimator1002_Input(estimator);

    matrix_numbermulti(0, E1002_Xmean, E1002_Xmean);
    matrix_numbermulti(0, E1002_Zmean, E1002_Zmean);
    for (i = 0; i < n; i++)
    {
        // Get sigma point set
        for (j = 0; j < estimator->Nx; j++)
        {
            E1002_Xsigma->p[i + j * n] = E1002_Xe->p[j] + E1002_Xsgm0->p[i + j * n];
        }
        // State UT transformation
        matrix_extraction(E1002_Xsigma, E1002_X11, 0, i);
        estimator->StateTransitionEquation(E1002_X11->p, E1002_X12->p, estimator);
        matrix_valuation(E1002_X11, E1002_X1, 0, i);
        matrix_numbermulti(E1002_Wm->p[i], E1002_X11, E1002_X12);
        matrix_add(E1002_Xmean, E1002_X12, E1002_Xmean);
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < estimator->Nx; j++)
        {
            E1002_X2->p[i + j * n] = E1002_X1->p[i + j * n] - E1002_Xmean->p[j];
        }
    }

    // State covariance
    matrix_multiplication(E1002_X2, E1002_DiagWc, E1002_X2X1);
    matrix_transpose(E1002_X2, E1002_X2T);
    matrix_multiplication(E1002_X2X1, E1002_X2T, E1002_P_pre);
    matrix_add(E1002_P_pre, E1002_Q, E1002_P_pre);
    // print_matrix(E1002_P_pre,"E1002_P_pre");
    for (i = 0; i < n; i++)
    {
        // Observation UT transformation
        matrix_extraction(E1002_X1, E1002_X11, 0, i);
        estimator->ObservationEquation(E1002_X11->p, E1002_Z11->p, estimator);
        matrix_valuation(E1002_Z11, E1002_Z1, 0, i);
        matrix_numbermulti(E1002_Wm->p[i], E1002_Z11, E1002_Z12);
        matrix_add(E1002_Zmean, E1002_Z12, E1002_Zmean);
    }
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < estimator->Nz; j++)
        {
            E1002_Z2->p[i + j * n] = E1002_Z1->p[i + j * n] - E1002_Zmean->p[j];
        }
    }

    // Covariance
    matrix_multiplication(E1002_Z2, E1002_DiagWc, E1002_Z2X1);
    matrix_transpose(E1002_Z2, E1002_Z2T);
    matrix_multiplication(E1002_Z2X1, E1002_Z2T, E1002_Pzz);
    matrix_add(E1002_Pzz, E1002_R, E1002_Pzz);
    matrix_multiplication(E1002_X2, E1002_DiagWc, E1002_X2X1);
    matrix_multiplication(E1002_X2X1, E1002_Z2T, E1002_Pxz);
    // print_matrix(E1002_Pzz,"E1002_Pzz");
    // print_matrix(E1002_Pxz,"E1002_Pxz");

    // Kalman gain
    matrix_inverse(E1002_Pzz, E1002_ZZ1);
    matrix_multiplication(E1002_Pxz, E1002_ZZ1, E1002_K);
    // print_matrix(E1002_K,"E1002_K");

    // State and covariance update
    matrix_subtraction(E1002_Z, E1002_Zmean, E1002_Z11);
    matrix_multiplication(E1002_K, E1002_Z11, E1002_X11);
    matrix_add(E1002_Xmean, E1002_X11, E1002_Xe);
    if (E1002_Xe->p[0] > 2 * PI / E1002_Xe->p[1])
    {
        E1002_Xe->p[0] = E1002_Xe->p[0] - 2 * PI / E1002_Xe->p[1];
    }
    if (E1002_Xe->p[3] > 2 * PI / E1002_Xe->p[4])
    {
        E1002_Xe->p[3] = E1002_Xe->p[3] - 2 * PI / E1002_Xe->p[3];
    }
    // print_matrix(E1002_Z11,"E1002_Z11");
    // print_matrix(E1002_Xe,"E1002_Xe");

    matrix_transpose(E1002_Pxz, E1002_PxzT);
    matrix_multiplication(E1002_K, E1002_PxzT, E1002_XX1);
    matrix_subtraction(E1002_P_pre, E1002_XX1, E1002_P);
    // print_matrix(E1002_P,"E1002_P");

    estimator->PredictionEquation(E1002_Xe->p, E1002_Xp->p, estimator);
    estimator->ObservationEquation(E1002_Xp->p, E1002_Ze->p, estimator);
    
    Estimator1002_Output(estimator);
}

void Estimator1002_Output(EstimatorPortN *estimator)
{
    for(int i = 0; i < estimator->Nz; i++)
    {
        estimator->PredictedObservation[i] = E1002_Ze->p[i];
    }
    for(int i = 0; i < estimator->Nx; i++)
    {
        estimator->EstimatedState[i] = E1002_Xe->p[i];
        estimator->PredictedState[i] = E1002_Xp->p[i];
    }
    for(int i = 0; i < estimator->Nx * estimator->Nx; i++)
    {
        estimator->Matrix_P[i] = E1002_P->p[i];
    }
}

void Estimator1002_Termination()
{
    free_stack(&S);
}
