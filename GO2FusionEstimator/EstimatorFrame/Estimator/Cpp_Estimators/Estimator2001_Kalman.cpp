#include "Estimator2001_Kalman.h"
#include <iostream>
#include "Eigen/Dense"

using namespace Eigen;

static MatrixXd E2001_F;
static MatrixXd E2001_H;
static MatrixXd E2001_Q;
static MatrixXd E2001_R;
static MatrixXd E2001_P;
static MatrixXd E2001_K;

static VectorXd E2001_Xe;
static VectorXd E2001_Xp;
static VectorXd E2001_Ze;
static VectorXd E2001_Z;

void Estimator2001_Init(EstimatorPortN *estimator)
{
    std::cout << "Linear Kalman estimator is initialized" << std::endl;
}

void Estimator2001_Estimation(EstimatorPortN *estimator)
{
    // Map input data to Eigen objects
    E2001_Z = Map<Matrix<double, Dynamic, 1>>(estimator->CurrentObservation, estimator->Nz);
    E2001_Xe = Map<Matrix<double, Dynamic, 1>>(estimator->EstimatedState, estimator->Nx);
    E2001_P = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_P, estimator->Nx, estimator->Nx);
    E2001_F = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_F, estimator->Nx, estimator->Nx);
    E2001_Q = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_Q, estimator->Nx, estimator->Nx);
    E2001_H = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_H, estimator->Nz, estimator->Nx);
    E2001_R = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_R, estimator->Nz, estimator->Nz);


    // State prediction
    E2001_Xe = E2001_F * E2001_Xe;

    // Covariance prediction
    E2001_P = E2001_F * E2001_P * E2001_F.transpose() + E2001_Q;

    // Calculate Kalman gain
    MatrixXd S = E2001_H * E2001_P * E2001_H.transpose() + E2001_R;
    E2001_K = E2001_P * E2001_H.transpose() * S.inverse();

    // State update
    VectorXd y = E2001_Z - E2001_H * E2001_Xe;
    E2001_Xe = E2001_Xe + E2001_K * y;

    // Update covariance matrix
    E2001_P = (MatrixXd::Identity(estimator->Nx, estimator->Nx) - E2001_K * E2001_H) * E2001_P;

    // State prediction (multi-step prediction)
    E2001_Xp = E2001_Xe;
    for (int i = 0; i < estimator->PredictStep; ++i)
    {
        E2001_Xp = E2001_F * E2001_Xp;
    }

    // Observation prediction
    E2001_Ze = E2001_H * E2001_Xp;

    // Output results
    Map<VectorXd>(estimator->EstimatedState, estimator->Nx) = E2001_Xe;
    Map<VectorXd>(estimator->PredictedState, estimator->Nx) = E2001_Xp;
    Map<VectorXd>(estimator->PredictedObservation, estimator->Nz) = E2001_Ze;
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_P, estimator->Nx, estimator->Nx) = E2001_P;
}

void Estimator2001_Termination()
{
    std::cout << "Estimator2001 terminated." << std::endl;
}
