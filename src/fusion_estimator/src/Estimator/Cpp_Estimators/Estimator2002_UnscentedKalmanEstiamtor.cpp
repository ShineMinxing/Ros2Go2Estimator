#include "Estimator2002_UnscentedKalmanEstiamtor.h"

#include <iostream>
#include "Eigen/Dense"

using namespace Eigen;

static double E2002_Alpha;
static double E2002_Beta;
static double E2002_Ki;
static double E2002_Lambda;

static VectorXd E2002_Wm;
static VectorXd E2002_Wc;
static MatrixXd E2002_DiagWc;

static VectorXd E2002_Xe;
static VectorXd E2002_Xp;
static VectorXd E2002_Z;
static VectorXd E2002_Ze;
static VectorXd E2002_Xmean;
static VectorXd E2002_Zmean;

static MatrixXd E2002_P;
static MatrixXd E2002_P_pre;
static MatrixXd E2002_Q;
static MatrixXd E2002_R;
static MatrixXd E2002_K;
static MatrixXd E2002_A;

static MatrixXd E2002_Xsigma;
static MatrixXd E2002_X1;
static MatrixXd E2002_X2;
static MatrixXd E2002_Z1;
static MatrixXd E2002_Z2;
static MatrixXd E2002_Pxz;
static MatrixXd E2002_Pzz;

void Estimator2002_Init(EstimatorPortN *estimator)
{
    int Nx = estimator->Nx;
    int Nz = estimator->Nz;
    int n_sigma = 2 * Nx + 1;

    // Initialize UKF parameters
    E2002_Alpha = 0.01;
    E2002_Beta = 2.0;
    E2002_Ki = 1.0;
    E2002_Lambda = E2002_Alpha * E2002_Alpha * (Nx + E2002_Ki) - Nx;

    // Initialize weights
    E2002_Wm = VectorXd(n_sigma);
    E2002_Wc = VectorXd(n_sigma);

    E2002_Wm(0) = E2002_Lambda / (Nx + E2002_Lambda);
    E2002_Wc(0) = E2002_Wm(0) + (1 - E2002_Alpha * E2002_Alpha + E2002_Beta);

    for (int i = 1; i < n_sigma; ++i) {
        E2002_Wm(i) = 0.5 / (Nx + E2002_Lambda);
        E2002_Wc(i) = E2002_Wm(i);
    }

    E2002_DiagWc = E2002_Wc.asDiagonal();

    // Initialize other variables
    E2002_Xp = VectorXd::Zero(Nx);
    E2002_Z = VectorXd::Zero(Nz);
    E2002_Ze = VectorXd::Zero(Nz);
    E2002_K = MatrixXd::Zero(Nx, Nz);
    E2002_A = MatrixXd::Zero(Nx, Nx);

    E2002_Xsigma = MatrixXd(Nx, n_sigma);
    E2002_X1 = MatrixXd(Nx, n_sigma);
    E2002_X2 = MatrixXd(Nx, n_sigma);
    E2002_Z1 = MatrixXd(Nz, n_sigma);
    E2002_Z2 = MatrixXd(Nz, n_sigma);
    E2002_Xmean = VectorXd::Zero(Nx);
    E2002_Zmean = VectorXd::Zero(Nz);

    E2002_Pxz = MatrixXd::Zero(Nx, Nz);
    E2002_Pzz = MatrixXd::Zero(Nz, Nz);

    std::cout << "Unscented Kalman estimator is initialized" << std::endl;
}

void Estimator2002_Estimation(EstimatorPortN *estimator)
{
    int Nx = estimator->Nx;
    int Nz = estimator->Nz;
    int n_sigma = 2 * Nx + 1;

    // Map current estimate and covariance
    E2002_Xe = Map<Matrix<double, Dynamic, 1>>(estimator->EstimatedState, Nx);
    E2002_P  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_P, Nx, Nx);
    E2002_Q  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_Q, Nx, Nx);
    E2002_R  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_R, Nz, Nz);
    E2002_Z  = Map<Matrix<double, Dynamic, 1>>(estimator->CurrentObservation, Nz);

    // Calculate sigma points
    LLT<MatrixXd> lltOfP(E2002_P);
    if (lltOfP.info() == NumericalIssue) {
        std::cout << "Possibly non semi-positive definite matrix!" << std::endl;
    }
    E2002_A = lltOfP.matrixL();
    double scaling = sqrt(Nx + E2002_Lambda);
    E2002_A *= scaling;

    // Sigma point set
    E2002_Xsigma.col(0) = E2002_Xe;
    for (int i = 0; i < Nx; ++i) {
        E2002_Xsigma.col(i + 1) = E2002_Xe + E2002_A.col(i);
        E2002_Xsigma.col(i + Nx + 1) = E2002_Xe - E2002_A.col(i);
    }

    // Sigma points through state transition function
    for (int i = 0; i < n_sigma; ++i) {
        VectorXd x_sigma = E2002_Xsigma.col(i);
        VectorXd x1(Nx);
        estimator->StateTransitionEquation(x_sigma.data(), x1.data(), estimator);
        E2002_X1.col(i) = x1;
    }

    // Predict state mean
    E2002_Xmean = E2002_X1 * E2002_Wm;

    // Calculate state deviation
    for (int i = 0; i < n_sigma; ++i) {
        E2002_X2.col(i) = E2002_X1.col(i) - E2002_Xmean;
    }

    // Predict state covariance
    E2002_P_pre = E2002_X2 * E2002_DiagWc * E2002_X2.transpose() + E2002_Q;

    // Sigma points through observation function
    for (int i = 0; i < n_sigma; ++i) {
        VectorXd x1 = E2002_X1.col(i);
        VectorXd z1(Nz);
        estimator->ObservationEquation(x1.data(), z1.data(), estimator);
        E2002_Z1.col(i) = z1;
    }

    // Predict observation mean
    E2002_Zmean = E2002_Z1 * E2002_Wm;

    // Calculate observation deviation
    for (int i = 0; i < n_sigma; ++i) {
        E2002_Z2.col(i) = E2002_Z1.col(i) - E2002_Zmean;
    }

    // Predict observation covariance
    E2002_Pzz = E2002_Z2 * E2002_DiagWc * E2002_Z2.transpose() + E2002_R;

    // Calculate cross-covariance between state and observation
    E2002_Pxz = E2002_X2 * E2002_DiagWc * E2002_Z2.transpose();

    // Calculate Kalman gain
    E2002_K = E2002_Pxz * E2002_Pzz.inverse();

    // Update state estimate
    VectorXd z_diff = E2002_Z - E2002_Zmean;
    E2002_Xe = E2002_Xmean + E2002_K * z_diff;

    // Update covariance estimate
    E2002_P = E2002_P_pre - E2002_K * E2002_Pzz * E2002_K.transpose();

    // State prediction
    estimator->PredictionEquation(E2002_Xe.data(), E2002_Xp.data(), estimator);

    // Observation prediction
    estimator->ObservationEquation(E2002_Xp.data(), E2002_Ze.data(), estimator);

    // Output results
    Map<VectorXd>(estimator->EstimatedState, Nx) = E2002_Xe;
    Map<VectorXd>(estimator->PredictedState, Nx) = E2002_Xp;
    Map<VectorXd>(estimator->PredictedObservation, Nz) = E2002_Ze;
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(estimator->Matrix_P, Nx, Nx) = E2002_P;
}

void Estimator2002_Termination()
{
    std::cout << "Estimator2002 terminated." << std::endl;
}