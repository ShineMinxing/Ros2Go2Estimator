/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <fstream>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include "EstimatorFrame/Estimator/EstimatorPortN.h"

namespace DataFusion
{
  extern double Est_Quaternion[4];
  extern double Est_QuaternionInv[4];

  class Sensors
  {
    public:

      Sensors(EstimatorPortN* StateSpaceModel_)
      {        
        StateSpaceModel = StateSpaceModel_;

        for(int i = 0; i < StateSpaceModel->Nz; i++)
          StateSpaceModel->Matrix_R[i*StateSpaceModel->Nz+i] = R_diag[i];
      }

      virtual ~Sensors() = default;  
      virtual void SensorDataHandle(double* Message, double Time) {}
      double SensorPosition[3] = {0,0,0};
      double SensorQuaternion[4]    = {1,0,0,0};
      double SensorQuaternionInv[4] = {1,0,0,0};


    protected:

      EstimatorPortN* StateSpaceModel;
      double Est_QuaternionTemp1[4], Est_QuaternionTemp2[4], Est_QuaternionTemp3[4];
      double Est_BodyAngleVel[3], Est_SensorWorldPosition[3], Est_SensorWorldVelocity[3], Est_SensorPosition[3], Est_Vector3dTemp1[3], Est_Vector3dTemp2[3];

      double Observation[9] = {0};
      double ObservationTime = 0;
      double R_diag[9] = {1,1,1,1,1,1,1,1,1};

      void UpdateEst_Quaternion();
      void ObservationCorrect_Position();
      void ObservationCorrect_Velocity();
      void ObservationCorrect_Acceleration();
      void ObservationCorrect_Orientation();
      void ObservationCorrect_AngularVelocity();
      void ObservationCorrect_AngularAcceleration();

  };

  // q = [w,x,y,z], v = [x,y,z]
  static inline void quat_conj(const double q[4], double qc[4]) {
    qc[0] =  q[0]; qc[1] = -q[1]; qc[2] = -q[2]; qc[3] = -q[3];
  }

  static inline void quat_mul(const double a[4], const double b[4], double out[4]) {
    // out = a ⊗ b
    const double aw=a[0], ax=a[1], ay=a[2], az=a[3];
    const double bw=b[0], bx=b[1], by=b[2], bz=b[3];
    out[0] = aw*bw - ax*bx - ay*by - az*bz;
    out[1] = aw*bx + ax*bw + ay*bz - az*by;
    out[2] = aw*by - ax*bz + ay*bw + az*bx;
    out[3] = aw*bz + ax*by - ay*bx + az*bw;
  }

  static inline void quat_normalize(double q[4]) {
    const double n = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n > 0) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
  }

  static inline void quat_rot_vec3(const double q_unit[4], const double v[3], double v_out[3]) {
    // Fast rotate: v' = v + 2*w*(q_vec × v) + 2*(q_vec × (q_vec × v))
    // q = [w, x, y, z], assumed normalized
    const double w = q_unit[0];
    const double x = q_unit[1];
    const double y = q_unit[2];
    const double z = q_unit[3];

    // t = 2*(q_vec × v)
    const double tx = 2.0 * (y*v[2] - z*v[1]);
    const double ty = 2.0 * (z*v[0] - x*v[2]);
    const double tz = 2.0 * (x*v[1] - y*v[0]);

    // v' = v + w*t + (q_vec × t)
    v_out[0] = v[0] + w*tx + (y*tz - z*ty);
    v_out[1] = v[1] + w*ty + (z*tx - x*tz);
    v_out[2] = v[2] + w*tz + (x*ty - y*tx);
  }

  static inline void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
  }

  static inline void eulerZYX_to_quat(double roll, double pitch, double yaw, double q_out[4]) {
    const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);

    q_out[0] = cy*cp*cr + sy*sp*sr;
    q_out[1] = cy*cp*sr - sy*sp*cr;
    q_out[2] = cy*sp*cr + sy*cp*sr;
    q_out[3] = sy*cp*cr - cy*sp*sr;
    quat_normalize(q_out);
  }

  static inline void quat_to_eulerZYX(const double q_in[4], double &roll, double &pitch, double &yaw)
  {
      double w = q_in[0], x = q_in[1], y = q_in[2], z = q_in[3];

      const double n = std::sqrt(w*w + x*x + y*y + z*z);
      if (n < 1e-12) { roll = pitch = yaw = 0.0; return; }
      w /= n; x /= n; y /= n; z /= n;

      const double sinr_cosp = 2.0 * (w * x + y * z);
      const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
      roll = std::atan2(sinr_cosp, cosr_cosp);

      const double sinp = 2.0 * (w * y - z * x);
      if (std::fabs(sinp) >= 1.0)
          pitch = std::copysign(1.5707963267948966, sinp);
      else
          pitch = std::asin(sinp);

      const double siny_cosp = 2.0 * (w * z + x * y);
      const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
      yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  static inline void mat3_mul_vec(const double A[3][3], const double v[3], double out[3]) {
    out[0] = A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2];
    out[1] = A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2];
    out[2] = A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2];
  }

  static inline void mat3_mul_mat3T(const double A[3][3], double M[3][3]) {
    // M = A * A^T
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            M[i][j] = A[i][0]*A[j][0] + A[i][1]*A[j][1] + A[i][2]*A[j][2];
        }
    }
  }

  static inline bool mat3_inv(const double A[3][3], double Ainv[3][3]) {
    const double a00=A[0][0], a01=A[0][1], a02=A[0][2];
    const double a10=A[1][0], a11=A[1][1], a12=A[1][2];
    const double a20=A[2][0], a21=A[2][1], a22=A[2][2];

    const double c00 =  (a11*a22 - a12*a21);
    const double c01 = -(a10*a22 - a12*a20);
    const double c02 =  (a10*a21 - a11*a20);

    const double c10 = -(a01*a22 - a02*a21);
    const double c11 =  (a00*a22 - a02*a20);
    const double c12 = -(a00*a21 - a01*a20);

    const double c20 =  (a01*a12 - a02*a11);
    const double c21 = -(a00*a12 - a02*a10);
    const double c22 =  (a00*a11 - a01*a10);

    const double det = a00*c00 + a01*c01 + a02*c02;
    if (std::fabs(det) < 1e-12) return false;

    const double invdet = 1.0 / det;

    // adj(A)^T / det
    Ainv[0][0] = c00*invdet; Ainv[0][1] = c10*invdet; Ainv[0][2] = c20*invdet;
    Ainv[1][0] = c01*invdet; Ainv[1][1] = c11*invdet; Ainv[1][2] = c21*invdet;
    Ainv[2][0] = c02*invdet; Ainv[2][1] = c12*invdet; Ainv[2][2] = c22*invdet;
    return true;
  }

  static inline double angle_wrap(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }
}