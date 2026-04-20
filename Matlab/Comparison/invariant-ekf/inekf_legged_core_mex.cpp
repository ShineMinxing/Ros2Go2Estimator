#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>
#include <map>
#include <algorithm>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "src/InEKF.h"

namespace {

enum StatusIndex {
    IndexInOrOut = 0,
    IndexStatusOK = 1,

    IndexContactTauThreshold = 10,
    IndexKinematicsPosStd = 11,

    IndexGyroNoise = 20,
    IndexAccelNoise = 21,
    IndexGyroBiasNoise = 22,
    IndexAccelBiasNoise = 23,
    IndexContactNoise = 24,
    IndexUseQuatInit = 25,
};

struct OdometerLite {
    double XPos = 0.0, YPos = 0.0, ZPos = 0.0;
    double XVel = 0.0, YVel = 0.0, ZVel = 0.0;
    double XAcc = 0.0, YAcc = 0.0, ZAcc = 0.0;

    double RollRad = 0.0, PitchRad = 0.0, YawRad = 0.0;
    double RollVel = 0.0, PitchVel = 0.0, YawVel = 0.0;
    double RollAcc = 0.0, PitchAcc = 0.0, YawAcc = 0.0;

    double FootfallAverageX = 0.0;
    double FootfallAverageY = 0.0;
    double FootfallAverageYaw = 0.0;

    double FLFootLanded = 0.0;
    double FRFootLanded = 0.0;
    double RLFootLanded = 0.0;
    double RRFootLanded = 0.0;

    double LoadedWeight = 0.0;
};

struct LegParam {
    int id = 0;
    int q0 = -1;
    int q1 = -1;
    int q2 = -1;
    int contact_tau = -1;

    int hip_sign = 1;

    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
    double hip = 0.0;
    double thigh = 0.0;
    double shank = 0.0;
};

class InEKFLeggedCore {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InEKFLeggedCore() {
        setGo2P();
        resetFilterOnly();
    }

    void resetAll() {
        resetFilterOnly();
    }

    void fusion_status(double status[200]) {
        if (!status) return;

        const int cmd = static_cast<int>(std::llround(status[IndexInOrOut]));

        if (cmd == 1) {
            status[IndexInOrOut] = 0.0;
            status[IndexStatusOK] += 1.0;

            if (status[IndexContactTauThreshold] > 0.0)
                contact_tau_threshold_ = status[IndexContactTauThreshold];
            if (status[IndexKinematicsPosStd] > 0.0)
                kinematics_pos_std_ = status[IndexKinematicsPosStd];

            if (status[IndexGyroNoise] > 0.0)
                noise_.setGyroscopeNoise(status[IndexGyroNoise]);
            if (status[IndexAccelNoise] > 0.0)
                noise_.setAccelerometerNoise(status[IndexAccelNoise]);
            if (status[IndexGyroBiasNoise] > 0.0)
                noise_.setGyroscopeBiasNoise(status[IndexGyroBiasNoise]);
            if (status[IndexAccelBiasNoise] > 0.0)
                noise_.setAccelerometerBiasNoise(status[IndexAccelBiasNoise]);
            if (status[IndexContactNoise] > 0.0)
                noise_.setContactNoise(status[IndexContactNoise]);

            use_quat_init_ = (status[IndexUseQuatInit] != 0.0);

            if (filter_) filter_->setNoiseParams(noise_);
            return;
        }

        if (cmd == 2) {
            status[IndexInOrOut] = 0.0;
            status[IndexStatusOK] += 10.0;
            if (status[IndexStatusOK] > 999.0) status[IndexStatusOK] = 1.0;

            status[IndexContactTauThreshold] = contact_tau_threshold_;
            status[IndexKinematicsPosStd] = kinematics_pos_std_;
            status[IndexGyroNoise] = std::sqrt(noise_.getGyroscopeCov()(0,0));
            status[IndexAccelNoise] = std::sqrt(noise_.getAccelerometerCov()(0,0));
            status[IndexGyroBiasNoise] = std::sqrt(noise_.getGyroscopeBiasCov()(0,0));
            status[IndexAccelBiasNoise] = std::sqrt(noise_.getAccelerometerBiasCov()(0,0));
            status[IndexContactNoise] = std::sqrt(noise_.getContactCov()(0,0));
            status[IndexUseQuatInit] = use_quat_init_ ? 1.0 : 0.0;

            status[30] = static_cast<double>(preset_code_);
            status[31] = static_cast<double>(leg_num_);

            if (filter_) {
                inekf::RobotState s = filter_->getState();
                Eigen::Matrix3d R = s.getRotation();
                Eigen::Vector3d v = s.getVelocity();
                Eigen::Vector3d p = s.getPosition();
                Eigen::Vector3d bg = s.getGyroscopeBias();
                Eigen::Vector3d ba = s.getAccelerometerBias();

                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                rotToEulerZYX(R, roll, pitch, yaw);

                status[50] = p.x();
                status[51] = p.y();
                status[52] = p.z();
                status[53] = v.x();
                status[54] = v.y();
                status[55] = v.z();
                status[56] = roll;
                status[57] = pitch;
                status[58] = yaw;
                status[59] = bg.x();
                status[60] = bg.y();
                status[61] = bg.z();
                status[62] = ba.x();
                status[63] = ba.y();
                status[64] = ba.z();
            } else {
                for (int i = 50; i <= 64; ++i) status[i] = 0.0;
            }

            status[70] = last_contact_[0] ? 1.0 : 0.0;
            status[71] = last_contact_[1] ? 1.0 : 0.0;
            status[72] = last_contact_[2] ? 1.0 : 0.0;
            status[73] = last_contact_[3] ? 1.0 : 0.0;
            status[74] = last_loaded_weight_;
            return;
        }

        if (cmd == 3) {
            status[IndexInOrOut] = 0.0;
            status[IndexStatusOK] += 20.0;
            if (status[IndexStatusOK] > 999.0) status[IndexStatusOK] = 1.0;
            resetFilterOnly();
            return;
        }

        if (cmd == 4) {
            status[IndexInOrOut] = 0.0;
            status[IndexStatusOK] += 40.0;
            if (status[IndexStatusOK] > 999.0) status[IndexStatusOK] = 1.0;
            setMP();
            resetFilterOnly();
            return;
        }

        if (cmd == 99) {
            status[IndexInOrOut] = 0.0;
            status[IndexStatusOK] += 99.0;
            if (status[IndexStatusOK] > 999.0) status[IndexStatusOK] = 1.0;
            setGo2P();
            resetFilterOnly();
            return;
        }
    }

    OdometerLite step(long long ts_ms,
                      const double* q,
                      const double* dq,
                      const double* tau,
                      const double* acc,
                      const double* gyro,
                      const double* quat)
    {
        OdometerLite odom;

        if (!q || !dq || !tau || !acc || !gyro || !quat)
            return odom;

        if (!filter_) {
            initializeFilter(quat);
        }

        if (!initialized_) {
            initializeFilter(quat);
        }

        if (last_ts_ms_ >= 0) {
            const double dt = 1e-3 * static_cast<double>(ts_ms - last_ts_ms_);
            if (dt > 1e-6 && dt < 1.0) {
                Eigen::Matrix<double,6,1> m;
                m << gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2];
                filter_->Propagate(m, dt);
            }
        }

        std::vector<std::pair<int,bool> > contacts;
        contacts.reserve(leg_num_);
        last_loaded_weight_ = 0.0;
        for (int i = 0; i < leg_num_; ++i) {
            const bool on = std::fabs(tau[legs_[i].contact_tau]) >= contact_tau_threshold_;
            contacts.push_back(std::make_pair(legs_[i].id, on));
            last_contact_[i] = on;
            last_loaded_weight_ += std::fabs(tau[legs_[i].contact_tau]);
        }
        for (int i = leg_num_; i < 4; ++i) {
            last_contact_[i] = false;
        }

        filter_->setContacts(contacts);

        inekf::vectorKinematics kin_meas;
        kin_meas.reserve(leg_num_);
        for (int i = 0; i < leg_num_; ++i) {
            const Eigen::Vector3d p_bc = footPositionBase(legs_[i], q);

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3,1>(0,3) = p_bc;

            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();
            cov.block<3,3>(3,3) = (kinematics_pos_std_ * kinematics_pos_std_) * Eigen::Matrix3d::Identity();

            kin_meas.push_back(inekf::Kinematics(legs_[i].id, pose, cov));
        }

        filter_->CorrectKinematics(kin_meas);

        for (int i = 0; i < 16; ++i) {
            last_q_[i] = q[i];
            last_dq_[i] = dq[i];
            last_tau_[i] = tau[i];
        }
        for (int i = 0; i < 3; ++i) {
            last_acc_[i] = acc[i];
            last_gyro_[i] = gyro[i];
        }
        for (int i = 0; i < 4; ++i) {
            last_quat_[i] = quat[i];
        }
        last_ts_ms_ = ts_ms;

        fillOdom(odom);
        return odom;
    }

private:
    struct LegPreset {
        int id;
        int q0, q1, q2;
        int contact_tau;
        int hip_sign;
        double mx, my, mz;
        double hip, thigh, shank;
    };

    void resetFilterOnly() {
        filter_.reset();
        initialized_ = false;
        last_ts_ms_ = -1;
        last_loaded_weight_ = 0.0;
        for (int i = 0; i < 16; ++i) {
            last_q_[i] = 0.0;
            last_dq_[i] = 0.0;
            last_tau_[i] = 0.0;
        }
        for (int i = 0; i < 3; ++i) {
            last_acc_[i] = 0.0;
            last_gyro_[i] = 0.0;
        }
        last_quat_[0] = 1.0;
        last_quat_[1] = 0.0;
        last_quat_[2] = 0.0;
        last_quat_[3] = 0.0;
        for (int i = 0; i < 4; ++i) {
            last_contact_[i] = false;
        }
    }

    void initializeFilter(const double* quat) {
        inekf::RobotState s;
        s.setRotation(use_quat_init_ ? quatToRotation(quat) : Eigen::Matrix3d::Identity());
        s.setVelocity(Eigen::Vector3d::Zero());
        s.setPosition(Eigen::Vector3d::Zero());
        s.setGyroscopeBias(Eigen::Vector3d::Zero());
        s.setAccelerometerBias(Eigen::Vector3d::Zero());

        filter_.reset(new inekf::InEKF(s, noise_));
        initialized_ = true;
    }

    Eigen::Matrix3d quatToRotation(const double* quat) const {
        const double w = quat[0];
        const double x = quat[1];
        const double y = quat[2];
        const double z = quat[3];
        const double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n < 1e-12) return Eigen::Matrix3d::Identity();
        Eigen::Quaterniond qn(w/n, x/n, y/n, z/n);
        return qn.toRotationMatrix();
    }

    void rotToEulerZYX(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) const {
        const double sinp = -R(2,0);
        if (std::fabs(sinp) >= 1.0)
            pitch = std::copysign(M_PI * 0.5, sinp);
        else
            pitch = std::asin(sinp);

        roll = std::atan2(R(2,1), R(2,2));
        yaw  = std::atan2(R(1,0), R(0,0));
    }

    Eigen::Vector3d footPositionBase(const LegParam& leg, const double* q) const {
        const double q1 = q[leg.q0];
        const double q2 = q[leg.q1];
        const double q3 = q[leg.q2];

        const double s1 = std::sin(q1);
        const double c1 = std::cos(q1);
        const double s2 = std::sin(q2);
        const double c2 = std::cos(q2);
        const double s23 = std::sin(q2 + q3);
        const double c23 = std::cos(q2 + q3);

        const double L = leg.thigh * c2 + leg.shank * c23;

        Eigen::Vector3d p;
        p.x() = leg.mx - leg.thigh * s2 - leg.shank * s23;
        p.y() = leg.my + leg.hip_sign * leg.hip * c1 + L * s1;
        p.z() = leg.mz + leg.hip_sign * leg.hip * s1 - L * c1;
        return p;
    }

    void fillFootAverage(double& fx, double& fy, double& fyaw) const {
        fx = 0.0;
        fy = 0.0;
        fyaw = 0.0;

        if (!filter_) return;

        inekf::RobotState s = filter_->getState();
        Eigen::MatrixXd X = s.getX();
        const std::map<int,int> cps = filter_->getEstimatedContactPositions();

        Eigen::Vector3d cp[4];
        bool have[4] = {false, false, false, false};

        for (std::map<int,int>::const_iterator it = cps.begin(); it != cps.end(); ++it) {
            if (it->first >= 0 && it->first < 4) {
                cp[it->first] = X.block<3,1>(0, it->second);
                have[it->first] = true;
            }
        }

        int cnt = 0;
        for (int i = 0; i < leg_num_; ++i) {
            if (have[i]) {
                fx += cp[i].x();
                fy += cp[i].y();
                cnt++;
            }
        }

        if (cnt > 0) {
            fx /= static_cast<double>(cnt);
            fy /= static_cast<double>(cnt);
        }

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        rotToEulerZYX(s.getRotation(), roll, pitch, yaw);

        if (leg_num_ == 4 && have[0] && have[1] && have[2] && have[3]) {
            const double front_x = 0.5 * (cp[0].x() + cp[1].x());
            const double front_y = 0.5 * (cp[0].y() + cp[1].y());
            const double rear_x  = 0.5 * (cp[2].x() + cp[3].x());
            const double rear_y  = 0.5 * (cp[2].y() + cp[3].y());
            fyaw = std::atan2(front_y - rear_y, front_x - rear_x);
        } else {
            fyaw = yaw;
        }
    }

    void fillOdom(OdometerLite& odom) const {
        if (!filter_) return;

        inekf::RobotState s = filter_->getState();
        Eigen::Matrix3d R = s.getRotation();
        Eigen::Vector3d v = s.getVelocity();
        Eigen::Vector3d p = s.getPosition();
        Eigen::Vector3d bg = s.getGyroscopeBias();
        Eigen::Vector3d ba = s.getAccelerometerBias();

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        rotToEulerZYX(R, roll, pitch, yaw);

        const Eigen::Vector3d gyro_corr(last_gyro_[0] - bg.x(), last_gyro_[1] - bg.y(), last_gyro_[2] - bg.z());
        const Eigen::Vector3d acc_corr(last_acc_[0] - ba.x(), last_acc_[1] - ba.y(), last_acc_[2] - ba.z());
        const Eigen::Vector3d acc_world = R * acc_corr + Eigen::Vector3d(0.0, 0.0, -9.81);

        odom.XPos = p.x();
        odom.YPos = p.y();
        odom.ZPos = p.z();

        odom.XVel = v.x();
        odom.YVel = v.y();
        odom.ZVel = v.z();

        odom.XAcc = acc_world.x();
        odom.YAcc = acc_world.y();
        odom.ZAcc = acc_world.z();

        odom.RollRad = roll;
        odom.PitchRad = pitch;
        odom.YawRad = yaw;

        odom.RollVel = gyro_corr.x();
        odom.PitchVel = gyro_corr.y();
        odom.YawVel = gyro_corr.z();

        odom.RollAcc = 0.0;
        odom.PitchAcc = 0.0;
        odom.YawAcc = 0.0;

        fillFootAverage(odom.FootfallAverageX, odom.FootfallAverageY, odom.FootfallAverageYaw);

        odom.FLFootLanded = last_contact_[0] ? 1.0 : 0.0;
        odom.FRFootLanded = last_contact_[1] ? 1.0 : 0.0;
        odom.RLFootLanded = last_contact_[2] ? 1.0 : 0.0;
        odom.RRFootLanded = last_contact_[3] ? 1.0 : 0.0;

        odom.LoadedWeight = last_loaded_weight_;
    }

    void setPresetCommon() {
        for (int i = 0; i < 4; ++i) {
            legs_[i] = LegParam();
        }
        leg_num_ = 4;
    }

    void setGo2P() {
        setPresetCommon();
        preset_code_ = 99;

        legs_[0] = makeLeg(0, 0, 1, 2, 2, +1,  0.1934, -0.0465,  0.0000, 0.0955, 0.2130, 0.2350);
        legs_[1] = makeLeg(1, 4, 5, 6, 6, -1,  0.1934,  0.0465,  0.0000, 0.0955, 0.2130, 0.2350);
        legs_[2] = makeLeg(2, 8, 9,10,10, +1, -0.1934, -0.0465,  0.0000, 0.0955, 0.2130, 0.2350);
        legs_[3] = makeLeg(3,12,13,14,14, -1, -0.1934,  0.0465,  0.0000, 0.0955, 0.2130, 0.2350);
    }

    void setMP() {
        setPresetCommon();
        preset_code_ = 4;

        legs_[0] = makeLeg(0, 0, 1, 2, 2, +1,  0.2878, -0.0700, 0.0000, 0.1709, 0.2600, 0.2900);
        legs_[1] = makeLeg(1, 4, 5, 6, 6, -1,  0.2878,  0.0700, 0.0000, 0.1709, 0.2600, 0.2900);
        legs_[2] = makeLeg(2, 8, 9,10,10, +1, -0.2878, -0.0700, 0.0000, 0.1709, 0.2600, 0.2900);
        legs_[3] = makeLeg(3,12,13,14,14, -1, -0.2878,  0.0700, 0.0000, 0.1709, 0.2600, 0.2900);
    }

    LegParam makeLeg(int id, int q0, int q1, int q2, int contact_tau, int hip_sign,
                     double mx, double my, double mz, double hip, double thigh, double shank) {
        LegParam leg;
        leg.id = id;
        leg.q0 = q0;
        leg.q1 = q1;
        leg.q2 = q2;
        leg.contact_tau = contact_tau;
        leg.hip_sign = hip_sign;
        leg.mx = mx;
        leg.my = my;
        leg.mz = mz;
        leg.hip = hip;
        leg.thigh = thigh;
        leg.shank = shank;
        return leg;
    }

private:
    std::unique_ptr<inekf::InEKF> filter_;
    inekf::NoiseParams noise_;

    LegParam legs_[4];
    int leg_num_ = 4;
    int preset_code_ = 99;

    double contact_tau_threshold_ = 1.0;
    double kinematics_pos_std_ = 0.01;
    bool use_quat_init_ = true;

    bool initialized_ = false;
    long long last_ts_ms_ = -1;

    double last_q_[16] = {0.0};
    double last_dq_[16] = {0.0};
    double last_tau_[16] = {0.0};
    double last_acc_[3] = {0.0};
    double last_gyro_[3] = {0.0};
    double last_quat_[4] = {1.0,0.0,0.0,0.0};
    bool   last_contact_[4] = {false,false,false,false};
    double last_loaded_weight_ = 0.0;
};

static std::unique_ptr<InEKFLeggedCore> g_core;

void cleanupMex() {
    g_core.reset();
}

mxArray* makeOdomStruct(const OdometerLite& odom) {
    const char* fn[] = {
        "XPos","YPos","ZPos",
        "XVel","YVel","ZVel",
        "XAcc","YAcc","ZAcc",
        "RollRad","PitchRad","YawRad",
        "RollVel","PitchVel","YawVel",
        "RollAcc","PitchAcc","YawAcc",
        "FootfallAverageX","FootfallAverageY","FootfallAverageYaw",
        "FLFootLanded","FRFootLanded","RLFootLanded","RRFootLanded",
        "LoadedWeight"
    };

    mxArray* out = mxCreateStructMatrix(1, 1, 26, fn);

    mxSetField(out, 0, "XPos", mxCreateDoubleScalar(odom.XPos));
    mxSetField(out, 0, "YPos", mxCreateDoubleScalar(odom.YPos));
    mxSetField(out, 0, "ZPos", mxCreateDoubleScalar(odom.ZPos));

    mxSetField(out, 0, "XVel", mxCreateDoubleScalar(odom.XVel));
    mxSetField(out, 0, "YVel", mxCreateDoubleScalar(odom.YVel));
    mxSetField(out, 0, "ZVel", mxCreateDoubleScalar(odom.ZVel));

    mxSetField(out, 0, "XAcc", mxCreateDoubleScalar(odom.XAcc));
    mxSetField(out, 0, "YAcc", mxCreateDoubleScalar(odom.YAcc));
    mxSetField(out, 0, "ZAcc", mxCreateDoubleScalar(odom.ZAcc));

    mxSetField(out, 0, "RollRad", mxCreateDoubleScalar(odom.RollRad));
    mxSetField(out, 0, "PitchRad", mxCreateDoubleScalar(odom.PitchRad));
    mxSetField(out, 0, "YawRad", mxCreateDoubleScalar(odom.YawRad));

    mxSetField(out, 0, "RollVel", mxCreateDoubleScalar(odom.RollVel));
    mxSetField(out, 0, "PitchVel", mxCreateDoubleScalar(odom.PitchVel));
    mxSetField(out, 0, "YawVel", mxCreateDoubleScalar(odom.YawVel));

    mxSetField(out, 0, "RollAcc", mxCreateDoubleScalar(odom.RollAcc));
    mxSetField(out, 0, "PitchAcc", mxCreateDoubleScalar(odom.PitchAcc));
    mxSetField(out, 0, "YawAcc", mxCreateDoubleScalar(odom.YawAcc));

    mxSetField(out, 0, "FootfallAverageX", mxCreateDoubleScalar(odom.FootfallAverageX));
    mxSetField(out, 0, "FootfallAverageY", mxCreateDoubleScalar(odom.FootfallAverageY));
    mxSetField(out, 0, "FootfallAverageYaw", mxCreateDoubleScalar(odom.FootfallAverageYaw));

    mxSetField(out, 0, "FLFootLanded", mxCreateDoubleScalar(odom.FLFootLanded));
    mxSetField(out, 0, "FRFootLanded", mxCreateDoubleScalar(odom.FRFootLanded));
    mxSetField(out, 0, "RLFootLanded", mxCreateDoubleScalar(odom.RLFootLanded));
    mxSetField(out, 0, "RRFootLanded", mxCreateDoubleScalar(odom.RRFootLanded));

    mxSetField(out, 0, "LoadedWeight", mxCreateDoubleScalar(odom.LoadedWeight));
    return out;
}

void requireVectorLength(const mxArray* a, mwSize n, const char* name) {
    if (!mxIsDouble(a) || mxIsComplex(a) || mxGetNumberOfElements(a) != n) {
        mexErrMsgIdAndTxt("inekf_legged_mex:badInput", "%s must be a real double vector of length %d.", name, (int)n);
    }
}

} // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("inekf_legged_mex:usage", "First input must be command string: 'reset' | 'status' | 'step'.");
    }

    char cmd[32] = {0};
    mxGetString(prhs[0], cmd, sizeof(cmd));

    if (std::strcmp(cmd, "reset") == 0) {
        g_core.reset();
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    if (!g_core) {
        mexAtExit(cleanupMex);
        g_core.reset(new InEKFLeggedCore());
    }

    if (std::strcmp(cmd, "status") == 0) {
        if (nrhs < 2) mexErrMsgIdAndTxt("inekf_legged_mex:usage", "'status' needs a 200x1 or 1x200 double vector.");
        requireVectorLength(prhs[1], 200, "status");

        double status[200] = {0.0};
        std::memcpy(status, mxGetPr(prhs[1]), sizeof(status));

        g_core->fusion_status(status);

        if (nlhs > 0) {
            plhs[0] = mxCreateDoubleMatrix(200, 1, mxREAL);
            std::memcpy(mxGetPr(plhs[0]), status, sizeof(status));
        }
        return;
    }

    if (std::strcmp(cmd, "step") == 0) {
        if (nrhs < 8) {
            mexErrMsgIdAndTxt("inekf_legged_mex:usage",
                "'step' needs: ts_ms, q16, dq16, tau16, acc3, gyro3, quat4.");
        }

        const long long ts_ms = static_cast<long long>(std::llround(mxGetScalar(prhs[1])));
        requireVectorLength(prhs[2], 16, "q16");
        requireVectorLength(prhs[3], 16, "dq16");
        requireVectorLength(prhs[4], 16, "tau16");
        requireVectorLength(prhs[5], 3, "acc3");
        requireVectorLength(prhs[6], 3, "gyro3");
        requireVectorLength(prhs[7], 4, "quat4");

        const double* q = mxGetPr(prhs[2]);
        const double* dq = mxGetPr(prhs[3]);
        const double* tau = mxGetPr(prhs[4]);
        const double* acc = mxGetPr(prhs[5]);
        const double* gyro = mxGetPr(prhs[6]);
        const double* quat = mxGetPr(prhs[7]);

        const OdometerLite odom = g_core->step(ts_ms, q, dq, tau, acc, gyro, quat);

        if (nlhs > 0) {
            plhs[0] = makeOdomStruct(odom);
        }
        return;
    }

    mexErrMsgIdAndTxt("inekf_legged_mex:badCommand", "Unknown command.");
}