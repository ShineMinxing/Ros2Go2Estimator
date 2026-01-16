#include "mex.h"
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>

#ifndef NOMINMAX
#  define NOMINMAX
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#  define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#include "fusion_estimator.h"

static FusionEstimatorCore* g_core = nullptr;
static bool g_atExitRegistered = false;

static void cleanup()
{
    if (g_core) { 
        delete g_core; 
        g_core = nullptr; 
    }

    while (mexIsLocked()) 
        mexUnlock();
}

static void require(bool cond, const char* msg)
{
    if (!cond) mexErrMsgIdAndTxt("fusion_estimator_mex:invalid", "%s", msg);
}

static std::string get_cmd(const mxArray* a)
{
    require(mxIsChar(a), "First input must be command string.");
    char* s = mxArrayToString(a);
    require(s, "Failed to read command string.");
    std::string out(s);
    mxFree(s);
    return out;
}

static long long get_i64_scalar(const mxArray* a, const char* name)
{
    require(mxIsNumeric(a) && mxGetNumberOfElements(a) == 1,
            (std::string(name) + " must be scalar numeric.").c_str());

    if (mxIsInt64(a)) {
        const int64_t v = *reinterpret_cast<const int64_t*>(mxGetData(a));
        return (long long)v;
    }
}

static void read_vec_float(const mxArray* a, std::vector<float>& out, int n, const char* name)
{
    require(mxIsDouble(a) && !mxIsComplex(a), "All vectors must be real double.");
    require((int)mxGetNumberOfElements(a) == n, "Vector length mismatch.");
    const double* p = (const double*)mxGetData(a);
    out.resize(n);
    for (int i = 0; i < n; ++i) out[i] = (float)p[i];
    (void)name;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    require(nrhs >= 1, "Usage: fusion_estimator_mex('status', status_) or fusion_estimator_mex('step', ...)");
    static unsigned int frame_count = 0;

    if (!g_atExitRegistered) {
        mexAtExit(cleanup);
        g_atExitRegistered = true;
    }

    const std::string cmd = get_cmd(prhs[0]);

    if (cmd == "reset") {
        cleanup();
        frame_count = 0;
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    if (cmd == "status") {
        require(nrhs >= 2, "Usage: st = fusion_estimator_mex('status', status_)");
        require(mxIsDouble(prhs[1]) && !mxIsComplex(prhs[1]), "status_ must be real double.");
        require((int)mxGetNumberOfElements(prhs[1]) == 200, "status_ must have 200 elements.");

        if (!g_core) {
            g_core = new FusionEstimatorCore();
            if (!mexIsLocked()) mexLock();
        }

        double status[200];
        std::memcpy(status, mxGetPr(prhs[1]), sizeof(double) * 200);

        g_core->fusion_estimator_status(status);

        if (nlhs > 0) {
            plhs[0] = mxCreateDoubleMatrix(200, 1, mxREAL);
            std::memcpy(mxGetPr(plhs[0]), status, sizeof(double) * 200);
        }
        return;
    }

    if (cmd == "step") {

        if (!g_core) {
            g_core = new FusionEstimatorCore();
            if (!mexIsLocked()) mexLock();
        }
        require(nrhs >= 8, "Usage: odom = fusion_estimator_mex('step', ts_ms, q, dq, tau, acc3, gyro3, quat4)");

        long long ts_ms = get_i64_scalar(prhs[1], "ts_ms");
        int nq = 16;

        std::vector<float> q, dq, tau, acc, gyro, quat;
        read_vec_float(prhs[2], q,   nq, "q");
        read_vec_float(prhs[3], dq,  nq, "dq");
        read_vec_float(prhs[4], tau, nq, "tau");
        read_vec_float(prhs[5], acc,  3, "acc3");
        read_vec_float(prhs[6], gyro, 3, "gyro3");
        read_vec_float(prhs[7], quat, 4, "quat4");

        LowlevelState st{};
        st.dataAvailable = true;
        st.imu.timestamp = ts_ms;

        st.imu.accelerometer[0] = acc[0];
        st.imu.accelerometer[1] = acc[1];
        st.imu.accelerometer[2] = acc[2];
        st.imu.gyroscope[0]     = gyro[0];
        st.imu.gyroscope[1]     = gyro[1];
        st.imu.gyroscope[2]     = gyro[2];
        st.imu.quaternion[0]    = quat[0];
        st.imu.quaternion[1]    = quat[1];
        st.imu.quaternion[2]    = quat[2];
        st.imu.quaternion[3]    = quat[3];

        for (int i = 0; i < nq; ++i) {
            st.motorState[i].q      = q[i];
            st.motorState[i].dq     = dq[i];
            st.motorState[i].tauEst = tau[i];
        }

        const Odometer odom = g_core->fusion_estimator(st);

        const char* fn[] = {"x","y","z","angularX","angularY","angularZ"};
        plhs[0] = mxCreateStructMatrix(1, 1, 6, fn);
        mxSetField(plhs[0], 0, "x",        mxCreateDoubleScalar((double)odom.x));
        mxSetField(plhs[0], 0, "y",        mxCreateDoubleScalar((double)odom.y));
        mxSetField(plhs[0], 0, "z",        mxCreateDoubleScalar((double)odom.z));
        mxSetField(plhs[0], 0, "angularX", mxCreateDoubleScalar((double)odom.angularX));
        mxSetField(plhs[0], 0, "angularY", mxCreateDoubleScalar((double)odom.angularY));
        mxSetField(plhs[0], 0, "angularZ", mxCreateDoubleScalar((double)odom.angularZ));
        return;
    }

    mexErrMsgIdAndTxt("fusion_estimator_mex:unknown", "Unknown cmd. Use 'status'/'step'/'reset'.");
}
