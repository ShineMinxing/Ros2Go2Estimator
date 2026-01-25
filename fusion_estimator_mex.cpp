#include "mex.h"
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

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    char cmd[32] = {0};
    mxGetString(prhs[0], cmd, sizeof(cmd));   // 假设 prhs[0] 一定是 char

    if (std::strcmp(cmd, "reset") == 0) {
        delete g_core;
        g_core = nullptr;
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    if (!g_core) g_core = new FusionEstimatorCore();

    if (std::strcmp(cmd, "status") == 0) {
        double status[200];
        std::memcpy(status, mxGetPr(prhs[1]), sizeof(status));  // 不检查长度/类型

        g_core->fusion_estimator_status(status);

        if (nlhs > 0) {
            plhs[0] = mxCreateDoubleMatrix(200, 1, mxREAL);
            std::memcpy(mxGetPr(plhs[0]), status, sizeof(status));
        }
        return;
    }

    if (std::strcmp(cmd, "step") == 0) {
        long long ts_ms = (long long)llround(mxGetScalar(prhs[1]));

        const double* qd    = mxGetPr(prhs[2]);
        const double* dqd   = mxGetPr(prhs[3]);
        const double* taud  = mxGetPr(prhs[4]);
        const double* accd  = mxGetPr(prhs[5]);
        const double* gyrod = mxGetPr(prhs[6]);
        const double* quatd = mxGetPr(prhs[7]);

        LowlevelState st{};
        st.imu.timestamp = ts_ms;

        st.imu.accelerometer[0] = (float)accd[0];
        st.imu.accelerometer[1] = (float)accd[1];
        st.imu.accelerometer[2] = (float)accd[2];

        st.imu.gyroscope[0] = (float)gyrod[0];
        st.imu.gyroscope[1] = (float)gyrod[1];
        st.imu.gyroscope[2] = (float)gyrod[2];

        st.imu.quaternion[0] = (float)quatd[0];
        st.imu.quaternion[1] = (float)quatd[1];
        st.imu.quaternion[2] = (float)quatd[2];
        st.imu.quaternion[3] = (float)quatd[3];

        for (int i = 0; i < 16; ++i) {
            st.motorState[i].q      = (float)qd[i];
            st.motorState[i].dq     = (float)dqd[i];
            st.motorState[i].tauEst = (float)taud[i];
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

    if (nlhs > 0) plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
}
