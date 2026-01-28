#ifndef __CONTROL_FRAME_LOWLEVELCMD_H__
#define __CONTROL_FRAME_LOWLEVELCMD_H__

struct MotorCmd
{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd()
    {
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd
{
    MotorCmd motorCmd[16];
};

#endif
