using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.InputSystem;

public class PID
{
    public float kp, ki, kd;

    private float p = 0, i = 0, d = 0;
    private float lastError = 0;

    public PID(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    // returns dx
    public float Update(float e, float dt)
    {
        float de = e - lastError;
        lastError = e;

        p = e;
        i += e * dt; // i = S(e(t)dt)
        d = de / dt;

        return kp * p + ki * i + kd * d;
    }

    public float Update(float src, float tgt, float dt)
    {
        return Update(tgt - src, dt);
    }
}
