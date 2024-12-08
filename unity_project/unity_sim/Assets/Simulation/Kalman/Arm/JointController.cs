using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Pipes;
using System.Runtime.CompilerServices;
using UnityEngine;

public enum JointType
{
    Normal,
    Differential,
    Gripper
}

public enum JointId
{
    Joint1,
    Joint2,
    Joint3,
    Joint4,
    Joint5,
    Joint6,
    Gripper
}

public enum ControlMode
{
    SPEED,
    POSITION,
    POS_VEL,
}

public class JointController : MonoBehaviour
{
    [SerializeField]
    public JointId jointId;

    private ArticulationBody articulation;

    private ControlMode mode = ControlMode.POS_VEL;

    private float targetPosition, diffPosition;
    private float targetVelocity, diffVelocity;

    private PID pid = new PID(1.0f, 0.0f, 0.0f);
    private const float maxSpeed = 30.0f;

    public static Dictionary<JointId, JointType> jointTypes = new Dictionary<JointId, JointType>
    {
        {JointId.Joint1, JointType.Normal},
        { JointId.Joint2, JointType.Normal},
        { JointId.Joint3, JointType.Normal},
        { JointId.Joint4, JointType.Normal},
        { JointId.Joint5, JointType.Differential},
        { JointId.Joint6, JointType.Differential},
        { JointId.Gripper, JointType.Gripper}
    };

    public static Dictionary<JointId, JointId> differentialJoints = new Dictionary<JointId, JointId>
    {
        {JointId.Joint5, JointId.Joint6},
        {JointId.Joint6, JointId.Joint5}
    };


    private float[] gearRatio = { 0.0125f, 0.00625f, 0.00625f, 0.01f, 0.0131034f, 0.0131034f, 1.0f };
    private float[] invertDirection = { -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };

    // LIFE CYCLE

    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
        targetPosition = CurrentPrimaryAxisRotation();
        targetVelocity = 0;
    }

    void FixedUpdate()
    {
        if (articulation == null)
        {
            articulation = GetComponent<ArticulationBody>();
            if (articulation == null)
            {
                UnityEngine.Debug.LogWarning($"Joint {jointId} not initialized");
                return;
            }
            targetPosition = CurrentPrimaryAxisRotation();
        }


        if (jointTypes[jointId] == JointType.Gripper)
        {
            RotateTo(targetPosition);
        }
        else if (jointTypes[jointId] == JointType.Differential)
        {
            float realVelocity = (diffVelocity - invertDirection[(int)jointId] * targetVelocity) / 2;
            if (mode == ControlMode.POS_VEL)
            {
                if (Math.Abs(realVelocity) > 0.001)
                    targetPosition = (float)(CurrentPrimaryAxisRotation() + realVelocity * invertDirection[(int)jointId]);
            }
            float positionChange = Mathf.Clamp(pid.Update(CurrentPrimaryAxisRotation(), targetPosition, Time.fixedDeltaTime), -maxSpeed * Time.fixedDeltaTime, maxSpeed * Time.fixedDeltaTime);
            RotateTo(CurrentPrimaryAxisRotation() + positionChange);
        }
        else
        {
            if (mode == ControlMode.POS_VEL)
            {
                if (Math.Abs(targetVelocity) > 0.001)
                    targetPosition = (float)(CurrentPrimaryAxisRotation() + targetVelocity * invertDirection[(int)jointId]);
            }
            float positionChange = Mathf.Clamp(pid.Update(CurrentPrimaryAxisRotation(), targetPosition, Time.fixedDeltaTime), -maxSpeed * Time.fixedDeltaTime, maxSpeed * Time.fixedDeltaTime);
            if (jointId == JointId.Joint1)
            {
                UnityEngine.Debug.Log($"Joint {jointId} position {CurrentPrimaryAxisRotation()} target {targetPosition} change {positionChange}");
            }
            // UnityEngine.Debug.Log($"Joint {jointId} position {CurrentPrimaryAxisRotation()} target {targetPosition} change {positionChange}");
            RotateTo(CurrentPrimaryAxisRotation() + positionChange);
        }
    }


    // MOVEMENT HELPERS

    float CurrentPrimaryAxisRotation()
    {
        if (articulation == null)
        {
            return 0;
        }
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    // PUBLIC INTERFACE

    public void SetControlMode(ControlMode mode)
    {
        this.mode = mode;
    }

    public void SetTargetPosition(float targetPosition)
    {
        if (jointId == JointId.Joint1)
        {
            // UnityEngine.Debug.Log($"Setting target position {targetPosition}");
        }
        this.targetPosition = targetPosition * gearRatio[(int)jointId];
    }

    public void SetDiffPosition(float diffPosition)
    {
        this.diffPosition = diffPosition * gearRatio[(int)jointId];
    }

    public void SetTargetVelocity(float targetVelocity)
    {
        this.targetVelocity = targetVelocity * gearRatio[(int)jointId];
        // if (jointId == JointId.Joint1)
        // {
        //     UnityEngine.Debug.Log($"Setting target velocity {this.targetVelocity}");
        // }
    }

    public void SetDiffVelocity(float diffVelocity)
    {
        this.diffVelocity = diffVelocity * gearRatio[(int)jointId];
    }
}