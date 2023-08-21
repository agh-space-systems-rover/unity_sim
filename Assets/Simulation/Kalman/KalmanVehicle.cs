using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using UnityEngine.InputSystem;

// damping - how much the wheel RPM is limited; high damping plays well with high torque

public class KalmanVehicle : MonoBehaviour
{
    [SerializeField]
    private WheelCollider flWheel, frWheel, blWheel, brWheel;

    [SerializeField]
    private Transform rootBone;

    [SerializeField]
    private Transform flSuspBone, frSuspBone, blSuspBone, brSuspBone;

    [SerializeField]
    private Transform flTurnBone, frTurnBone, blTurnBone, brTurnBone;

    [SerializeField]
    private Transform flWheelBone, frWheelBone, blWheelBone, brWheelBone;

    private float maxTorque = 30;
    private float driveSpeed = 1;

    // Dictionary that contains original rotations and positions of various bones.
    private Dictionary<Transform, Quaternion> ogBoneRots = new Dictionary<Transform, Quaternion>();
    private Dictionary<Transform, Vector3> ogSupToWheelVecs = new Dictionary<Transform, Vector3>();

    private Vector2 input = Vector2.zero;

    private void OnMove(InputValue value)
    {
        input = value.Get<Vector2>();
    }

    private void Start()
    {
        // Override center of mass.
        GetComponent<Rigidbody>().centerOfMass = new Vector3(0, 0.2F, 0);
    }

    private void FixedUpdate()
    {
        flWheel.steerAngle = input.x * 20.0F;
        frWheel.steerAngle = input.x * 20.0F;
        blWheel.steerAngle = -input.x * 20.0F;
        brWheel.steerAngle = -input.x * 20.0F;

        flWheel.motorTorque = maxTorque * input.y * driveSpeed;
        frWheel.motorTorque = maxTorque * input.y * driveSpeed;
        blWheel.motorTorque = maxTorque * input.y * driveSpeed;
        brWheel.motorTorque = maxTorque * input.y * driveSpeed;
    }

    private void Update()
    {
        CopyWheelTransform(flWheel, flSuspBone, flTurnBone, flWheelBone, false, true);
        CopyWheelTransform(frWheel, frSuspBone, frTurnBone, frWheelBone, true, true);
        CopyWheelTransform(blWheel, blSuspBone, blTurnBone, blWheelBone, false, false);
        CopyWheelTransform(brWheel, brSuspBone, brTurnBone, brWheelBone, true, false);
    }

    private void CopyWheelTransform(WheelCollider wheel, Transform suspBone, Transform turnBone, Transform wheelBone, bool rightSide, bool front)
    {
        // Put turn and suspension rotations in the dictionary if needed.
        if (!ogBoneRots.ContainsKey(turnBone))
        {
            ogBoneRots.Add(turnBone, turnBone.localRotation);
        }
        if (!ogBoneRots.ContainsKey(suspBone))
        {
            ogBoneRots.Add(suspBone, suspBone.localRotation);
        }
        // Also remember original positions of wheel bones relative to suspension bones.
        if (!ogSupToWheelVecs.ContainsKey(wheelBone))
        {
            Matrix4x4 worldToLocal = rootBone.parent.worldToLocalMatrix;
            Vector3 from = worldToLocal.MultiplyPoint(suspBone.position);
            Vector3 to = worldToLocal.MultiplyPoint(wheelBone.position);
            ogSupToWheelVecs.Add(wheelBone, to - from);
        }

        // Get the wheel's world position and rotation.
        Vector3 position;
        Quaternion rotation;
        wheel.GetWorldPose(out position, out rotation);

        // Apply wheel rotation about wheel bone's up axis.
        float angularSpeed = wheel.rpm / 60 * 360 * (rightSide ? 1 : -1);
        if (Mathf.Abs(angularSpeed) > 10)
        {
            wheelBone.Rotate(Vector3.up, angularSpeed * Time.deltaTime);
        }

        // Apply turn steering about turn bone's up axis.
        Quaternion newRot = ogBoneRots[turnBone] * Quaternion.Euler(0, -wheel.steerAngle, 0);
        turnBone.localRotation = Quaternion.Slerp(turnBone.localRotation, newRot, Time.deltaTime * 10.0F);

        // Point the suspension bone at the wheel. Rotate only about the up axis.
        Vector3 ogToWheel = ogSupToWheelVecs[wheelBone];
        Matrix4x4 worldToLocal2 = rootBone.parent.worldToLocalMatrix;
        Vector3 from2 = worldToLocal2.MultiplyPoint(suspBone.position);
        Vector3 to2 = worldToLocal2.MultiplyPoint(position);
        Vector3 toWheel = to2 - from2;
        float angle = Vector3.SignedAngle(ogToWheel, toWheel, Vector3.right);

        // Apply rotation about suspension bone's up axis.
        newRot = ogBoneRots[suspBone] * Quaternion.Euler(0, angle * (rightSide ? -1 : 1) * (front ? 2 : 1), 0);
        suspBone.localRotation = Quaternion.Slerp(suspBone.localRotation, newRot, Time.deltaTime * 10.0F);
    }
}
