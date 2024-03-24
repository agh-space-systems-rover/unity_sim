using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

public class IMU : MonoBehaviour
{
    [SerializeField]
    private string frameId = "imu_link";
    [SerializeField]
    private string topic = "/imu/data";
    [SerializeField]
    private float frequency = 60;
    [SerializeField]
    private float yawOffset = 0; // rad
    [SerializeField]
    private float gravity = 9.81f;

    private Vector3 prevPos;
    private Vector3 prevVel;
    private Vector3 prevAcc;
    private Quaternion prevAngPos;
    private Quaternion prevAngVel;
    private ROS ros;
    private Publisher<ROSBridge.SensorMsgs.Imu> imuPublisher = null;
    private double lastPublishTime = 0.0;
    private double[] covariance = new double[9] { 1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3 };

    private async void Start()
    {
        prevPos = transform.position;
        prevVel = Vector3.zero;
        prevAngPos = Quaternion.identity;

        ros = new ROS();
        imuPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.Imu>(topic);

        FixedUpdate();
    }

    private async void OnApplicationQuit()
    {
        if (ros != null)
        {
            await ros.Close();
        }
    }

    private void FixedUpdate()
    {
        // linear update
        Vector3 pos = transform.position;
        Vector3 vel = (pos - prevPos) / Time.fixedDeltaTime;
        Vector3 acc = (vel - prevVel) / Time.fixedDeltaTime;
        prevPos = pos;
        prevVel = vel;
        prevAcc = acc;

        // angular update
        Quaternion angPos = transform.rotation; // world space rotation as a quaternion
        Quaternion deltaAngPos = angPos * Quaternion.Inverse(prevAngPos);
        // Vector3 deltaAngPosAxis = new Vector3(deltaAngPos.x, deltaAngPos.y, deltaAngPos.z).normalized;
        // float deltaAngPosAngle = Mathf.Acos(deltaAngPos.w) * 2.0f;
        // Quaternion angVel = Quaternion.AngleAxis(deltaAngPosAngle / Time.fixedDeltaTime, deltaAngPosAxis);
        Quaternion angVel = Quaternion.SlerpUnclamped(Quaternion.identity, deltaAngPos, 1 / Time.fixedDeltaTime);
        prevAngPos = angPos;
        prevAngVel = angVel;
    }

    private async void Update()
    {
        // publish to ROS
        // See: https://web.archive.org/web/20160612171450/http://wiki.ros.org/robot_localization/Tutorials/Preparing%20Your%20Sensor%20Data
        if (imuPublisher == null)
        {
            return;
        }

        double currentTime = Time.unscaledTimeAsDouble;
        if (currentTime - lastPublishTime < 1.0 / frequency)
        {
            return;
        }
        lastPublishTime = currentTime;

        Quaternion angPos = prevAngPos;
        Quaternion angVel = prevAngVel;
        Vector3 acc = prevAcc;
        acc.y += gravity; // gravity compensation
        acc = transform.InverseTransformDirection(acc); // make acceleration local to the sensor

        Matrix4x4 rosToUnity = Matrix4x4.identity;
        // 0, -1, 0 // Ros +Y is Unity -X (left) (second column)
        // 0,  0, 1 // Ros +Z is Unity +Y (up) (third column)
        // 1,  0, 0 // Ros +X is Unity +Z (forward) (first column)
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 unityToRos = rosToUnity.inverse;

        // Transform absolute orientation to ROS coordinate system.
        Matrix4x4 unityToOffsetUnity = Matrix4x4.Rotate(Quaternion.AngleAxis(Mathf.Rad2Deg * yawOffset, Vector3.up));
        Matrix4x4 angPosMat = Matrix4x4.Rotate(angPos);
        angPosMat = unityToRos * unityToOffsetUnity * angPosMat * rosToUnity;
        angPos = angPosMat.rotation;

        // Transform angular velocity to ROS coordinate system.
        Matrix4x4 angVelMat = Matrix4x4.Rotate(angVel);
        angVelMat = unityToRos * angVelMat * rosToUnity;
        angVel = angVelMat.rotation;

        // Transform acceleration vector to ROS coordinate system.
        acc = unityToRos.MultiplyVector(acc);

        await imuPublisher.Publish(new ROSBridge.SensorMsgs.Imu
        {
            Header = new ROSBridge.StdMsgs.Header
            {
                Stamp = ROSBridge.BuiltinInterfaces.Time.Realtime(),
                FrameId = frameId
            },
            Orientation = new ROSBridge.GeometryMsgs.Quaternion
            {
                X = angPos.x,
                Y = angPos.y,
                Z = angPos.z,
                W = angPos.w
            },
            OrientationCovariance = covariance,
            AngularVelocity = new ROSBridge.GeometryMsgs.Vector3
            {
                X = toNormalizedRad(angVel.eulerAngles.x), // roll
                Y = toNormalizedRad(angVel.eulerAngles.y), // pitch
                Z = toNormalizedRad(angVel.eulerAngles.z)  // yaw
            },
            AngularVelocityCovariance = covariance,
            LinearAcceleration = new ROSBridge.GeometryMsgs.Vector3
            {
                X = acc.x,
                Y = acc.y,
                Z = acc.z
            },
            LinearAccelerationCovariance = covariance
        });
    }

    private float toNormalizedRad(float angle)
    {
        // while (angle > 180.0f)
        // {
        //     angle -= 360.0f;
        // }
        // while (angle < -180.0f)
        // {
        //     angle += 360.0f;
        // }
        // return angle;
        float n = Mathf.Floor((angle + 180) / 360);
        angle -= n * 360;
        return angle * Mathf.Deg2Rad;
    }
}
