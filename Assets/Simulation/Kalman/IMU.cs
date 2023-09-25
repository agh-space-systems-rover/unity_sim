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

    // [SerializeField]
    // private float noise = 0.0f;

    [SerializeField]
    private float gravity = 9.81f;


    private Vector3 prevPos;
    private Vector3 prevVel;
    private Vector3 prevAcc;
    private Quaternion prevAngPos;
    private Quaternion prevAngVel;
    private ROS ros;
    Publisher<ROSBridge.SensorMsgs.Imu> imuPublisher = null;
    private double lastPublishTime = 0.0;
    private double[] covariance = new double[9] { 1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3 };

    private async void Start()
    {
        prevPos = transform.position;
        prevVel = Vector3.zero;

        prevAngPos = Quaternion.identity;

        ros = new ROS();
        imuPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.Imu>(topic);
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
        Quaternion angPos = transform.rotation;
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

        await imuPublisher.Publish(new ROSBridge.SensorMsgs.Imu
        {
            Header = new ROSBridge.StdMsgs.Header
            {
                Stamp = ROSBridge.BuiltinInterfaces.Time.Current(),
                FrameId = frameId
            },
            Orientation = new ROSBridge.GeometryMsgs.Quaternion
            {
                X = -angPos.z,
                Y = angPos.x,
                Z = -angPos.y,
                W = angPos.w
            },
            OrientationCovariance = covariance,
            AngularVelocity = new ROSBridge.GeometryMsgs.Vector3
            {
                X = -toNormalizedRad(angVel.eulerAngles.z),  // roll
                Y = toNormalizedRad(angVel.eulerAngles.x), // pitch
                Z = -toNormalizedRad(angVel.eulerAngles.y)   // yaw
            },
            AngularVelocityCovariance = covariance,
            LinearAcceleration = new ROSBridge.GeometryMsgs.Vector3
            {
                X = acc.z,
                Y = -acc.x,
                Z = acc.y
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
