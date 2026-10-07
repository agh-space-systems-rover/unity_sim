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
    private bool useMag = true;
    [SerializeField]
    private float magOffset = 0; // rad
    [SerializeField]
    private float gravity = 9.81f;

    private Vector3 prevPos;
    private Vector3 prevVel;
    private Vector3 linearAcceleration;
    private Quaternion prevAngPos;
    private Vector3 angularVelocity;
    private Rigidbody parentRigidbody;
    private ROS ros;
    private Publisher<ROSBridge.SensorMsgs.Imu> imuPublisher = null;
    private double publishAccumulator = 0.0;
    private double[] covariance = new double[9] { 1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3 };
    private float noMagYawError = 0.0f;

    private void Start()
    {
        parentRigidbody = GetComponentInParent<Rigidbody>();
        prevPos = transform.position;
        prevVel = parentRigidbody != null
            ? parentRigidbody.GetPointVelocity(transform.position)
            : Vector3.zero;
        prevAngPos = transform.rotation;

        ros = new ROS();
        imuPublisher = ros.CreatePublisher<ROSBridge.SensorMsgs.Imu>(topic);

        GPS.GPSMeasurement gpsMeasurement = GPS.MeasureAtPos(transform.position);
        if (gpsMeasurement.hasFix)
        {
            magOffset = (float)gpsMeasurement.yaw;
            Debug.Log($"IMU magOffset set from GPS yaw: {magOffset} rad");
        }
        else
        {
            Debug.LogWarning("IMU: No GPS fix available to set magOffset.");
        }
    }

    private void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }

    private void FixedUpdate()
    {
        Vector3 worldVelocity;
        Vector3 worldAngularVelocity;

        if (parentRigidbody != null)
        {
            // GetPointVelocity includes the velocity induced by rotation at the
            // sensor's offset from the rigid body's center of mass.
            worldVelocity = parentRigidbody.GetPointVelocity(transform.position);
            worldAngularVelocity = parentRigidbody.angularVelocity;
        }
        else
        {
            Vector3 pos = transform.position;
            worldVelocity = (pos - prevPos) / Time.fixedDeltaTime;
            prevPos = pos;

            Quaternion angPos = transform.rotation;
            Quaternion deltaAngPos = angPos * Quaternion.Inverse(prevAngPos);
            deltaAngPos.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (angleDegrees > 180.0f)
            {
                angleDegrees -= 360.0f;
            }
            worldAngularVelocity =
                axis * (angleDegrees * Mathf.Deg2Rad / Time.fixedDeltaTime);
            prevAngPos = angPos;
        }

        Vector3 worldAcceleration =
            (worldVelocity - prevVel) / Time.fixedDeltaTime;
        prevVel = worldVelocity;

        // An accelerometer reports specific force, so a stationary sensor
        // measures the acceleration opposite to gravity.
        Vector3 specificForce =
            worldAcceleration - Physics.gravity.normalized * gravity;
        linearAcceleration = transform.InverseTransformDirection(specificForce);

        // sensor_msgs/Imu requires angular velocity in the sensor frame.
        // The previous implementation left it in the Unity world frame.
        angularVelocity =
            transform.InverseTransformDirection(worldAngularVelocity);

        // update noMagYawError
        if (!useMag)
        {
            float angSpeed = angularVelocity.magnitude;
            noMagYawError += angSpeed * Time.fixedDeltaTime * Random.Range(-0.05f, 0.3f);
        } else {
            noMagYawError = 0.0f;
        }

        if (imuPublisher == null || frequency <= 0)
        {
            return;
        }

        publishAccumulator += Time.fixedDeltaTime;
        double publishPeriod = 1.0 / frequency;
        if (publishAccumulator + 1e-9 < publishPeriod)
        {
            return;
        }
        publishAccumulator -= publishPeriod;
        if (publishAccumulator < 0.0 || publishAccumulator >= publishPeriod)
        {
            // The physics rate is the maximum rate of unique IMU samples.
            publishAccumulator = 0.0;
        }

        PublishMeasurement();
    }

    private async void PublishMeasurement()
    {
        Quaternion angPos = transform.rotation;
        Vector3 angVel = angularVelocity;
        Vector3 acc = linearAcceleration;

        Matrix4x4 rosToUnity = Matrix4x4.identity;
        // 0, -1, 0 // Ros +Y is Unity -X (left) (second column)
        // 0,  0, 1 // Ros +Z is Unity +Y (up) (third column)
        // 1,  0, 0 // Ros +X is Unity +Z (forward) (first column)
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 unityToRos = rosToUnity.inverse;

        // Transform absolute orientation to ROS coordinate system.
        Matrix4x4 unityToOffsetUnity = Matrix4x4.Rotate(Quaternion.AngleAxis(Mathf.Rad2Deg * (magOffset + noMagYawError), Vector3.up));
        Matrix4x4 angPosMat = Matrix4x4.Rotate(angPos);
        angPosMat = unityToRos * unityToOffsetUnity * angPosMat * rosToUnity;
        angPos = angPosMat.rotation;

        // Transform local sensor vectors to ROS coordinates.
        angVel = unityToRos.MultiplyVector(angVel);
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
                X = angVel.x,
                Y = angVel.y,
                Z = angVel.z
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
}
