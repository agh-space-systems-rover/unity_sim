using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

// Simulates the IMU built into the upside-down Livox Mid-360.
//
// Measurements are taken at the Mid-360's internal IMU position and expressed
// in its raw sensor axes. An upside-down sensor therefore reports approximately
// -9.81 m/s^2 on Z at rest, matching the real hardware. Keeping this separate
// from IMU.cs prevents FAST-LIO-specific timing and frame conventions from
// affecting the primary /imu/data sensor.
public class LivoxIMU : MonoBehaviour
{
    [SerializeField]
    private string frameId = "livox_imu_link";
    [SerializeField]
    private string topic = "/livox/imu";
    [SerializeField]
    private float frequency = 200.0f;
    [SerializeField]
    private float gravity = 9.81f;

    private readonly double[] orientationCovariance =
        new double[9] { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    private readonly double[] angularVelocityCovariance =
        new double[9] { 1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4 };
    private readonly double[] linearAccelerationCovariance =
        new double[9] { 1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3 };

    private Rigidbody roverRigidbody;
    private Vector3 previousPointVelocity;
    private Matrix4x4 unityToRos;
    private double publishAccumulator;
    private ROSConnection ros;

    private void Start()
    {
        if (frequency <= 0.0f)
        {
            Debug.LogError("LivoxIMU frequency must be greater than zero.");
            enabled = false;
            return;
        }

        float requiredFixedDeltaTime = 1.0f / frequency;
        if (UnityEngine.Time.fixedDeltaTime > requiredFixedDeltaTime)
        {
            UnityEngine.Time.fixedDeltaTime = requiredFixedDeltaTime;
        }

        roverRigidbody = GetComponentInParent<Rigidbody>();
        if (roverRigidbody == null)
        {
            Debug.LogError("LivoxIMU requires a Rigidbody in its parent hierarchy.");
            enabled = false;
            return;
        }

        previousPointVelocity =
            roverRigidbody.GetPointVelocity(transform.position);

        // ROS +X is Unity +Z, ROS +Y is Unity -X, ROS +Z is Unity +Y.
        Matrix4x4 rosToUnity = Matrix4x4.identity;
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        unityToRos = rosToUnity.inverse;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topic);
    }

    private void FixedUpdate()
    {
        if (ros == null || frequency <= 0.0f)
        {
            return;
        }

        Vector3 pointVelocity =
            roverRigidbody.GetPointVelocity(transform.position);
        Vector3 worldAcceleration =
            (pointVelocity - previousPointVelocity) / UnityEngine.Time.fixedDeltaTime;
        previousPointVelocity = pointVelocity;

        Vector3 specificForce =
            worldAcceleration - Physics.gravity.normalized * gravity;

        Vector3 bodyAcceleration =
            transform.InverseTransformDirection(specificForce);
        Vector3 bodyAngularVelocity =
            transform.InverseTransformDirection(roverRigidbody.angularVelocity);

        publishAccumulator += UnityEngine.Time.fixedDeltaTime;
        double publishPeriod = 1.0 / frequency;
        if (publishAccumulator + 1e-9 < publishPeriod)
        {
            return;
        }

        publishAccumulator -= publishPeriod;
        if (publishAccumulator < 0.0 || publishAccumulator >= publishPeriod)
        {
            // Unique samples cannot be generated faster than the physics rate.
            publishAccumulator = 0.0;
        }

        // fixedTime advances once per physics sample, even when Unity executes
        // several FixedUpdate calls in one rendered frame.
        ROSBridge.BuiltinInterfaces.Time stamp =
            ROSBridge.BuiltinInterfaces.Time.FromSimulationTime(
                UnityEngine.Time.fixedUnscaledTimeAsDouble
            );

        PublishMeasurement(
            stamp,
            unityToRos.MultiplyVector(bodyAcceleration),
            // Unity and ROS use opposite coordinate handedness. Acceleration
            // is a polar vector, but angular velocity is an axial vector, so
            // it requires the additional sign inversion under this reflection.
            -unityToRos.MultiplyVector(bodyAngularVelocity)
        );
    }

    private void PublishMeasurement(
        ROSBridge.BuiltinInterfaces.Time stamp,
        Vector3 acceleration,
        Vector3 angularVelocity)
    {
        ros.Publish(
            topic,
            new ImuMsg(
                new HeaderMsg(
                    new TimeMsg(stamp.Sec, stamp.Nanosec),
                    frameId
                ),
                // The real Livox driver does not provide an absolute
                // orientation.
                new QuaternionMsg(0.0, 0.0, 0.0, 1.0),
                orientationCovariance,
                new Vector3Msg(
                    angularVelocity.x,
                    angularVelocity.y,
                    angularVelocity.z
                ),
                angularVelocityCovariance,
                new Vector3Msg(
                    acceleration.x,
                    acceleration.y,
                    acceleration.z
                ),
                linearAccelerationCovariance
            )
        );
    }
}
