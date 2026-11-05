using UnityEngine;
using ROSBridge;

/// <summary>
/// Publishes the exact Unity pose of base_link as nav_msgs/Odometry.
///
/// Unity uses X-right, Y-up, Z-forward. The published ROS frame uses
/// X-forward, Y-left, Z-up. No noise, delay, or filtering is applied.
/// </summary>
public class GroundTruthOdometry : MonoBehaviour
{
    [SerializeField]
    private string topic = "/benchmark/ground_truth";

    [SerializeField]
    private string frameId = "unity_world";

    [SerializeField]
    private string childFrameId = "base_link";

    [SerializeField]
    [Min(1.0f)]
    private float frequency = 100.0f;

    private ROS ros;
    private Publisher<ROSBridge.NavMsgs.Odometry> publisher;
    private Rigidbody parentRigidbody;
    private double publishAccumulator;
    private Vector3 previousPosition;
    private Quaternion previousRotation;

    private static readonly double[] ExactCovariance = new double[36];

    private void Start()
    {
        parentRigidbody = GetComponentInParent<Rigidbody>();
        previousPosition = transform.position;
        previousRotation = transform.rotation;

        ros = new ROS();
        publisher = ros.CreatePublisher<ROSBridge.NavMsgs.Odometry>(topic);
    }

    private void OnDestroy()
    {
        if (ros != null)
        {
            ros.Close();
            ros = null;
        }
    }

    private void FixedUpdate()
    {
        if (publisher == null || frequency <= 0.0f)
        {
            return;
        }

        publishAccumulator += Time.fixedUnscaledDeltaTime;
        double publishPeriod = 1.0 / frequency;
        if (publishAccumulator + 1e-9 < publishPeriod)
        {
            return;
        }

        // A publisher cannot provide more unique samples than the physics loop.
        publishAccumulator %= publishPeriod;
        PublishMeasurement();
    }

    private async void PublishMeasurement()
    {
        Matrix4x4 rosToUnity = Matrix4x4.identity;
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 unityToRos = rosToUnity.inverse;

        Vector3 positionRos = unityToRos.MultiplyPoint3x4(transform.position);
        Matrix4x4 rotationRosMatrix =
            unityToRos * Matrix4x4.Rotate(transform.rotation) * rosToUnity;
        Quaternion rotationRos = rotationRosMatrix.rotation;

        Vector3 linearVelocityUnity;
        Vector3 angularVelocityUnity;
        if (parentRigidbody != null)
        {
            linearVelocityUnity =
                parentRigidbody.GetPointVelocity(transform.position);
            angularVelocityUnity = parentRigidbody.angularVelocity;
        }
        else
        {
            linearVelocityUnity =
                (transform.position - previousPosition)
                / Time.fixedUnscaledDeltaTime;

            Quaternion rotationDelta =
                transform.rotation * Quaternion.Inverse(previousRotation);
            rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (angleDegrees > 180.0f)
            {
                angleDegrees -= 360.0f;
            }
            angularVelocityUnity =
                axis
                * (angleDegrees * Mathf.Deg2Rad / Time.fixedUnscaledDeltaTime);
        }

        previousPosition = transform.position;
        previousRotation = transform.rotation;

        Vector3 linearVelocityRosWorld =
            unityToRos.MultiplyVector(linearVelocityUnity);
        Vector3 angularVelocityRosWorld =
            unityToRos.MultiplyVector(angularVelocityUnity);
        Vector3 linearVelocityRosBody =
            Quaternion.Inverse(rotationRos) * linearVelocityRosWorld;
        Vector3 angularVelocityRosBody =
            Quaternion.Inverse(rotationRos) * angularVelocityRosWorld;

        await publisher.Publish(new ROSBridge.NavMsgs.Odometry
        {
            Header = new ROSBridge.StdMsgs.Header
            {
                // Keep benchmark reference on the same physics clock as the
                // simulated LiDAR and IMU, including across rendered-frame
                // hitches and FixedUpdate catch-up.
                Stamp = ROSBridge.BuiltinInterfaces.Time.FromSimulationTime(
                    Time.fixedUnscaledTimeAsDouble
                ),
                FrameId = frameId
            },
            ChildFrameId = childFrameId,
            Pose = new ROSBridge.GeometryMsgs.PoseWithCovariance
            {
                Pose = new ROSBridge.GeometryMsgs.Pose
                {
                    Position = new ROSBridge.GeometryMsgs.Point
                    {
                        X = positionRos.x,
                        Y = positionRos.y,
                        Z = positionRos.z
                    },
                    Orientation = new ROSBridge.GeometryMsgs.Quaternion
                    {
                        X = rotationRos.x,
                        Y = rotationRos.y,
                        Z = rotationRos.z,
                        W = rotationRos.w
                    }
                },
                Covariance = ExactCovariance
            },
            Twist = new ROSBridge.GeometryMsgs.TwistWithCovariance
            {
                Twist = new ROSBridge.GeometryMsgs.Twist
                {
                    Linear = new ROSBridge.GeometryMsgs.Vector3
                    {
                        X = linearVelocityRosBody.x,
                        Y = linearVelocityRosBody.y,
                        Z = linearVelocityRosBody.z
                    },
                    Angular = new ROSBridge.GeometryMsgs.Vector3
                    {
                        X = angularVelocityRosBody.x,
                        Y = angularVelocityRosBody.y,
                        Z = angularVelocityRosBody.z
                    }
                },
                Covariance = ExactCovariance
            }
        });
    }

}
