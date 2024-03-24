using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

public class Altimeter : MonoBehaviour
{
    [SerializeField]
    private string frameId = "odom"; // not altimeter_link
    [SerializeField]
    private string topic = "/altimeter/pose";

    [SerializeField]
    private float frequency = 60;

    // [SerializeField]
    // private float noise = 0.0f;

    private double height = 0.0;
    private ROS ros;
    private Publisher<ROSBridge.GeometryMsgs.PoseWithCovarianceStamped> publisher = null;
    private double lastPublishTime = 0.0;
    private double variance = 0.1;

    private async void Start()
    {
        height = transform.position.y;

        ros = new ROS();
        publisher = await ros.CreatePublisher<ROSBridge.GeometryMsgs.PoseWithCovarianceStamped>(topic);

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
        height = transform.position.y;
    }

    private async void Update()
    {
        // publish to ROS
        // See: https://web.archive.org/web/20160612171450/http://wiki.ros.org/robot_localization/Tutorials/Preparing%20Your%20Sensor%20Data
        if (publisher == null)
        {
            return;
        }

        double currentTime = Time.unscaledTimeAsDouble;
        if (currentTime - lastPublishTime < 1.0 / frequency)
        {
            return;
        }
        lastPublishTime = currentTime;

        await publisher.Publish(new ROSBridge.GeometryMsgs.PoseWithCovarianceStamped
        {
            Header = new ROSBridge.StdMsgs.Header
            {
                Stamp = ROSBridge.BuiltinInterfaces.Time.Realtime(),
                FrameId = frameId
            },
            Pose = new ROSBridge.GeometryMsgs.PoseWithCovariance
            {
                Pose = new ROSBridge.GeometryMsgs.Pose
                {
                    Position = new ROSBridge.GeometryMsgs.Point
                    {
                        X = 0.0,
                        Y = 0.0,
                        Z = height
                    },
                    Orientation = new ROSBridge.GeometryMsgs.Quaternion
                    {
                        X = 0.0,
                        Y = 0.0,
                        Z = 0.0,
                        W = 1.0
                    }
                },
                Covariance = new double[] {
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, variance, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                }
            }
        });
    }
}
