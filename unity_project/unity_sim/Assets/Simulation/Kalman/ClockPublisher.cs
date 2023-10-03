using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

public class ClockPublisher : MonoBehaviour
{
    [SerializeField]
    private string topic = "/clock";

    [SerializeField]
    private float frequency = 120;

    private ROS ros;
    Publisher<ROSBridge.RosgraphMsgs.Clock> publisher = null;
    private double lastPublishTime = 0.0;

    private async void Start()
    {
        ros = new ROS();
        publisher = await ros.CreatePublisher<ROSBridge.RosgraphMsgs.Clock>(topic);
    }

    private async void OnApplicationQuit()
    {
        if (ros != null)
        {
            await ros.Close();
        }
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

        await publisher.Publish(new ROSBridge.RosgraphMsgs.Clock
        {
            ClockTime = ROSBridge.BuiltinInterfaces.Time.Simulated()
        });
    }
}
