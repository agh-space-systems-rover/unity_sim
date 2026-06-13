using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridge;

public class LaserMeter : MonoBehaviour
{
	[SerializeField]
	private string topic = "/arc/laser_distance";
	[SerializeField]
	private float frequency = 10;
	[SerializeField] 
	private float maxRange = 100.0f;
	[SerializeField]
	private LayerMask layerMask = -1;

	private ROS ros;
	private Publisher<ROSBridge.StdMsgs.Float32> rangePublisher = null;
	private double lastPublishTime = 0.0;

	private void Start()
	{
		ros = new ROS();
		rangePublisher = ros.CreatePublisher<ROSBridge.StdMsgs.Float32>(topic);
	}

	private void OnApplicationQuit()
	{
		if (ros != null)
		{
			ros.Close();
		}
	}

	private async void Update()
	{
		if (rangePublisher == null)
		{
			return;
		}

		double currentTime = Time.unscaledTimeAsDouble;
		if (currentTime - lastPublishTime < 1.0 / frequency)
		{
			return;
		}
		lastPublishTime = currentTime;

		float measuredRange = maxRange;
		RaycastHit hit;
		
		if (Physics.Raycast(transform.position, transform.forward, out hit, maxRange, layerMask))
		{
			measuredRange = hit.distance;
            Debug.DrawLine(transform.position, hit.point, Color.red);
		} else {
            Debug.DrawRay(transform.position, transform.forward * maxRange, Color.green);
        }

		await rangePublisher.Publish(new ROSBridge.StdMsgs.Float32
		{
			Data = measuredRange
		});
	}
}
