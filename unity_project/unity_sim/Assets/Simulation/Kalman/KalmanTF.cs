using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEditor;

using ROSBridge;

[Serializable]
public class FrameToTransform
{
    public string frame;
    public Transform transform;
}

public class KalmanTF : MonoBehaviour
{
    [SerializeField]
    private string tfTopicPrefix = "";

    [SerializeField]
    private string baseFrame = "base_link";

    [SerializeField]
    private List<FrameToTransform> framesToTransforms = new List<FrameToTransform>();

    [SerializeField]
    private float frequency = 1;

    private ROS ros;
    private TFTree tf;
    private float lastUpdateTime = 0;

    private void Start()
    {
        // Initialize ROSBridge connection.
        ros = new ROS();

        // Initialize TF tree.
        tf = new TFTree(ros, tfTopicPrefix);
    }

    private void Update()
    {
        if (Time.time - lastUpdateTime < 1 / frequency)
        {
            return;
        }
        lastUpdateTime = Time.time;

        Matrix4x4 rosToUnity = Matrix4x4.identity;
        // 0, -1, 0 // Ros +Y is Unity -X (left) (second column)
        // 0,  0, 1 // Ros +Z is Unity +Y (up) (third column)
        // 1,  0, 0 // Ros +X is Unity +Z (forward) (first column)
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 unityToRos = rosToUnity.inverse;

        // For each frame.
        foreach (FrameToTransform frameToTransform in framesToTransforms)
        {
            try
            {
                // Get the transform from the base frame to the frame.
                Matrix4x4 transform = tf.LatestTransform(baseFrame, frameToTransform.frame);

                // Map the transform to Unity space.
                transform = rosToUnity * transform * unityToRos;

                // Apply the transform to the frame.
                frameToTransform.transform.SetLocalPositionAndRotation(transform.GetColumn(3), transform.rotation);
            }
            catch (TFTree.TransformMissingException)
            {
                // Do nothing.
            }
        }
    }
}
