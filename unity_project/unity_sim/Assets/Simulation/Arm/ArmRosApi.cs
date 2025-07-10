using UnityEngine;
using UnityEngine.InputSystem;
using ROSBridge;
using ROSBridge.KalmanInterfaces;

public class ArmRosApi : MonoBehaviour
{
    [SerializeField]
    private string tgtVelTopic = "/arm/joints/target_vel";
    [SerializeField]
    private string curPosTopic = "/arm/joints/current_pos";
    [SerializeField]
    private double controlTimeout = 0.1; // s
    [SerializeField]
    private float feedbackRate = 30.0f; // Hz

    // ROS
    private ROS ros;
    private Publisher<ArmValues> curPosPub;

    private ArmJoint[] joints = new ArmJoint[7];

    private float[] lastTargetVel = new float[7] { 0f, 0f, 0f, 0f, 0f, 0f, 0f };
    private double lastTargetVelTime = 0.0;
    private double lastPubTime = 0.0;

    private void Start()
    {
        GameObject curr = gameObject;
        for (int i = 0; i < 7; i++) {
            curr = curr.transform.GetChild(0).gameObject;
            joints[i] = curr.GetComponent<ArmJoint>();
        }

        // ROS setup
        ros = new ROS();
        
        // Subscribe to arm joints topic
        ros.CreateSubscription<ArmValues>(tgtVelTopic, (msg) => {
            for (int i = 0; i < 6; i++)
            {
                lastTargetVel[i] = msg.Joints[i];
            }
            lastTargetVel[6] = msg.Jaw;
            lastTargetVelTime = Time.time;
        });
        
        // Publisher for joint states feedback
        curPosPub = ros.CreatePublisher<ArmValues>(curPosTopic);
    }

    private async void Update()
    {
        if (Time.time - lastTargetVelTime < controlTimeout)
        {
            // Add target velocities to joints
            float dt = Time.deltaTime;
            for (int i = 0; i < 7; i++)
            {
                if (joints[i] != null)
                {
                    joints[i].OffsetTargetAngle(lastTargetVel[i] * dt * Mathf.Rad2Deg);
                }
            }
        }

        // Publish position feedback with rate limiting
        if (curPosPub != null && Time.time - lastPubTime >= 1.0 / feedbackRate)
        {
            var armJoints = new ArmValues();
            
            armJoints.Joints = new float[6];
            for (int i = 0; i < 6; i++)
            {
                armJoints.Joints[i] = joints[i].CurrentAngle() * Mathf.Deg2Rad;
            }
            armJoints.Jaw = joints[6].CurrentAngle() * Mathf.Deg2Rad;
            
            await curPosPub.Publish(armJoints);
            lastPubTime = Time.time;
        }
    }
}
