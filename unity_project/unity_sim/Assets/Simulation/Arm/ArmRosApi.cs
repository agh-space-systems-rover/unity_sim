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
    private Publisher<ArmJointValues> curPosPub;

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
        ros.CreateSubscription<ArmJointValues>(tgtVelTopic, (msg) => {
            for (int i = 0; i < 6; i++)
            {
                lastTargetVel[i] = msg.Joints[i];
            }
            lastTargetVel[6] = msg.Jaw;
            lastTargetVelTime = Time.time;
        });
        
        // Publisher for joint states feedback
        curPosPub = ros.CreatePublisher<ArmJointValues>(curPosTopic);
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
            await curPosPub.Publish(new ArmJointValues
            {
                Header = new ROSBridge.StdMsgs.Header
                {
                    Stamp = ROSBridge.BuiltinInterfaces.Time.Realtime(),
                    FrameId = ""
                },
                Joints = new float[6]
                {
                    joints[0].CurrentAngle() * Mathf.Deg2Rad,
                    joints[1].CurrentAngle() * Mathf.Deg2Rad,
                    joints[2].CurrentAngle() * Mathf.Deg2Rad,
                    joints[3].CurrentAngle() * Mathf.Deg2Rad,
                    joints[4].CurrentAngle() * Mathf.Deg2Rad,
                    joints[5].CurrentAngle() * Mathf.Deg2Rad
                },
                Jaw = joints[6].CurrentAngle() * Mathf.Deg2Rad
            });
            lastPubTime = Time.time;
        }
    }
}
