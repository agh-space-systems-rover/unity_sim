using UnityEngine;
using UnityEngine.InputSystem;
using ROSBridge;
using ROSBridge.KalmanInterfaces;

public class ArmRosApi : MonoBehaviour
{
    [SerializeField]
    private string curPosTopic = "/arm/current_pos";
    [SerializeField]
    private string tgtVelTopic = "/arm/target_vel";
    [SerializeField]
    private string jointsTgtVelTopic = "/arm/target_vel/joints";
    [SerializeField]
    private string jawTgtVelTopic = "/arm/target_vel/jaw";
    [SerializeField]
    private string jawTgtPosTopic = "/arm/target_pos/jaw";
    [SerializeField]
    private double controlTimeout = 0.5; // s
    [SerializeField]
    private float feedbackRate = 60.0f; // Hz

    // ROS
    private ROS ros;
    private Publisher<ArmValues> curPosPub;

    private ArmJoint[] joints = new ArmJoint[7];

    private float[] lastJointsTargetVel = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };
    private double lastJointsTargetVelTime = 0.0;
    private float lastJawTargetVel = 0f;
    private double lastJawTargetVelTime = 0.0;

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
        
        // Subscribe to arm velocity topic
        ros.CreateSubscription<ArmValues>(tgtVelTopic, (msg) => {
            for (int i = 0; i < 6; i++)
            {
                lastJointsTargetVel[i] = msg.Joints[i];
            }
            lastJointsTargetVelTime = Time.time;
            lastJawTargetVel = msg.Jaw;
            lastJawTargetVelTime = Time.time;
        });
        // Subscribe to arm joints topic
        ros.CreateSubscription<ArmValues>(jointsTgtVelTopic, (msg) => {
            for (int i = 0; i < 6; i++)
            {
                lastJointsTargetVel[i] = msg.Joints[i];
            }
            lastJointsTargetVelTime = Time.time;
        });
        // Subscribe to jaw target velocity topic
        ros.CreateSubscription<ArmValues>(jawTgtVelTopic, (msg) => {
            lastJawTargetVel = msg.Jaw;
            lastJawTargetVelTime = Time.time;
        });
        // Subscribe to jaw target position topic
        ros.CreateSubscription<ArmValues>(jawTgtPosTopic, (msg) => {
            if (joints[6] != null)
            {
                joints[6].SetTargetAngle(msg.Jaw * Mathf.Rad2Deg);
            }
        });
        
        // Publisher for joint states feedback
        curPosPub = ros.CreatePublisher<ArmValues>(curPosTopic);
    }

    private async void Update()
    {
        // Publish position feedback with rate limiting
        if (curPosPub != null && Time.time - lastPubTime >= 1.0 / feedbackRate)
        {
            await curPosPub.Publish(new ArmValues
            {
                Header = new ROSBridge.StdMsgs.Header
                {
                    Stamp = ROSBridge.BuiltinInterfaces.Time.Realtime(),
                    FrameId = ""
                },
                Joints = new float[6]
                {
                    joints[0].CurrentAngleUnnormalized() * Mathf.Deg2Rad,
                    joints[1].CurrentAngleUnnormalized() * Mathf.Deg2Rad,
                    joints[2].CurrentAngleUnnormalized() * Mathf.Deg2Rad,
                    joints[3].CurrentAngleUnnormalized() * Mathf.Deg2Rad,
                    joints[4].CurrentAngleUnnormalized() * Mathf.Deg2Rad,
                    joints[5].CurrentAngleUnnormalized() * Mathf.Deg2Rad
                },
                Jaw = joints[6].CurrentAngleUnnormalized() * Mathf.Deg2Rad
            });
            lastPubTime = Time.time;
        }

        // Apply target velocities to joints if within control timeout
        if (Time.time - lastJointsTargetVelTime < controlTimeout)
        {
            float dt = Time.deltaTime;
            for (int i = 0; i < 6; i++)
            {
                if (joints[i] != null)
                {
                    joints[i].OffsetTargetAngle(lastJointsTargetVel[i] * dt * Mathf.Rad2Deg, dt);
                    // ^ fitInTime=dt arg reduces the offset if it cannot be reached in dt seconds
                    // effectively clamping the target velocity to a maximum value
                }
            }
        }
        // Apply target velocity to jaw if within control timeout
        if (Time.time - lastJawTargetVelTime < controlTimeout && joints[6] != null)
        {
            float dt = Time.deltaTime;
            joints[6].OffsetTargetAngle(lastJawTargetVel * dt * Mathf.Rad2Deg, dt);
        }
    }
}
