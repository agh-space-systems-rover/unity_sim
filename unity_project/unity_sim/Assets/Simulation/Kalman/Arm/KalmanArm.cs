using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SocketCANSharp;
using SocketCANSharp.Network;
using System.Linq;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System;
using UnityEngine.PlayerLoop;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine.InputSystem.EnhancedTouch;

public class CanDriver
{
    CanNetworkInterface vcan0 = CanNetworkInterface.GetAllInterfaces(true).First(iface => iface.Name.Equals("vcan0"));

    public static readonly BlockingCollection<CanFdFrame> canFrames = new BlockingCollection<CanFdFrame>();
    private static SafeFileDescriptorHandle socketHandle;


    public CanDriver(Action<CanFdFrame> frameHandler)
    {
        socketHandle = LibcNativeMethods.Socket(SocketCanConstants.PF_CAN, SocketType.Raw, SocketCanProtocolType.CAN_RAW);
        int enableFdFrames = 1;
        LibcNativeMethods.SetSockOpt(socketHandle, SocketLevel.SOL_CAN_RAW, CanSocketOptions.CAN_RAW_FD_FRAMES, ref enableFdFrames, sizeof(int));

        LibcNativeMethods.SetSockOpt(socketHandle, SocketLevel.SOL_SOCKET, SocketLevelOptions.SO_RCVTIMEO, new Timeval(0, 1), Marshal.SizeOf(typeof(Timeval)));
        if (socketHandle.IsInvalid)
        {
            UnityEngine.Debug.LogWarning("Failed to create socket.");
            return;
        }

        var ifr = new Ifreq("vcan0");
        int ioctlResult = LibcNativeMethods.Ioctl(socketHandle, SocketCanConstants.SIOCGIFINDEX, ifr);
        if (ioctlResult == -1)
        {
            UnityEngine.Debug.LogWarning("Failed to look up interface by name.");
            return;
        }

        var addr = new SockAddrCan(ifr.IfIndex)
        {
            CanFamily = 29
        };
        int bindResult = LibcNativeMethods.Bind(socketHandle, addr, Marshal.SizeOf(typeof(SockAddrCan)));
        if (bindResult == -1)
        {
            UnityEngine.Debug.LogWarning("Failed to bind to address.");
            return;
        }

        Task.Run(() => HandleLoop(frameHandler));
        Task.Run(() => ReadFrames());
    }

    private static void ReadFrames()
    {
        int frameSize = Marshal.SizeOf(typeof(CanFdFrame));
        UnityEngine.Debug.Log("Sniffing vcan0...");
        while (true)
        {
            var readFrame = new CanFdFrame();
            int nReadBytes = LibcNativeMethods.Read(socketHandle, ref readFrame, frameSize);
            if (nReadBytes > 0)
            {
                canFrames.Add(readFrame);
                // UnityEngine.Debug.Log($"Received frame with id {readFrame.CanId}, current count {canFrames.Count}");
            }
        }
    }

    private static void HandleLoop(Action<CanFdFrame> frameHandler)
    {
        while (true)
        {
            CanFdFrame frame = canFrames.Take();
            frameHandler(frame);
        }
    }
}

public enum CanMessageIds
{
    SET_VELOCITY = 0x025,
    SET_POSITION = 0x026,
    CONTROL_TYPE = 0x035,
    GRIPPER = 0xE3,
}
public class KalmanArm : MonoBehaviour
{
    // class VirtualJoint
    // {
    //     public Transform joint;
    //     public int canId;

    //     public enum ControlMode
    //     {
    //         SPEED,
    //         POSITION,
    //         POS_VEL,
    //     }

    //     public enum CanMessageIds
    //     {
    //         SET_VELOCITY = 0x025,
    //         SET_POSITION = 0x026,
    //         CONTROL_TYPE = 0x035,
    //     }

    //     public ControlMode mode;

    //     public float position; // in deg

    //     public float targetPosition;

    //     public double velocity; // in RPM

    //     public PID pid;

    //     public float maxSpeed = 60.0f;

    //     public float[] gearRatio = { 0.0125f, 0.00625f, 0.00625f, 0.01f, 0.0131034f, 0.0131034f };

    //     public VirtualJoint(Transform joint, int canId)
    //     {
    //         this.joint = joint;
    //         this.canId = canId;

    //         mode = ControlMode.POS_VEL;
    //         position = joint.rotation.eulerAngles[1];
    //         targetPosition = position;
    //         pid = new PID(10.0f, 0.01f, 0.01f);
    //     }

    //     public CanFdFrame GetStatusFrame()
    //     {
    //         CanFdFrame frame = new()
    //         {
    //             CanId = (uint)canId + 0x036,
    //             Length = 6
    //         };
    //         frame.Data = new byte[frame.Length];
    //         frame.Data[0] = 0x00;
    //         frame.Data[1] = 0x00;
    //         frame.Data[2] = 0x00;
    //         frame.Data[3] = 0x00;
    //         frame.Data[4] = 0x00;
    //         frame.Data[5] = 0x00;
    //         return frame;
    //     }

    //     public void ParseFrame(CanFdFrame frame)
    //     {
    //         // UnityEngine.Debug.Log($"Received frame on joint {joint} with id {frame.CanId}");
    //         int msgId = (int)frame.CanId & (0x7F);
    //         int jointId = ((int)frame.CanId >> 7) - 1;


    //         switch (msgId)
    //         {
    //             case (int)CanMessageIds.SET_VELOCITY:
    //                 velocity = BitConverter.ToInt16(frame.Data) * gearRatio[jointId] * 0.6;
    //                 break;

    //             case (int)CanMessageIds.SET_POSITION:
    //                 // targetPosition = (float)(BitConverter.ToInt32(frame.Data) * 100.0);
    //                 break;

    //             case (int)CanMessageIds.CONTROL_TYPE:
    //                 mode = (ControlMode)frame.Data[0];
    //                 break;

    //             default:
    //                 break;
    //         }

    //     }


    //     public void Update()
    //     {
    //         if (joint.name == "dof1")
    //         {
    //             UnityEngine.Debug.Log($"Updating joint {joint} with mode {mode}, position {position}, target {targetPosition}, velocity {velocity}");
    //         }
    //         if (this.mode == ControlMode.POS_VEL)
    //         {
    //             if (Math.Abs(velocity) > 0.001)
    //                 targetPosition = (float)(position + velocity);
    //         }
    //         position += Mathf.Clamp(pid.Update(position, targetPosition, Time.fixedDeltaTime), -maxSpeed * Time.fixedDeltaTime, maxSpeed * Time.fixedDeltaTime);

    //         // set only y rotation, with x and z not being changed
    //         joint.localRotation = Quaternion.Euler(joint.localRotation.eulerAngles[0], position, joint.localRotation.eulerAngles[2]);


    //     }
    // }

    [SerializeField]
    public List<GameObject> armObjects = new List<GameObject>();

    private CanDriver driver;

    private static Dictionary<JointId, JointController> armControllers = new Dictionary<JointId, JointController>();

    private static void HandleFrame(CanFdFrame frame)
    {
        int id = (int)frame.CanId >> 7;
        int msgId = (int)frame.CanId & 0x7F;

        if (frame.CanId == (uint)CanMessageIds.GRIPPER)
        {
            UnityEngine.Debug.Log($"Setting gripper to {BitConverter.ToUInt16(frame.Data)}");
            armControllers[JointId.Gripper].SetTargetPosition((2600 - BitConverter.ToUInt16(frame.Data)) * 100.0f / 1230.0f);
        }
        else if (frame.CanId == (uint)CanMessageIds.CONTROL_TYPE)
        {
            // UnityEngine.Debug.Log($"Setting control type to {BitConverter.ToInt32(frame.Data)}");
            foreach (JointController controller in armControllers.Values)
            {
                controller.SetControlMode((ControlMode)frame.Data[0]);
            }
        }
        else if (id > 0 && id <= armControllers.Count)
        {
            switch (msgId)
            {
                case (int)CanMessageIds.SET_VELOCITY:
                    if (JointController.jointTypes[(JointId)(id - 1)] == JointType.Differential)
                    {
                        armControllers[JointController.differentialJoints[(JointId)id - 1]].SetDiffVelocity(BitConverter.ToInt16(frame.Data) * 0.6f);
                    }
                    armControllers[(JointId)id - 1].SetTargetVelocity(BitConverter.ToInt16(frame.Data) * 0.6f);
                    break;

                case (int)CanMessageIds.SET_POSITION:
                    if (JointController.jointTypes[(JointId)(id - 1)] == JointType.Differential)
                    {
                        armControllers[JointController.differentialJoints[(JointId)id - 1]].SetDiffPosition(BitConverter.ToInt32(frame.Data) / 100.0f);
                    }
                    armControllers[(JointId)id - 1].SetTargetPosition(BitConverter.ToInt32(frame.Data) / 100.0f);
                    break;

                default:
                    UnityEngine.Debug.Log($"Unknown message id {msgId}");
                    break;
            }
        }
        else
        {
            UnityEngine.Debug.LogWarning($"Unknown joint id {id}");
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        foreach (GameObject armObject in armObjects)
        {
            JointController controller = armObject.GetComponent<JointController>();
            armControllers.Add(controller.jointId, controller);
        }
        driver = new CanDriver(HandleFrame);
    }

    // Update is called once per frame
    void Update()
    {
        // UnityEngine.Debug.Log($"{CanDriver.canFrames.Count} frames in queue");

    }
}
