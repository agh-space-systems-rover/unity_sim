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

    public void SendFastStatus(Dictionary<JointId, JointController> armControllers)
    {

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
        /**
 * @brief Structure representing the fast status (only vel and pos).
 *
 * @param velocity int16_t Velocity - bytes 0-1
 * @param position int32_t Position - bytes 2-5
//  */
// typedef struct __attribute__((__packed__))
// {
//   int16_t velocity;  // RPM*10
//   int32_t position;  // pozycja 0-36000 (co 0.01 deg)
// } jointMotorFastStatus_t;
// #define CMD_JOINT_FAST_STATUS 0x036
// #define LEN_JOINT_FAST_STATUS 6

    }
}
