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

public class KalmanArm : MonoBehaviour
{
    class VirtualJoint
    {
        public Transform joint;
        public int canId;

        public enum ControlMode
        {
            SPEED,
            POSITION,
            POS_VEL,
        }

        public enum CanMessageIds
        {
            SET_VELOCITY = 0x025,
            SET_POSITION = 0x026,
            CONTROL_TYPE = 0x035,
        }

        public ControlMode mode;

        public float position; // in deg

        public float targetPosition;

        public double velocity; // in RPM

        public PID pid;

        public float maxSpeed = 60.0f;

        public float[] gearRatio = { 0.0125f, 0.00625f, 0.00625f, 0.01f, 0.0131034f, 0.0131034f };

        public VirtualJoint(Transform joint, int canId)
        {
            this.joint = joint;
            this.canId = canId;

            mode = ControlMode.POS_VEL;
            position = joint.rotation.eulerAngles[1];
            targetPosition = position;
            pid = new PID(10.0f, 0.01f, 0.01f);
        }

        public CanFdFrame GetStatusFrame()
        {
            CanFdFrame frame = new()
            {
                CanId = (uint)canId + 0x036,
                Length = 6
            };
            frame.Data = new byte[frame.Length];
            frame.Data[0] = 0x00;
            frame.Data[1] = 0x00;
            frame.Data[2] = 0x00;
            frame.Data[3] = 0x00;
            frame.Data[4] = 0x00;
            frame.Data[5] = 0x00;
            return frame;
        }

        public void ParseFrame(CanFdFrame frame)
        {
            // UnityEngine.Debug.Log($"Received frame on joint {joint} with id {frame.CanId}");
            int msgId = (int)frame.CanId & (0x7F);
            int jointId = ((int)frame.CanId >> 7) - 1;


            switch (msgId)
            {
                case (int)CanMessageIds.SET_VELOCITY:
                    velocity = BitConverter.ToInt16(frame.Data) * gearRatio[jointId] * 0.6;
                    break;

                case (int)CanMessageIds.SET_POSITION:
                    // targetPosition = (float)(BitConverter.ToInt32(frame.Data) * 100.0);
                    break;

                case (int)CanMessageIds.CONTROL_TYPE:
                    mode = (ControlMode)frame.Data[0];
                    break;

                default:
                    break;
            }

        }


        public void Update()
        {
            if (joint.name == "dof1")
            {
                UnityEngine.Debug.Log($"Updating joint {joint} with mode {mode}, position {position}, target {targetPosition}, velocity {velocity}");
            }
            if (this.mode == ControlMode.POS_VEL)
            {
                if (Math.Abs(velocity) > 0.001)
                    targetPosition = (float)(position + velocity);
            }
            position += Mathf.Clamp(pid.Update(position, targetPosition, Time.fixedDeltaTime), -maxSpeed * Time.fixedDeltaTime, maxSpeed * Time.fixedDeltaTime);

            // set only y rotation, with x and z not being changed
            joint.localRotation = Quaternion.Euler(joint.localRotation.eulerAngles[0], position, joint.localRotation.eulerAngles[2]);


        }
    }

    [SerializeField]
    private List<Transform> armJoints = new List<Transform>();
    CanNetworkInterface vcan0 = CanNetworkInterface.GetAllInterfaces(true).First(iface => iface.Name.Equals("vcan0"));

    [SerializeField]
    private List<ArticulationBody> armArticulations = new List<ArticulationBody>();

    private static readonly BlockingCollection<CanFdFrame> canFrames = new BlockingCollection<CanFdFrame>();
    private static SafeFileDescriptorHandle socketHandle;

    private static List<VirtualJoint> virtualJoints = new();

    private void FixedUpdate()
    {
        foreach (VirtualJoint joint in virtualJoints)
        {
            joint.Update();
        }
    }

    private static void HandleFrame(CanFdFrame frame)
    {
        int id = (int)frame.CanId >> 7;

        if (id > 0 && id <= virtualJoints.Count)
        {
            // UnityEngine.Debug.Log($"Handling frame with joint id {id}");
            virtualJoints[id - 1].ParseFrame(frame);
            // UnityEngine.Debug.Log($"Hadled");
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < armJoints.Count; i++)
        {
            virtualJoints.Add(new VirtualJoint(armJoints[i], i));
        }

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

        Task.Run(() => PrintLoop());
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
                // UnityEngine.Debug.LogWarning($"Received frame on vcan0, passing its {nReadBytes} bytes");
                canFrames.Add(readFrame);
            }
        }
    }




    private static void PrintLoop()
    {
        while (true)
        {
            CanFdFrame frame = canFrames.Take();
            // if (frame.CanId == 0xA5)
            // {
            //     if ((frame.CanId & (uint)CanIdFlags.CAN_RTR_FLAG) != 0)
            //     {
            //         UnityEngine.Debug.Log($":{SocketCanConstants.CAN_ERR_MASK & frame.CanId,8:X}   [{frame.Length:D2}]  RTR");
            //     }
            //     else
            //     {
            //         UnityEngine.Debug.Log($":{SocketCanConstants.CAN_ERR_MASK & frame.CanId,8:X}   [{frame.Length:D2}]  {BitConverter.ToString(frame.Data.Take(frame.Length).ToArray()).Replace("-", " ")}");
            //     }
            // }
            HandleFrame(frame);
        }
    }
    // Update is called once per frame
    void Update()
    {
        // foreach (Transform joint in armJoints)
        // {
        //     joint.Rotate(Vector3.up, 1);
        // }

    }
}
