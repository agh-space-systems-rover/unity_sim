using System;
using System.Runtime.InteropServices;
using UnityEngine;
using RosMessageTypes.Sensor;
using ROSBridgeTime = ROSBridge.BuiltinInterfaces.Time;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Publisher;
using UnitySensors.ROS.Serializer;
using UnitySensors.ROS.Serializer.Std;
using UnitySensors.Sensor;

namespace Kalman.UnitySimulation.Livox
{
    [Serializable]
    public class LivoxPointCloud2MsgSerializer : RosMsgSerializer<PointCloud2Msg>
    {
        private const int PointStep = 26;
        private const int LaserLines = 4;

        [SerializeField]
        private HeaderSerializer _header;

        private IPointCloudInterface<PointXYZI> _sourceInterface;
        private UnitySensor _sensor;
        private int _pointsNum;
        private double _pointIntervalNanoseconds;

        [StructLayout(LayoutKind.Explicit)]
        private struct FloatBits
        {
            [FieldOffset(0)] public float value;
            [FieldOffset(0)] public uint bits;
        }

        [StructLayout(LayoutKind.Explicit)]
        private struct DoubleBits
        {
            [FieldOffset(0)] public double value;
            [FieldOffset(0)] public ulong bits;
        }

        public void SetSource(
            IPointCloudInterface<PointXYZI> sourceInterface,
            UnitySensor sensor)
        {
            _sourceInterface = sourceInterface;
            _sensor = sensor;
        }

        public override void Init()
        {
            base.Init();

            if (_sourceInterface == null || _sensor == null)
            {
                throw new InvalidOperationException(
                    "Livox point cloud publisher requires a UnitySensor implementing IPointCloudInterface<PointXYZI>."
                );
            }

            _header.Init();

            _pointsNum = _sourceInterface.pointCloud.points.Length;
            int dataSize = _pointsNum * PointStep;
            _pointIntervalNanoseconds = _sensor.dt * 1e9 / _pointsNum;

            _msg.height = 1;
            _msg.width = (uint)_pointsNum;
            _msg.fields = new[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("intensity", 12, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("tag", 16, PointFieldMsg.UINT8, 1),
                new PointFieldMsg("line", 17, PointFieldMsg.UINT8, 1),
                new PointFieldMsg("timestamp", 18, PointFieldMsg.FLOAT64, 1),
            };
            _msg.is_bigendian = false;
            _msg.point_step = PointStep;
            _msg.row_step = (uint)dataSize;
            _msg.data = new byte[dataSize];
            _msg.is_dense = true;
        }

        public override PointCloud2Msg Serialize()
        {
            _msg.header = _header.Serialize();
            ROSBridgeTime stamp = ROSBridgeTime.FromSimulationTime(
                Time.unscaledTimeAsDouble
            );
            _msg.header.stamp.sec = stamp.Sec;
            _msg.header.stamp.nanosec = stamp.Nanosec;

            var points = _sourceInterface.pointCloud.points;
            for (int i = 0; i < _pointsNum; i++)
            {
                PointXYZI point = points[i];
                int offset = i * PointStep;

                // Unity uses right/up/forward; ROS uses forward/left/up.
                WriteFloatLittleEndian(_msg.data, offset, point.position.z);
                WriteFloatLittleEndian(_msg.data, offset + 4, -point.position.x);
                WriteFloatLittleEndian(_msg.data, offset + 8, point.position.y);
                WriteFloatLittleEndian(_msg.data, offset + 12, point.intensity);
                _msg.data[offset + 16] = 0;
                _msg.data[offset + 17] = (byte)(i % LaserLines);
                WriteDoubleLittleEndian(
                    _msg.data,
                    offset + 18,
                    i * _pointIntervalNanoseconds
                );
            }

            return _msg;
        }

        private static void WriteFloatLittleEndian(byte[] data, int offset, float value)
        {
            uint bits = new FloatBits { value = value }.bits;
            data[offset] = (byte)bits;
            data[offset + 1] = (byte)(bits >> 8);
            data[offset + 2] = (byte)(bits >> 16);
            data[offset + 3] = (byte)(bits >> 24);
        }

        private static void WriteDoubleLittleEndian(byte[] data, int offset, double value)
        {
            ulong bits = new DoubleBits { value = value }.bits;
            data[offset] = (byte)bits;
            data[offset + 1] = (byte)(bits >> 8);
            data[offset + 2] = (byte)(bits >> 16);
            data[offset + 3] = (byte)(bits >> 24);
            data[offset + 4] = (byte)(bits >> 32);
            data[offset + 5] = (byte)(bits >> 40);
            data[offset + 6] = (byte)(bits >> 48);
            data[offset + 7] = (byte)(bits >> 56);
        }
    }

    public class LivoxPointCloud2MsgPublisher
        : RosMsgPublisher<LivoxPointCloud2MsgSerializer, PointCloud2Msg>
    {
        [SerializeField, Interface(typeof(IPointCloudInterface<PointXYZI>))]
        private UnityEngine.Object _source;

        private void Awake()
        {
            _serializer.SetSource(
                _source as IPointCloudInterface<PointXYZI>,
                _source as UnitySensor
            );
        }
    }
}
