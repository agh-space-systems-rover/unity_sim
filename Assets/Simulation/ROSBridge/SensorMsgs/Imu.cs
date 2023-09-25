using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct Imu
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/Imu";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("orientation")]
        public ROSBridge.GeometryMsgs.Quaternion Orientation { get; set; }

        [JsonProperty("orientation_covariance")]
        public double[] OrientationCovariance { get; set; }

        [JsonProperty("angular_velocity")]
        public ROSBridge.GeometryMsgs.Vector3 AngularVelocity { get; set; }

        [JsonProperty("angular_velocity_covariance")]
        public double[] AngularVelocityCovariance { get; set; }

        [JsonProperty("linear_acceleration")]
        public ROSBridge.GeometryMsgs.Vector3 LinearAcceleration { get; set; }

        [JsonProperty("linear_acceleration_covariance")]
        public double[] LinearAccelerationCovariance { get; set; }
    }

}