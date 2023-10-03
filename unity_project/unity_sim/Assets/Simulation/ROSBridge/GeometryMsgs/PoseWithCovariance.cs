using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct PoseWithCovariance
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/PoseWithCovariance";

        [JsonProperty("pose")]
        public Pose Pose { get; set; }

        [JsonProperty("covariance")]
        public double[] Covariance { get; set; }
    }
}
