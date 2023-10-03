using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct PoseWithCovarianceStamped
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/PoseWithCovarianceStamped";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("pose")]
        public PoseWithCovariance Pose { get; set; }
    }
}
