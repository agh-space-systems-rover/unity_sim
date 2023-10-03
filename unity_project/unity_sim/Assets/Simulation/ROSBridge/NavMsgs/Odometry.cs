using Newtonsoft.Json;

namespace ROSBridge.NavMsgs
{
    public struct Odometry
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "nav_msgs/msg/Odometry";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("child_frame_id")]
        public string ChildFrameId { get; set; }

        [JsonProperty("pose")]
        public ROSBridge.GeometryMsgs.PoseWithCovariance Pose { get; set; }

        [JsonProperty("twist")]
        public ROSBridge.GeometryMsgs.TwistWithCovariance Twist { get; set; }
    }
}
