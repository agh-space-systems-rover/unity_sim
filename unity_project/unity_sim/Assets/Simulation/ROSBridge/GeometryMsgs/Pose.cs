using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct Pose
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Pose";

        [JsonProperty("position")]
        public Point Position { get; set; }

        [JsonProperty("orientation")]
        public Quaternion Orientation { get; set; }
    }
}
