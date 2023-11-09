using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct TransformStamped
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/TransformStamped";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("child_frame_id")]
        public string ChildFrameId { get; set; }

        [JsonProperty("transform")]
        public Transform Transform { get; set; }
    }
}
