using Newtonsoft.Json;

namespace ROSBridge.TF2Msgs
{
    public struct TFMessage
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "tf2_msgs/msg/TFMessage";

        [JsonProperty("transforms")]
        public ROSBridge.GeometryMsgs.TransformStamped[] Transforms { get; set; }
    }
}
