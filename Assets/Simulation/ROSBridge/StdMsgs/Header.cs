using Newtonsoft.Json;
using ROSBridge.BuiltinInterfaces;

namespace ROSBridge.StdMsgs
{
    public struct Header
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "std_msgs/msg/Header";

        [JsonProperty("stamp")]
        public ROSBridge.BuiltinInterfaces.Time Stamp { get; set; }

        [JsonProperty("frame_id")]
        public string FrameId { get; set; }
    }
}