using Newtonsoft.Json;

namespace ROSBridge.RosgraphMsgs
{
    public struct Clock
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "rosgraph_msgs/msg/Clock";

        [JsonProperty("clock")]
        public ROSBridge.BuiltinInterfaces.Time ClockTime { get; set; }
    }
}
