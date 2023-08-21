using Newtonsoft.Json;

namespace ROSBridge.BuiltinInterfaces
{
    public struct Time
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "builtin_interfaces/msg/Time";

        [JsonProperty("sec")]
        public int Sec { get; set; }

        [JsonProperty("nanosec")]
        public uint Nanosec { get; set; }
    }
}