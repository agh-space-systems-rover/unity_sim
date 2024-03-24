using Newtonsoft.Json;
using ROSBridge.BuiltinInterfaces;

namespace ROSBridge.StdMsgs
{
    public struct ColorRGBA
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "std_msgs/msg/ColorRGBA";

        [JsonProperty("r")]
        public float R { get; set; }

        [JsonProperty("g")]
        public float G { get; set; }

        [JsonProperty("b")]
        public float B { get; set; }

        [JsonProperty("a")]
        public float A { get; set; }
    }
}