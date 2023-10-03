using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct Point
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Point";

        [JsonProperty("x")]
        public double X { get; set; }

        [JsonProperty("y")]
        public double Y { get; set; }

        [JsonProperty("z")]
        public double Z { get; set; }
    }
}