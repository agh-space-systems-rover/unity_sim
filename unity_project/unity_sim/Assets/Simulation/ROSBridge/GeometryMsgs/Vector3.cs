using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct Vector3
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Vector3";

        [JsonProperty("x")]
        public double X { get; set; }

        [JsonProperty("y")]
        public double Y { get; set; }

        [JsonProperty("z")]
        public double Z { get; set; }
    }

}