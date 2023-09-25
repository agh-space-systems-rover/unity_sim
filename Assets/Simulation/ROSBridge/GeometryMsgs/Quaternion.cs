using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{

    public struct Quaternion
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Quaternion";

        [JsonProperty("x")]
        public float X { get; set; }

        [JsonProperty("y")]
        public float Y { get; set; }

        [JsonProperty("z")]
        public float Z { get; set; }

        [JsonProperty("w")]
        public float W { get; set; }
    }

}