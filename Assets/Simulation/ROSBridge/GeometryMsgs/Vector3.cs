using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{

    public struct Vector3
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Vector3";

        [JsonProperty("x")]
        public float X { get; set; }

        [JsonProperty("y")]
        public float Y { get; set; }

        [JsonProperty("z")]
        public float Z { get; set; }
    }

}