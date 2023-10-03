using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct Twist
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Twist";

        [JsonProperty("linear")]
        public Vector3 Linear { get; set; }

        [JsonProperty("angular")]
        public Vector3 Angular { get; set; }
    }
}
