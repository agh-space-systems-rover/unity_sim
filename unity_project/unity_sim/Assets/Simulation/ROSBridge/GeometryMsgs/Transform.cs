using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct Transform
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/Transform";

        [JsonProperty("translation")]
        public Vector3 Translation { get; set; }

        [JsonProperty("rotation")]
        public Quaternion Rotation { get; set; }
    }
}
