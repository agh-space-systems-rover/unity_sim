using Newtonsoft.Json;

namespace ROSBridge.UnityRSPublisherMsgs
{
    public struct CameraMetadata
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "unity_rs_publisher_msgs/msg/CameraMetadata";

        [JsonProperty("width")]
        public int Width { get; set; }

        [JsonProperty("height")]
        public int Height { get; set; }

        [JsonProperty("depth_min")]
        public float DepthMin { get; set; }

        [JsonProperty("depth_max")]
        public float DepthMax { get; set; }

        [JsonProperty("fx")]
        public float Fx { get; set; }

        [JsonProperty("fy")]
        public float Fy { get; set; }

        [JsonProperty("cx")]
        public float Cx { get; set; }

        [JsonProperty("cy")]
        public float Cy { get; set; }
    }
}
