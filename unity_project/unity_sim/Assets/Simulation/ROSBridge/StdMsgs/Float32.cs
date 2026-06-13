using Newtonsoft.Json;

namespace ROSBridge.StdMsgs
{
    public struct Float32
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "std_msgs/msg/Float32";

        [JsonProperty("data")]
        public float Data { get; set; }
    }
}