using Newtonsoft.Json;

namespace ROSBridge.StdMsgs
{
    public struct String
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "std_msgs/msg/String";

        [JsonProperty("data")]
        public string Data { get; set; }
    }
}