using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct CompressedImage
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/CompressedImage";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("format")]
        public string Format { get; set; }

        [JsonProperty("data")]
        public byte[] Data { get; set; }
    }

}