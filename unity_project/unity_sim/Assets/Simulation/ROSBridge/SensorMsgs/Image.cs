using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct Image
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/Image";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("height")]
        public uint Height { get; set; }

        [JsonProperty("width")]
        public uint Width { get; set; }

        [JsonProperty("encoding")]
        public string Encoding { get; set; }

        [JsonProperty("is_bigendian")]
        public byte IsBigEndian { get; set; }

        [JsonProperty("step")]
        public uint Step { get; set; }

        [JsonProperty("data")]
        public byte[] Data { get; set; }
    }

}