using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct RegionOfInterest
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/RegionOfInterest";

        [JsonProperty("x_offset")]
        public uint XOffset { get; set; }

        [JsonProperty("y_offset")]
        public uint YOffset { get; set; }

        [JsonProperty("height")]
        public uint Height { get; set; }

        [JsonProperty("width")]
        public uint Width { get; set; }

        [JsonProperty("do_rectify")]
        public bool DoRectify { get; set; }
    }

}