using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct CameraInfo
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/CameraInfo";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("height")]
        public uint Height { get; set; }

        [JsonProperty("width")]
        public uint Width { get; set; }

        [JsonProperty("distortion_model")]
        public string DistortionModel { get; set; }

        [JsonProperty("d")]
        public double[] D { get; set; }

        [JsonProperty("k")]
        public double[] K { get; set; }

        [JsonProperty("r")]
        public double[] R { get; set; }

        [JsonProperty("p")]
        public double[] P { get; set; }

        [JsonProperty("binning_x")]
        public uint BinningX { get; set; }

        [JsonProperty("binning_y")]
        public uint BinningY { get; set; }

        [JsonProperty("roi")]
        public ROSBridge.SensorMsgs.RegionOfInterest ROI { get; set; }
    }

}