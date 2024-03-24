using Newtonsoft.Json;

namespace ROSBridge.SensorMsgs
{

    public struct NavSatFix
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "sensor_msgs/msg/NavSatFix";

        [JsonIgnore]
        public static readonly byte COVARIANCE_TYPE_UNKNOWN = 0;
        [JsonIgnore]
        public static readonly byte COVARIANCE_TYPE_APPROXIMATED = 1;
        [JsonIgnore]
        public static readonly byte COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
        [JsonIgnore]
        public static readonly byte COVARIANCE_TYPE_KNOWN = 3;

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("status")]
        public ROSBridge.SensorMsgs.NavSatStatus Status { get; set; }

        [JsonProperty("latitude")]
        public double Latitude { get; set; }

        [JsonProperty("longitude")]
        public double Longitude { get; set; }

        [JsonProperty("altitude")]
        public double Altitude { get; set; }

        [JsonProperty("position_covariance")]
        public double[] PositionCovariance { get; set; }

        [JsonProperty("position_covariance_type")]
        public byte PositionCovarianceType { get; set; }
    }

}