using Newtonsoft.Json;

namespace ROSBridge.GeometryMsgs
{
    public struct TwistWithCovariance
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "geometry_msgs/msg/TwistWithCovariance";

        [JsonProperty("twist")]
        public Twist Twist { get; set; }

        [JsonProperty("covariance")]
        public double[] Covariance { get; set; }
    }
}
