using Newtonsoft.Json;

namespace ROSBridge.KalmanInterfaces
{
    public struct WheelTemperature
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "kalman_interfaces/msg/WheelTemperature";

        [JsonProperty("motor")]
        public float Motor { get; set; }

        [JsonProperty("swivel")]
        public float Swivel { get; set; }
    }
}
