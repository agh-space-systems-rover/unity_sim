using Newtonsoft.Json;

namespace ROSBridge.KalmanInterfaces
{
    public struct WheelTemperatures
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "kalman_interfaces/msg/WheelTemperatures";

        [JsonProperty("front_left")]
        public WheelTemperature FrontLeft { get; set; }

        [JsonProperty("front_right")]
        public WheelTemperature FrontRight { get; set; }

        [JsonProperty("back_left")]
        public WheelTemperature BackLeft { get; set; }

        [JsonProperty("back_right")]
        public WheelTemperature BackRight { get; set; }
    }
}
