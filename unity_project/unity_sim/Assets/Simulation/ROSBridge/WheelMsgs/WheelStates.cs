using Newtonsoft.Json;

namespace ROSBridge.WheelMsgs
{
    public struct WheelStates
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "kalman_interfaces/msg/WheelStates";

        [JsonProperty("front_left")]
        public WheelState FrontLeft { get; set; }

        [JsonProperty("front_right")]
        public WheelState FrontRight { get; set; }

        [JsonProperty("back_left")]
        public WheelState BackLeft { get; set; }

        [JsonProperty("back_right")]
        public WheelState BackRight { get; set; }
    }
}
