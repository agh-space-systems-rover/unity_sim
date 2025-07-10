using Newtonsoft.Json;

namespace ROSBridge.KalmanInterfaces
{
    public struct ArmValues
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "kalman_interfaces/msg/ArmValues";

        [JsonProperty("joints")]
        public float[] Joints { get; set; }

        [JsonProperty("jaw")]
        public float Jaw { get; set; }
    }
}
