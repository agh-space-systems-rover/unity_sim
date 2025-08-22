using Newtonsoft.Json;
using ROSBridge.StdMsgs;

namespace ROSBridge.KalmanInterfaces
{
    public struct ArmValues
    {
        [JsonIgnore]
        public static readonly string ROSMessageType = "kalman_interfaces/msg/ArmValues";

        [JsonProperty("header")]
        public ROSBridge.StdMsgs.Header Header { get; set; }

        [JsonProperty("joints")]
        public float[] Joints { get; set; }

        [JsonProperty("jaw")]
        public float Jaw { get; set; }
    }
}
