using Newtonsoft.Json;
using ROSBridge.StdMsgs;

namespace ROSBridge.KalmanInterfaces
{
    public struct SetUeuosEffect
    {
        [JsonIgnore]
        public const string ROSServiceType = "kalman_interfaces/srv/SetUeuosEffect";

        public struct Request
        {
            [JsonIgnore]
            public const byte BOOT = 0;
            [JsonIgnore]
            public const byte RAINBOW = 1;

            [JsonProperty("effect")]
            public byte Effect { get; set; }
        }

        public struct Response
        {
            // no response
        }
    }
}
