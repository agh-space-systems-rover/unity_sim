using Newtonsoft.Json;
using ROSBridge.StdMsgs;

namespace ROSBridge.KalmanInterfaces
{
    public struct SetUeuosMode
    {
        [JsonIgnore]
        public const string ROSServiceType = "kalman_interfaces/srv/SetUeuosMode";

        public struct Request
        {
            [JsonIgnore]
            public const byte OFF = 0;
            [JsonIgnore]
            public const byte AUTONOMY = 1;
            [JsonIgnore]
            public const byte TELEOP = 2;
            [JsonIgnore]
            public const byte FINISHED = 3;

            [JsonProperty("mode")]
            public byte Mode { get; set; }
        }

        public struct Response
        {
            // no response
        }
    }
}
