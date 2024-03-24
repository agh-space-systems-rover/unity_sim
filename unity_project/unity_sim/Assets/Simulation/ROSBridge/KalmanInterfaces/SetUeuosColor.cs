using Newtonsoft.Json;
using ROSBridge.StdMsgs;

namespace ROSBridge.KalmanInterfaces
{
    public struct SetUeuosColor
    {
        [JsonIgnore]
        public const string ROSServiceType = "kalman_interfaces/srv/SetUeuosColor";

        public struct Request
        {
            [JsonProperty("color")]
            public ColorRGBA Color { get; set; }
        }

        public struct Response
        {
            // no response
        }
    }
}
