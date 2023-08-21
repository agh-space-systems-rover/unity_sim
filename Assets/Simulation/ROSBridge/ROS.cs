using System;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Net.WebSockets;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace ROSBridge
{

    public class ROS
    {
        internal static readonly int reconnectionAttempts = 5;
        internal static readonly float reconnectionDelay = 1.0F;

        internal ClientWebSocket socket;
        internal TaskCompletionSource<bool> connectionEstablished = new TaskCompletionSource<bool>();
        internal readonly Dictionary<string, Action<JObject>> _subscribers = new Dictionary<string, Action<JObject>>();
        // private readonly var _servers = new Dictionary<string, Func<object, object>>();
        // private readonly var _callInstances = new Dictionary<string, CallInstance>();
        internal int callIdCounter = 0;

        public ROS(string host = "localhost:9090")
        {
            _ = ConnectAndLoop(host);
        }

        ~ROS()
        {
            _ = Close();
        }

        public async Task CreateSubscription<T>(string topic, Action<T> callback)
        {
            // Send a subscription request.
            await Send(new
            {
                op = "subscribe",
                topic = topic,
                type = typeof(T).GetField("ROSMessageType").GetValue(null)
            });

            // Remember the callback.
            _subscribers[topic] = (JObject msg) => callback(msg.ToObject<T>());
        }

        public async Task<Publisher<T>> CreatePublisher<T>(string topic)
        {
            // Advertise the publisher.
            await Send(new
            {
                op = "advertise",
                topic = topic,
                type = typeof(T).GetField("ROSMessageType").GetValue(null)
            });

            // Return a publisher object.
            return new Publisher<T>(this, topic);
        }

        // public void Serve(string service, Func<object, object> callback)
        // {
        //     _ = ServeAsync(service, callback);
        // }

        // public async Task ServeAsync(string service, Func<object, object> callback)
        // {
        //     // Wait for the connection to be established.
        //     await connectionEstablished.Task;

        //     // Send a service registration request.
        //     await SendAsync(CreateCborMessage(new Dictionary<string, object> { ["serve"] = service }));

        //     // Remember the callback.
        //     _servers[service] = callback;
        // }

        // public ServiceCall Call(string service)
        // {
        //     // Normalize service ID.
        //     service = service.Trim('/');

        //     // Return a call object.
        //     return new ServiceCall(this, service);
        // }

        public async Task Close()
        {
            // Wait for the connection to be established.
            await connectionEstablished.Task;

            await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", CancellationToken.None);
            socket.Dispose();
        }

        private async Task ConnectAndLoop(string host)
        {
            // Read server host from environment variable and fall back to default.
            // Connect to the server.
            socket = new ClientWebSocket();
            for (int attempt = 0; attempt < reconnectionAttempts; attempt++)
            {
                try
                {
                    await socket.ConnectAsync(new Uri($"ws://{host}"), CancellationToken.None);
                    break;
                }
                catch
                {
                    await Task.Delay(TimeSpan.FromSeconds(reconnectionDelay));
                }
            }

            if (socket.State != WebSocketState.Open)
            {
                throw new InvalidOperationException($"Failed to connect to the server at {host}.");
            }

            // Set the flag.
            connectionEstablished.SetResult(true);

            // Start the connection loop.
            while (socket.State == WebSocketState.Open)
            {
                using (var ms = new MemoryStream())
                {
                    // Receive the whole message.
                    var buffer = new ArraySegment<byte>(new byte[4096]);
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await socket.ReceiveAsync(buffer, CancellationToken.None);

                        // Handle close message.
                        if (result.CloseStatus.HasValue)
                        {
                            throw new InvalidOperationException("Connection closed.");
                        }

                        ms.Write(buffer.Array, buffer.Offset, result.Count);
                    }
                    while (!result.EndOfMessage);

                    ms.Seek(0, SeekOrigin.Begin);
                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        using (var reader = new StreamReader(ms, Encoding.UTF8))
                        {
                            JObject json = JObject.Parse(reader.ReadToEnd());
                            string op = (string)json["op"];

                            if (op == "publish")
                            {
                                string topic = (string)json["topic"];
                                JObject msg = (JObject)json["msg"];

                                _subscribers[topic]?.Invoke(msg);
                            }
                        }
                    }
                }
            }
        }

        // private async Task ProcessMessageAsync(CBORObject msg)
        // {
        //     if (msg.ContainsKey("publish") && msg.ContainsKey("msg"))
        //     {
        //         string topic = msg["publish"].AsString();
        //         CBORObject _msg = msg["msg"];

        //         _subscribers[topic]?.Invoke(_msg);
        //     }
        //     else if (msg.ContainsKey("call") && msg.ContainsKey("id") && msg.ContainsKey("req"))
        //     {
        //         string service = msg["call"].AsString();
        //         string id = msg["id"].AsString();
        //         object req = msg["req"];

        //         object res = _servers[service]?.Invoke(req);

        //         await SendAsync(CreateCborMessage(new Dictionary<string, object> { ["call"] = service, ["id"] = id, ["res"] = res }));
        //     }
        //     else if (msg.ContainsKey("call") && msg.ContainsKey("id") && msg.ContainsKey("res"))
        //     {
        //         string service = msg["call"].AsString();
        //         string id = msg["id"].AsString();
        //         CBORObject res = msg["res"];
        //         CBORObject error = msg.ContainsKey("error") ? msg["error"] : null;

        //         if (_callInstances.TryGetValue(id, out CallInstance call))
        //         {
        //             call.Res = res;
        //             call.Error = error;

        //             // Set the condition.
        //             call.Condition.SetResult(true);

        //             if (error == null)
        //             {
        //                 _callInstances.Remove(id);
        //             }
        //         }
        //     }
        //     else if (msg.ContainsKey("error"))
        //     {
        //         throw new InvalidOperationException(msg["error"].AsString());
        //     }
        //     else
        //     {
        //         throw new InvalidOperationException("Received invalid message from server.");
        //     }
        // }

        internal async Task Send<T>(T msg)
        {
            // Wait for the connection to be established.
            await connectionEstablished.Task;

            // If connection is still open.
            if (socket.State != WebSocketState.Open)
            {
                return;
            }

            // // Serialize the message.
            string str = JsonConvert.SerializeObject(msg);
            // Debug.Log("Sending: " + str);
            var encoded = Encoding.UTF8.GetBytes(str);

            // Send it.
            await socket.SendAsync(encoded, WebSocketMessageType.Text, true, CancellationToken.None);
        }


        // private async Task<CBORObject> SendCallAsync(string service, CBORObject req)
        // {
        //     string callId = _callIdCounter++.ToString();
        //     CallInstance call = new CallInstance();

        //     CBORObject callMsg = CBORObject.NewMap();
        //     callMsg.Add("call", service);
        //     callMsg.Add("id", callId);
        //     callMsg.Add("req", req);

        //     // Reset the condition.
        //     call.Condition = new TaskCompletionSource<bool>();

        //     _callInstances[callId] = call;
        //     await SendAsync(callMsg);

        //     // Wait for the condition to be set by the recv loop.
        //     await call.Condition.Task;

        //     if (call.Error != null)
        //     {
        //         throw new InvalidOperationException(call.Error.ToString());
        //     }

        //     return call.Res;
        // }

        // private class CallInstance
        // {
        //     public CBORObject Res { get; set; }
        //     public CBORObject Error { get; set; }
        //     public TaskCompletionSource<bool> Condition { get; set; }
        // }



        // public class ServiceCall
        // {
        //     private readonly Client _client;
        //     private readonly string _service;

        //     public ServiceCall(Client client, string service)
        //     {
        //         _client = client;
        //         _service = service;
        //     }

        //     public CBORObject Call(CBORObject req)
        //     {
        //         var task = CallAsync(req);
        //         Task.Run(async () => await task).Wait();
        //         return task.Result;
        //     }

        //     public async Task<CBORObject> CallAsync(CBORObject req)
        //     {
        //         // Wait for the connection to be established.
        //         await _client.connectionEstablished.Task;

        //         return await _client.SendCallAsync(_service, req);
        //     }
        // }
    }

}