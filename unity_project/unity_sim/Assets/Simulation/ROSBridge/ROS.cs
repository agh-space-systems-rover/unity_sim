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
        internal readonly List<string> publishedTopics = new List<string>();
        internal bool closed = false;

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

            // Save the publisher.
            publishedTopics.Add(topic);

            // Return a publisher object.
            return new Publisher<T>(this, topic);
        }

        public async Task Close()
        {
            // If already closed.
            if (closed)
            {
                return;
            }
            closed = true;

            // Wait for the connection to be established.
            await connectionEstablished.Task;

            // Unadvertise all publishers.
            foreach (string topic in publishedTopics)
            {
                await Send(new
                {
                    op = "unadvertise",
                    topic = topic
                });
            }

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

                    // Deserialize the WebSocket message.
                    ms.Seek(0, SeekOrigin.Begin);
                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        using (var reader = new StreamReader(ms, Encoding.UTF8))
                        {
                            JObject json = JObject.Parse(reader.ReadToEnd());
                            string op = (string)json["op"];

                            // Invoke subscribers if a ROS message was published.
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
    }

}