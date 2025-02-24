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
        public ROS() {
            // Start the connection loop if it's not already running.
            instances.Add(this);
            // Debug.Log("Added to instances. Size=" + instances.Count);
            if (instances.Count == 1)
            {
                _ = ConnectAndLoop("localhost:9090");
            }
        }

        public void Close()
        {
            // If already closed.
            if (closed)
            {
                return;
            }
            closed = true;

            // Unadvertise all publishers.
            if (socket != null && socket.State == WebSocketState.Open)
            {
                foreach (string topic in publishedTopics)
                {
                    _ = Send(new
                    {
                        op = "unadvertise",
                        topic = topic
                    });
                }

                // Unadvertise all services.
                foreach (string service in services.Keys)
                {
                    _ = Send(new
                    {
                        op = "unadvertise_service",
                        service = service
                    });
                }
            }

            // Remove this instance.
            instances.Remove(this);
            // Debug.Log("Removed from instances. Size=" + instances.Count);
            if (instances.Count == 0) {
                if (socket != null) {
                    try {
                        _ = Task.Run(async () =>
                        {
                            await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", CancellationToken.None);
                            socket.Dispose();
                        });
                    } catch {}
                    socket = null;
                }
                Debug.Log("ROSBridge connection was closed.");
            }
        }

        public void CreateSubscription<T>(string topic, Action<T> callback)
        {
            string type = typeof(T).GetField("ROSMessageType").GetValue(null).ToString();

            // Remember the callback.
            _subscribers[topic] = (JObject msg) => callback(msg.ToObject<T>());
            activeSubscriptions.Add(new AdvertisementStatus { name = topic, type = type, advertised = false });
        }

        public Publisher<T> CreatePublisher<T>(string topic)
        {
            string type = typeof(T).GetField("ROSMessageType").GetValue(null).ToString();

            // Save the publisher.
            publishedTopics.Add(topic);
            activePublishers.Add(new AdvertisementStatus { name = topic, type = type, advertised = false });

            // Return a publisher object.
            return new Publisher<T>(this, topic);
        }

        public void CreateService<Srv, Req, Res>(string service, Action<Req, Res> callback)
        {
            string type = typeof(Srv).GetField("ROSServiceType").GetValue(null).ToString();

            // Remember the callback.
            services[service] = (JObject req, JObject res) => callback(req.ToObject<Req>(), res.ToObject<Res>());
            activeServices.Add(new AdvertisementStatus { name = service, type = type, advertised = false });
        }

        internal static async Task Send<T>(T msg)
        {
            // Wait for the connection to be established.
            await connectionEstablished.Task;

            // If connection is still open.
            if (socket == null || socket.State != WebSocketState.Open)
            {
                return;
            }

            // Serialize the message.
            string str = JsonConvert.SerializeObject(msg);
            // Replace "NaN" with NaN.
            str = str.Replace("\"NaN\"", "NaN");
            var encoded = Encoding.UTF8.GetBytes(str);

            // Send it.
            try {
                await socket.SendAsync(encoded, WebSocketMessageType.Text, true, CancellationToken.None);
            } catch (Exception) {
                // If the connection is closed, continue without sending.
            }
        }

        private struct AdvertisementStatus
        {
            public string name;
            public string type;
            public bool advertised;
        }

        private static readonly float RECONNECTION_DELAY = 1.0F;
        private static readonly int RECEIVE_TIMEOUT_MS = 1000; 

        private static List<ROS> instances = new List<ROS>();
        private static ClientWebSocket socket;
        private static TaskCompletionSource<bool> connectionEstablished = new TaskCompletionSource<bool>();
        private static CancellationTokenSource receiveCts = new CancellationTokenSource();

        private static async Task ConnectAndLoop(string host)
        {
            while (instances.Count > 0)
            {
                try
                {
                    // Reset connection flag and cancellation token
                    connectionEstablished = new TaskCompletionSource<bool>();
                    receiveCts = new CancellationTokenSource();

                    // Connect to the server
                    bool connected = false;
                    try
                    {
                        socket = new ClientWebSocket();
                        await socket.ConnectAsync(new Uri($"ws://{host}"), CancellationToken.None);
                        connected = true;
                    }
                    catch (Exception ex)
                    {
                        socket = null;
                        Debug.LogWarning($"Failed to connect to ROSBridge server at {host}: {ex.Message}");
                        await Task.Delay(TimeSpan.FromSeconds(RECONNECTION_DELAY));
                        continue;
                    }

                    if (!connected)
                    {
                        socket = null;
                        continue;
                    }

                    Debug.Log("Connected to ROSBridge server.");
                    connectionEstablished.SetResult(true);
                    _ = ReadvertiseAll();

                    // Start the connection loop
                    // Debug.Log("Starting connection loop...");
                    while (instances.Count > 0)
                    {
                        // Debug.Log("Connection loop...");
                        try
                        {
                            // Advertise new publishers and subscribers
                            await ReadvertiseAll(false);

                            using (var ms = new MemoryStream())
                            {
                                var buffer = new ArraySegment<byte>(new byte[4096]);
                                WebSocketReceiveResult result = null;
                                bool messageComplete = false;

                                // Start a task that will complete when the connection is lost
                                var receiveTask = Task.Run(async () =>
                                {
                                    try
                                    {
                                        while (!messageComplete && socket.State == WebSocketState.Open)
                                        {
                                            using (var timeoutCts = new CancellationTokenSource(RECEIVE_TIMEOUT_MS))
                                            {
                                                try
                                                {
                                                    // Debug.Log("Receiving message...");
                                                    result = await socket.ReceiveAsync(buffer, timeoutCts.Token);
                                                    
                                                    if (result.CloseStatus.HasValue)
                                                    {
                                                        throw new WebSocketException($"Server closed connection: {result.CloseStatus}");
                                                    }

                                                    ms.Write(buffer.Array, buffer.Offset, result.Count);
                                                    messageComplete = result.EndOfMessage;
                                                }
                                                catch (OperationCanceledException)
                                                {
                                                    // Check socket state
                                                    if (socket.State != WebSocketState.Open && socket.State != WebSocketState.Aborted)
                                                    {
                                                        // Log the state
                                                        Debug.LogWarning($"WebSocket state: {socket.State}");
                                                        throw new WebSocketException("WebSocket connection lost");
                                                    }
                                                }
                                                finally {
                                                    // Debug.Log("Finished receiving message.");
                                                }
                                            }
                                        }
                                    }
                                    catch
                                    {
                                        // Ensure the socket is closed on any error
                                        messageComplete = true;
                                        throw;
                                    }
                                });

                                // Wait for the receive task with a timeout
                                try
                                {
                                    await receiveTask;

                                    // Process the received message if we got one
                                    if (result != null && result.MessageType == WebSocketMessageType.Text && ms.Length > 0)
                                    {
                                        await ProcessWebSocketMessage(ms);
                                    }
                                }
                                catch (Exception ex)
                                {
                                    Debug.LogError($"Error receiving message: {ex.Message}");
                                    break; // Exit the connection loop to trigger reconnect
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"Error in connection loop: {ex.Message}");
                            break;
                        }
                    }
                    // Debug.Log("Connection loop ended.");
                }
                finally
                {
                    // Clean up the socket
                    if (socket != null)
                    {
                        try
                        {
                            if (socket.State == WebSocketState.Open)
                            {
                                await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", CancellationToken.None);
                            }
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"Error closing socket: {ex.Message}");
                        }
                        finally
                        {
                            socket.Dispose();
                            socket = null;
                        }
                    }

                    // Wait before attempting to reconnect
                    await Task.Delay(TimeSpan.FromSeconds(RECONNECTION_DELAY));
                }
            }
        }

        private static async Task ReadvertiseAll(bool force = true)
        {
            foreach (ROS instance in instances)
            {
                for (int i = 0; i < instance.activeSubscriptions.Count; i++)
                {
                    var sub = instance.activeSubscriptions[i];
                    if (force || !sub.advertised)
                    {
                        await Send(new
                        {
                            op = "subscribe",
                            topic = sub.name,
                            type = sub.type
                        });
                        sub.advertised = true;
                        instance.activeSubscriptions[i] = sub;
                    }
                }
                for (int i = 0; i < instance.activePublishers.Count; i++)
                {
                    var pub = instance.activePublishers[i];
                    if (force || !pub.advertised)
                    {
                        await Send(new
                        {
                            op = "advertise",
                            topic = pub.name,
                            type = pub.type
                        });
                        pub.advertised = true;
                        instance.activePublishers[i] = pub;
                    }
                }
                for (int i = 0; i < instance.activeServices.Count; i++)
                {
                    var srv = instance.activeServices[i];
                    if (force || !srv.advertised)
                    {
                        await Send(new
                        {
                            op = "advertise_service",
                            service = srv.name,
                            type = srv.type
                        });
                        srv.advertised = true;
                        instance.activeServices[i] = srv;
                    }
                }
            }
        }

        private static async Task ProcessWebSocketMessage(MemoryStream ms)
        {
            ms.Seek(0, SeekOrigin.Begin);
            using (var reader = new StreamReader(ms, Encoding.UTF8))
            {
                JObject json = JObject.Parse(reader.ReadToEnd());
                string op = (string)json["op"];

                // Handle publish messages
                if (op == "publish")
                {
                    string topic = (string)json["topic"];
                    JObject msg = (JObject)json["msg"];

                    foreach (ROS instance in instances)
                    {
                        if (instance._subscribers.ContainsKey(topic))
                        {
                            instance._subscribers[topic]?.Invoke(msg);
                        }
                    }
                }
                // Handle service calls
                else if (op == "call_service")
                {
                    string service = (string)json["service"];
                    JObject req = (JObject)json["args"];
                    JObject res = new JObject();

                    foreach (ROS instance in instances)
                    {
                        if (instance.services.ContainsKey(service))
                        {
                            instance.services[service]?.Invoke(req, res);
                        }
                    }

                    await Send(new
                    {
                        op = "service_response",
                        id = (string)json["id"],
                        service = service,
                        values = res,
                        result = true
                    });
                }
            }
        }

        private bool closed = false;
        private readonly Dictionary<string, Action<JObject>> _subscribers = new Dictionary<string, Action<JObject>>();
        private readonly List<string> publishedTopics = new List<string>();
        private readonly Dictionary<string, Action<JObject, JObject>> services = new Dictionary<string, Action<JObject, JObject>>();
        private readonly List<AdvertisementStatus> activePublishers = new List<AdvertisementStatus>();
        private readonly List<AdvertisementStatus> activeSubscriptions = new List<AdvertisementStatus>();
        private readonly List<AdvertisementStatus> activeServices = new List<AdvertisementStatus>();
    }
}