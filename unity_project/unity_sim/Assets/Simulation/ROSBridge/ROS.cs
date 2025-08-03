using System;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Collections.Concurrent;
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

        internal struct AdvertisementStatus
        {
            public string name;
            public string type;
            public bool advertised;
        }

        internal readonly List<AdvertisementStatus> activePublishers = new List<AdvertisementStatus>();
        internal readonly List<AdvertisementStatus> activeSubscriptions = new List<AdvertisementStatus>();
        internal readonly List<AdvertisementStatus> activeServices = new List<AdvertisementStatus>();

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

        private const int RECONNECTION_DELAY = 2; // Seconds between reconnection attempts
        private const int HEARTBEAT_INTERVAL = 5000; // Milliseconds between connection checks
        private const int MAX_MESSAGE_SIZE = 1024 * 1024; // 1MB max message size

        private static List<ROS> instances = new List<ROS>();
        private static ClientWebSocket socket;
        private static TaskCompletionSource<bool> connectionEstablished = new TaskCompletionSource<bool>();
        private static CancellationTokenSource connectionCts;
        private static ConcurrentQueue<MemoryStream> messageQueue = new ConcurrentQueue<MemoryStream>();

        private static async Task ConnectAndLoop(string host)
    {
        while (instances.Count > 0)
        {
            try
            {
                // Reset connection state
                connectionEstablished = new TaskCompletionSource<bool>();
                connectionCts = new CancellationTokenSource();
                
                // Clear message queue from previous connection
                while (messageQueue.TryDequeue(out var _)) { }

                // Connect to the server
                if (!await TryConnectToServer(host))
                {
                    await Task.Delay(TimeSpan.FromSeconds(RECONNECTION_DELAY));
                    continue;
                }

                Debug.Log("Connected to ROSBridge server.");
                connectionEstablished.SetResult(true);
                await ReadvertiseAll();

                // Start the message receiver and connection monitor tasks
                var receiverTask = StartMessageReceiver(connectionCts.Token);
                var monitorTask = StartConnectionMonitor(connectionCts.Token);
                var processingTask = ProcessMessageQueue(connectionCts.Token);

                // Wait for any task to fail, which would indicate connection issues
                await Task.WhenAny(receiverTask, monitorTask, processingTask);
                
                // If any task completes, it means there was a connection issue
                if (instances.Count > 0) {
                    Debug.LogWarning("WebSocket connection issue detected, will attempt to reconnect...");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error in connection loop: {ex.Message}");
            }
            finally
            {
                // Clean up
                await CleanupConnection();
                await Task.Delay(TimeSpan.FromSeconds(RECONNECTION_DELAY));
            }
        }
    }

    private static async Task<bool> TryConnectToServer(string host)
    {
        try
        {
            socket = new ClientWebSocket();
            await socket.ConnectAsync(new Uri($"ws://{host}"), CancellationToken.None);
            return true;
        }
        catch (Exception ex)
        {
            socket = null;
            Debug.LogWarning($"Failed to connect to ROSBridge server at {host}: {ex.Message}");
            return false;
        }
    }

    private static async Task StartMessageReceiver(CancellationToken cancellationToken)
    {
        var buffer = new byte[MAX_MESSAGE_SIZE];
        
        try
        {
            while (!cancellationToken.IsCancellationRequested && socket != null && socket.State == WebSocketState.Open)
            {
                using (var ms = new MemoryStream())
                {
                    var receiveBuffer = new ArraySegment<byte>(buffer);
                    WebSocketReceiveResult result = null;
                    
                    // Receive the complete message
                    do
                    {
                        try
                        {
                            result = await socket.ReceiveAsync(receiveBuffer, cancellationToken);
                            
                            if (result.CloseStatus.HasValue)
                            {
                                Debug.LogWarning($"Server closed WebSocket connection: {result.CloseStatus} - {result.CloseStatusDescription}");
                                return;
                            }
                            
                            ms.Write(receiveBuffer.Array, receiveBuffer.Offset, result.Count);
                        }
                        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
                        {
                            return; // Clean cancellation
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"Error receiving WebSocket message: {ex.Message}");
                            return;
                        }
                    }
                    while (!result.EndOfMessage);
                    
                    // If message is complete and it's a text message
                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        // Reset position and add to queue for processing
                        ms.Position = 0;
                        var messageCopy = new MemoryStream();
                        await ms.CopyToAsync(messageCopy);
                        messageCopy.Position = 0;
                        messageQueue.Enqueue(messageCopy);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Fatal error in message receiver: {ex.Message}");
        }
        // finally
        // {
        //     // Signal that the receiver has stopped
        //     Debug.Log("Message receiver stopped");
        // }
    }

    private static async Task StartConnectionMonitor(CancellationToken cancellationToken)
    {
        try
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                // Check connection state
                if (socket == null || socket.State != WebSocketState.Open)
                {
                    Debug.LogWarning($"WebSocket state check failed: {(socket != null ? socket.State.ToString() : "null")}");
                    return; // Exit to trigger reconnection
                }
                
                // Wait for the next check
                await Task.Delay(HEARTBEAT_INTERVAL, cancellationToken);
            }
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
            // Expected when cancelling
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error in connection monitor: {ex.Message}");
        }
    }

    private static async Task ProcessMessageQueue(CancellationToken cancellationToken)
    {
        try
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                bool receivedAnything = false;
                int numMsgs = 0;
                // Process all available messages
                while (messageQueue.TryDequeue(out var messageStream))
                {
                    receivedAnything = true;
                    numMsgs++;
                    try
                    {
                        await ProcessWebSocketMessage(messageStream);
                    }
                    catch (Exception ex)
                    {
                        Debug.LogError($"Error processing message: {ex.Message}");
                    }
                    finally
                    {
                        messageStream.Dispose();
                    }
                }
                
                // Small delay to avoid tight loop
                if (!receivedAnything)
                {
                    await Task.Delay(1, cancellationToken);
                }
            }
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
            // Expected when cancelling
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error in message processor: {ex.Message}");
        }
    }

    private static async Task CleanupConnection()
    {
        // Cancel ongoing operations
        try
        {
            connectionCts?.Cancel();
        }
        catch { /* Ignore */ }

        // Clean up the socket
        if (socket != null)
        {
            try
            {
                if (socket.State == WebSocketState.Open)
                {
                    var closeToken = new CancellationTokenSource(5000).Token;
                    await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", closeToken);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Error closing socket: {ex.Message}");
            }
            finally
            {
                socket.Dispose();
                socket = null;
            }
        }

        // Mark all subscriptions, publishers and services as unadvertised
        foreach (ROS instance in instances)
        {
            for (int i = 0; i < instance.activeSubscriptions.Count; i++)
            {
                var sub = instance.activeSubscriptions[i];
                sub.advertised = false;
                instance.activeSubscriptions[i] = sub;
            }
            for (int i = 0; i < instance.activePublishers.Count; i++)
            {
                var pub = instance.activePublishers[i];
                pub.advertised = false;
                instance.activePublishers[i] = pub;
            }
            for (int i = 0; i < instance.activeServices.Count; i++)
            {
                var srv = instance.activeServices[i];
                srv.advertised = false;
                instance.activeServices[i] = srv;
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
    }
}