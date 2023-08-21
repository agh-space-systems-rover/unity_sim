using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Unity.Collections;
using ROSBridge;
using OperationCanceledException = System.OperationCanceledException;
using System.Threading.Tasks;
using System;

public class RealSense : MonoBehaviour
{
    private static readonly int Width = 640;
    private static readonly int Height = 480;
    private static readonly int FPS = 10;
    private static readonly float DepthMin = 0.5F;
    private static readonly float DepthMax = 16.0F;

    [SerializeField]
    private string cameraId = "camera";

    private RenderTexture depthTexture;
    private Texture2D depthTexture2D;
    private Material depthMaterial;
    private Camera cam;

    ROS ros;
    Publisher<ROSBridge.SensorMsgs.CompressedImage> colorPublisher, depthPublisher;
    Publisher<ROSBridge.SensorMsgs.CameraInfo> colorInfoPublisher, depthInfoPublisher;

    RenderTexture depthRt;
    NativeArray<byte> buffer, depthBuffer;

    float lastCaptureTime = 0.0F;

    // Start is called before the first frame update
    private async void Start()
    {
        // camera settings
        cam = GetComponent<Camera>();
        cam.enabled = false;
        cam.depthTextureMode = DepthTextureMode.Depth;
        cam.targetTexture = new RenderTexture(Width, Height, 0, RenderTextureFormat.ARGB32);
        cam.nearClipPlane = 0.01F;
        cam.farClipPlane = 3000;

        // depth shader
        Shader depthShader = Shader.Find("Custom/DepthTextureShader");
        depthMaterial = new Material(depthShader);
        depthMaterial.SetFloat("_DepthMin", DepthMin);
        depthMaterial.SetFloat("_DepthMax", DepthMax);
        depthMaterial.SetFloat("_DepthRange", cam.farClipPlane - cam.nearClipPlane);

        // ROS
        ros = new ROS();
        colorPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CompressedImage>("/" + cameraId + "/color/image_raw/compressed");
        depthPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CompressedImage>("/" + cameraId + "/depth/image_raw/compressed");
        colorInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + cameraId + "/color/camera_info");
        depthInfoPublisher = await ros.CreatePublisher<ROSBridge.SensorMsgs.CameraInfo>("/" + cameraId + "/depth/camera_info");

        // async capture
        depthRt = new RenderTexture(Width, Height, 0);
        buffer = new NativeArray<byte>(Width * Height * 3, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        depthBuffer = new NativeArray<byte>(Width * Height * 3, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        try
        {
            for (var cancel = destroyCancellationToken; ;)
            {

                // Interval (cancellable)
                await Awaitable.EndOfFrameAsync(cancel);

                // Throw if the cancellation is requested.
                cancel.ThrowIfCancellationRequested();

                // Read the depth data from the texture
                // RenderTexture.active = depthTexture;
                // depthTexture2D.ReadPixels(new Rect(0, 0, DepthWidth, DepthHeight), 0, 0);
                // NativeArray<byte> bytes = depthTexture2D.GetRawTextureData<byte>();

                try
                {
                    await AsyncGPUReadback.RequestIntoNativeArrayAsync(ref buffer, cam.targetTexture, 0, TextureFormat.RGB24);
                    await AsyncGPUReadback.RequestIntoNativeArrayAsync(ref depthBuffer, depthRt, 0, TextureFormat.RGB24);
                }
                catch (MissingReferenceException)
                {
                    // Rules are meant to be broken.
                    continue;
                }

                // Fetch current time
                DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                TimeSpan span = (DateTime.UtcNow - epochStart);
                int sec = (int)span.TotalSeconds;
                uint nanosec = (uint)span.Milliseconds * 1000;

                // Compose the header
                var header = new ROSBridge.StdMsgs.Header
                {
                    Stamp = new ROSBridge.BuiltinInterfaces.Time
                    {
                        Sec = sec,
                        Nanosec = nanosec
                    },
                    FrameId = cameraId
                };

                // begin background thread.
                await Awaitable.BackgroundThreadAsync();

                // Encode the color data as JPEG.
                NativeArray<byte> jpg = ImageConversion.EncodeNativeArrayToJPG(buffer, GraphicsFormat.R8G8B8_SRGB, (uint)Width, (uint)Height, 0, 80);

                // Only this thread can access ROS websocket.
                await colorPublisher.Publish(
                    new ROSBridge.SensorMsgs.CompressedImage
                    {
                        Header = header,
                        Format = "jpeg",
                        Data = jpg.ToArray()
                    }
                );

                jpg.Dispose();

                // Encode depth as JPG
                NativeArray<byte> depthJpg = ImageConversion.EncodeNativeArrayToJPG(depthBuffer, GraphicsFormat.R8G8B8_SRGB, (uint)Width, (uint)Height, 0, 80);

                // Only this thread can access ROS websocket.
                await depthPublisher.Publish(
                    new ROSBridge.SensorMsgs.CompressedImage
                    {
                        Header = header,
                        Format = "jpeg",
                        Data = depthJpg.ToArray()
                    }
                );

                depthJpg.Dispose();

                // Publish color info
                await colorInfoPublisher.Publish(
                    new ROSBridge.SensorMsgs.CameraInfo
                    {
                        Header = header,
                        Height = (uint)Height,
                        Width = (uint)Width,
                        DistortionModel = "plumb_bob",
                        D = new double[] { 0, 0, 0, 0, 0 },
                        K = new double[] { 283.119, 0, 320.5, 0, 283.119, 240.5, 0, 0, 1 },
                        R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                        P = new double[] { 283.119, 0, 320.5, -19.818, 0, 283.119, 240.5, 0, 0, 0, 1, 0 },
                        BinningX = 0,
                        BinningY = 0,
                        ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                        {
                            XOffset = 0,
                            YOffset = 0,
                            Height = 0,
                            Width = 0,
                            DoRectify = false
                        }
                    }
                );

                // Publish depth info
                await colorInfoPublisher.Publish(
                    new ROSBridge.SensorMsgs.CameraInfo
                    {
                        Header = header,
                        Height = (uint)Height,
                        Width = (uint)Width,
                        DistortionModel = "plumb_bob",
                        D = new double[] { 0, 0, 0, 0, 0 },
                        K = new double[] { 283.119, 0, 320.5, 0, 283.119, 240.5, 0, 0, 1 },
                        R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                        P = new double[] { 283.119, 0, 320.5, 0, 0, 283.119, 240.5, 0, 0, 0, 1, 0 },
                        BinningX = 0,
                        BinningY = 0,
                        ROI = new ROSBridge.SensorMsgs.RegionOfInterest
                        {
                            XOffset = 0,
                            YOffset = 0,
                            Height = 0,
                            Width = 0,
                            DoRectify = false
                        }
                    }
                );

                await Awaitable.MainThreadAsync();
            }
        }
        catch (OperationCanceledException)
        {
        }
        finally
        {
            await Awaitable.MainThreadAsync();
            // Note: We should clean the resources up very carefully because this
            // block can be run after OnDestroy; Unity objects would have been
            // recycled by the engine.
            if (buffer.IsCreated) buffer.Dispose();
            if (depthBuffer.IsCreated) depthBuffer.Dispose();
        }
    }

    private void Update()
    {
        if (colorPublisher == null || depthPublisher == null)
        {
            return;
        }

        if (Time.time - lastCaptureTime < 1.0F / FPS)
        {
            return;
        }
        lastCaptureTime = Time.time;

        cam.Render();
    }

    private void OnEnable()
    {
        RenderPipelineManager.endCameraRendering += RenderPipelineManager_endCameraRendering;
    }

    private void OnDisable()
    {
        RenderPipelineManager.endCameraRendering -= RenderPipelineManager_endCameraRendering;
    }

    private void RenderPipelineManager_endCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        if (camera == cam)
        {
            Graphics.Blit(cam.targetTexture, depthRt, depthMaterial);
        }
    }

    private async void OnApplicationQuit()
    {
        await ros.Close();
    }
}
