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
using System.Net.WebSockets;
using System.Threading;
using System.Text;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine.Rendering.Universal;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEditor;

public class RealSense : MonoBehaviour
{
    [SerializeField]
    private int width = 640;
    [SerializeField]
    private int height = 360;
    [SerializeField]
    private int fps = 10;
    [SerializeField]
    private float vFov = 60.0F;
    [SerializeField]
    private float depthMin = 0.5F;
    [SerializeField]
    private float depthMax = 5.0F;
    [SerializeField]
    private string id = "camera";

    private Camera cam = null;
    private Camera currentRenderCam = null;
    private bool frameAvailable = false; // Feels a bit smoother than cam.Render()
    private ROSBridge.BuiltinInterfaces.Time frameStamp;
    private RenderTexture asyncRt = null;
    private Material depthMaterial = null;

    [DllImport("UnityRSPublisherPlugin")]
    private static extern void OpenBridgeConnection(string id, int width, int height, float vFov, float depthMin, float depthMax);
    [DllImport("UnityRSPublisherPlugin")]
    private static extern void CloseBridgeConnection(string id);
    [DllImport("UnityRSPublisherPlugin")]
    private static unsafe extern void TryPushFrame(string id, int sec, uint nanosec, void* ptr);

    private async void Start()
    {
        RenderTexture rt = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        rt.depthStencilFormat = GraphicsFormat.D24_UNorm_S8_UInt;

        // camera settings
        cam = GetComponent<Camera>();
        cam.enabled = true;
        cam.depthTextureMode = DepthTextureMode.Depth;
        cam.targetTexture = rt;
        cam.nearClipPlane = 0.01F;
        cam.farClipPlane = 3000;
        cam.fieldOfView = vFov;

        // --------
        // Matrices
        // --------

        Matrix4x4 rosOpticalToRosView = Matrix4x4.identity;
        rosOpticalToRosView.SetColumn(0, new Vector4(0, -1, 0, 0));
        rosOpticalToRosView.SetColumn(1, new Vector4(0, 0, -1, 0));
        rosOpticalToRosView.SetColumn(2, new Vector4(1, 0, 0, 0));

        Matrix4x4 rosViewToGlView = Matrix4x4.identity;
        //  0, -1, 0 // Ros +Y is GL -X (left) (second column)
        //  0,  0, 1 // Ros +Z is GL +Y (up) (third column)
        // -1,  0, 0 // Ros +X is GL -Z (forward) (first column)
        rosViewToGlView.SetColumn(0, new Vector4(0, 0, -1, 0));
        rosViewToGlView.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosViewToGlView.SetColumn(2, new Vector4(0, 1, 0, 0));

        Matrix4x4 glViewToGlNdc = cam.nonJitteredProjectionMatrix;
        Matrix4x4 glNDCToGlUv = Matrix4x4.identity; // the standard "* 0.5 + 0.5" transform
        glNDCToGlUv.SetRow(0, new Vector4(0.5F, 0, 0, 0.5F));
        glNDCToGlUv.SetRow(1, new Vector4(0, 0.5F, 0, 0.5F));
        glNDCToGlUv.SetRow(2, new Vector4(0, 0, 0.5F, 0.5F));

        Matrix4x4 glUvToPixel = Matrix4x4.identity; // scale by screen size, flip Y
        glUvToPixel.SetRow(0, new Vector4(width, 0, 0, 0));
        glUvToPixel.SetRow(1, new Vector4(0, -height, 0, height));

        Matrix4x4 rosOpticalToPixel = glUvToPixel * glNDCToGlUv * glViewToGlNdc * rosViewToGlView * rosOpticalToRosView;

        // depth shader
        Shader depthShader = Shader.Find("Custom/DepthTextureShader");
        depthMaterial = new Material(depthShader);
        depthMaterial.SetFloat("_DepthMin", depthMin);
        depthMaterial.SetFloat("_DepthMax", depthMax);

        // async capture
        asyncRt = new RenderTexture(width, height, 0);
        var syncRt = new RenderTexture(width, height, 0);
        var buffer = new NativeArray<byte>(width * height * 4, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        float lastCaptureTime = 0.0F;

        // debug fps counter
        // float frameTime = 0.016F;
        // float lastFpsCaptureTime = 0.0F;
        // float lastFpsPrintTime = 0.0F;

        // ROS
        var ros = new ROS();
        var unityRsPublisherMetaPub = await ros.CreatePublisher<ROSBridge.UnityRSPublisherMsgs.CameraMetadata>("/" + id + "/unity_rs_publisher/meta");
        OpenBridgeConnection(id, width, height, vFov, depthMin, depthMax);

        // Use a custom update loop to guarantee resource destruction.
        try
        {
            var cancel = destroyCancellationToken;
            while (true)
            {
                // Wait for the end of the frame.
                await Awaitable.EndOfFrameAsync(cancel);
                if (cancel.IsCancellationRequested)
                {
                    break;
                }

                // If a frame is available
                if (frameAvailable)
                {
                    // Limit the framerate.
                    if (Time.time - lastCaptureTime < 1.0F / fps)
                    {
                        continue;
                    }
                    lastCaptureTime = Time.time;

                    // // Debug: mesaure framerate
                    // float delta = Time.time - lastFpsCaptureTime;
                    // frameTime = frameTime * 0.9F + delta * 0.1F;
                    // lastFpsCaptureTime = Time.time;
                    // if (Time.time - lastFpsPrintTime > 1.0F)
                    // {
                    //     Debug.Log("FPS: " + (1.0F / frameTime).ToString("F2"));
                    //     lastFpsPrintTime = Time.time;
                    // }

                    // Copy from asyncRt to syncRt.
                    Graphics.Blit(asyncRt, syncRt);

                    // Copy frame stamp.
                    int sec = frameStamp.Sec;
                    uint nanosec = frameStamp.Nanosec;

                    // Re-enable camera after the blit.
                    cam.enabled = true;
                    frameAvailable = false;

                    // Copy the frame to the CPU.
                    try
                    {
                        await AsyncGPUReadback.RequestIntoNativeArrayAsync(ref buffer, syncRt, 0, TextureFormat.RGBA32);
                    }
                    catch (MissingReferenceException)
                    {
                        // TODO: This is a hacky fix.
                        continue;
                    }

                    // Send frame to C++ for further processing.
                    unsafe
                    {
                        TryPushFrame(id, sec, nanosec, NativeArrayUnsafeUtility.GetUnsafePtr(buffer));
                    }

                    // Publish depth min and depth max for unity_rs_publisher.
                    await unityRsPublisherMetaPub.Publish(new ROSBridge.UnityRSPublisherMsgs.CameraMetadata
                    {
                        Width = width,
                        Height = height,
                        DepthMin = depthMin,
                        DepthMax = depthMax,
                        Fx = rosOpticalToPixel[0, 0],
                        Fy = rosOpticalToPixel[1, 1],
                        Cx = rosOpticalToPixel[0, 2],
                        Cy = rosOpticalToPixel[1, 2]
                    });
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
        finally
        {
            if (buffer.IsCreated) buffer.Dispose();
            await ros.Close();
            CloseBridgeConnection(id);
        }
    }

    private void OnEnable()
    {
        EditorApplication.LockReloadAssemblies();
        RenderPipelineManager.beginCameraRendering += RenderPipelineManager_beginCameraRendering;
    }

    private void OnDisable()
    {
        RenderPipelineManager.beginCameraRendering -= RenderPipelineManager_beginCameraRendering;
        EditorApplication.UnlockReloadAssemblies();
    }

    private void RenderPipelineManager_beginCameraRendering(ScriptableRenderContext context, Camera camera) {
        currentRenderCam = camera;
    }

    void OnRenderObject()
    {
        if (currentRenderCam == cam) {
            cam.enabled = false;

            // Save the time stamp of the frame.
            frameStamp = ROSBridge.BuiltinInterfaces.Time.Realtime();
            // Blit through the depth shader to the async gbuffer.
            depthMaterial.SetTexture("_ColorTex", cam.targetTexture);
            Graphics.Blit(cam.targetTexture, asyncRt, depthMaterial);

            frameAvailable = true;
        }
    }
}
