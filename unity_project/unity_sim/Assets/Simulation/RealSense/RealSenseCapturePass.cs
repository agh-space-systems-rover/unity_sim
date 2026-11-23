using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

// Render Graph pass that runs at the end of a RealSense camera's render and
// packs its final color and its depth into one RGBA texture (depth in alpha)
// using Custom/DepthTextureShader. RealSense.cs injects it per camera from
// beginCameraRendering, so no renderer asset changes are needed.
//
// Unity 6.4+ always renders URP through Render Graph, where the camera depth
// texture is a transient resource that is gone once the camera finishes; the
// old Graphics.Blit in endCameraRendering therefore saw no depth at all.
public class RealSenseCapturePass : ScriptableRenderPass
{
    private class PassData
    {
        public RenderTexture color;
        public TextureHandle depth;
        public Material material;
        public MaterialPropertyBlock props;
    }

    private static readonly int BlitTextureId = Shader.PropertyToID("_BlitTexture");
    private static readonly int BlitScaleBiasId = Shader.PropertyToID("_BlitScaleBias");
    private static readonly int DepthTexId = Shader.PropertyToID("_RSDepthTex");

    private readonly Material material;
    private readonly MaterialPropertyBlock props = new MaterialPropertyBlock();
    private RenderTexture sourceRt;
    private RTHandle source, output;

    public RealSenseCapturePass(Material material)
    {
        this.material = material;
        renderPassEvent = RenderPassEvent.AfterRendering;
        // Make URP produce the camera depth texture for this camera.
        ConfigureInput(ScriptableRenderPassInput.Depth);
        profilingSampler = new ProfilingSampler("RealSense Capture");
    }

    // src: the camera's target texture (final color, after post-processing).
    // dst: where the packed color + depth image goes.
    public void SetTargets(RenderTexture src, RenderTexture dst)
    {
        if (sourceRt != src)
        {
            // The camera target has a depth buffer, which Render Graph refuses
            // to import as a combined texture. Like URP does for camera targets,
            // wrap it by identifier and describe it as color-only on import.
            source?.Release();
            source = RTHandles.Alloc(new RenderTargetIdentifier(src), "RealSense color");
            sourceRt = src;
        }
        if (output == null || output.rt != dst)
        {
            output?.Release();
            output = RTHandles.Alloc(dst);
        }
    }

    public void Dispose()
    {
        source?.Release();
        output?.Release();
        source = output = null;
        sourceRt = null;
    }

    public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
    {
        var resources = frameData.Get<UniversalResourceData>();
        if (sourceRt == null || output == null || !resources.cameraDepthTexture.IsValid())
        {
            return;
        }

        TextureHandle src = renderGraph.ImportTexture(source, new RenderTargetInfo
        {
            width = sourceRt.width,
            height = sourceRt.height,
            volumeDepth = 1,
            msaaSamples = 1,
            format = sourceRt.graphicsFormat,
        });
        TextureHandle dst = renderGraph.ImportTexture(output);

        using (var builder = renderGraph.AddRasterRenderPass<PassData>("RealSense Capture", out var data, profilingSampler))
        {
            data.color = sourceRt;
            data.depth = resources.cameraDepthTexture;
            data.material = material;
            data.props = props;

            // src is only declared for ordering; it is bound by RenderTexture below.
            builder.UseTexture(src, AccessFlags.Read);
            builder.UseTexture(data.depth, AccessFlags.Read);
            builder.SetRenderAttachment(dst, 0, AccessFlags.Write);
            builder.AllowPassCulling(false);

            builder.SetRenderFunc((PassData d, RasterGraphContext ctx) =>
            {
                d.props.SetTexture(BlitTextureId, d.color);
                d.props.SetTexture(DepthTexId, (RTHandle)d.depth);
                d.props.SetVector(BlitScaleBiasId, new Vector4(1, 1, 0, 0));
                // Fullscreen triangle; the shader uses Blit.hlsl's Vert.
                ctx.cmd.DrawProcedural(Matrix4x4.identity, d.material, 0, MeshTopology.Triangles, 3, 1, d.props);
            });
        }
    }
}
