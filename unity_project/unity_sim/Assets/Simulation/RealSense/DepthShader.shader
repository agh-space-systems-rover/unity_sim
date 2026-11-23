// Packs a RealSense camera's color (RGB) and normalized, noisy depth (A) into
// one RGBA texture. Drawn by RealSenseCapturePass as a fullscreen Blitter pass:
// _BlitTexture is the camera color, _RSDepthTex the URP camera depth texture.
Shader "Custom/DepthTextureShader"
{
    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" }
        ZTest Always ZWrite Off Cull Off

        Pass
        {
            Name "RealSenseCapture"

            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"

            TEXTURE2D_X_FLOAT(_RSDepthTex);
            float _DepthMin;
            float _DepthMax;
            float4 _RSTexelSize; // (1/width, 1/height, 0, 0)

            #define MOD3 float3(.1031, .11369, .13787)

            float3 hash33(float3 p3)
            {
                p3 = frac(p3 * MOD3);
                p3 += dot(p3, p3.yxz + 19.19);
                return -1.0 + 2.0 * frac(float3((p3.x + p3.y) * p3.z, (p3.x + p3.z) * p3.y, (p3.y + p3.z) * p3.x));
            }

            float perlin_noise(float3 p)
            {
                float3 pi = floor(p);
                float3 pf = p - pi;
                float3 w = pf;

                return lerp(
                    lerp(
                        lerp(dot(pf - float3(0, 0, 0), hash33(pi + float3(0, 0, 0))),
                             dot(pf - float3(1, 0, 0), hash33(pi + float3(1, 0, 0))), w.x),
                        lerp(dot(pf - float3(0, 0, 1), hash33(pi + float3(0, 0, 1))),
                             dot(pf - float3(1, 0, 1), hash33(pi + float3(1, 0, 1))), w.x),
                        w.z),
                    lerp(
                        lerp(dot(pf - float3(0, 1, 0), hash33(pi + float3(0, 1, 0))),
                             dot(pf - float3(1, 1, 0), hash33(pi + float3(1, 1, 0))), w.x),
                        lerp(dot(pf - float3(0, 1, 1), hash33(pi + float3(0, 1, 1))),
                             dot(pf - float3(1, 1, 1), hash33(pi + float3(1, 1, 1))), w.x),
                        w.z),
                    w.y);
            }

            float EyeDepthAt(float2 uv)
            {
                float raw = SAMPLE_TEXTURE2D_X(_RSDepthTex, sampler_PointClamp, uv).r;
                return LinearEyeDepth(raw, _ZBufferParams);
            }

            half4 frag(Varyings i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float2 uv = i.texcoord;
                float zDistance = EyeDepthAt(uv);

                // Go over a neighborhood and see if there are any other pixels that are closer to the camera.
                // The threshold increases further from the center of the filter.
                #define NEIGHBORHOOD_SIZE 7
                for (int x = -NEIGHBORHOOD_SIZE; x <= NEIGHBORHOOD_SIZE; x++)
                {
                    for (int y = -NEIGHBORHOOD_SIZE; y <= NEIGHBORHOOD_SIZE; y++)
                    {
                        float2 offset = float2(x, y) * _RSTexelSize.xy;
                        float neighborZDistance = EyeDepthAt(uv + offset);
                        float radius = length(offset);
                        float thres = 30.0 * radius;
                        if (neighborZDistance < zDistance - thres)
                        {
                            zDistance = 0;
                        }
                    }
                }

                // Clip and normalize to [0, 1] over [depth min, depth max].
                float cutoff = (1 - step(_DepthMax, zDistance)) * step(_DepthMin, zDistance);
                float linearDepth = (zDistance - _DepthMin) / (_DepthMax - _DepthMin);
                linearDepth *= cutoff;

                // Add noise to depth.
                float noise = 0.0;
                float time = _Time.y;
                noise += perlin_noise(float3(uv * 10.0, time * 0.5)) * 0.5;
                noise -= perlin_noise(float3(uv * 30.0, time * 7.1));
                noise += perlin_noise(float3(uv * 100.0, time * 5.1));
                noise -= perlin_noise(float3(uv * 3.0, time * 1.3)) * 0.1;
                noise *= 0.1;
                linearDepth += noise * linearDepth;
                linearDepth = saturate(linearDepth);

                // Add random zeros.
                linearDepth *= step(perlin_noise(float3(uv * 20.0, time * 30.3)), 0.4);

                // Black bands on both sides of the screen.
                float bandWidth = 0.05;
                linearDepth *= step(abs(uv.x * 2.0 - 1.0), 1.0 - bandWidth);

                float3 color = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, uv).rgb;
                return half4(color, linearDepth);
            }
            ENDHLSL
        }
    }
}
