Shader "Custom/DepthTextureShader"
{
    SubShader {
        Tags
        {
            "RenderType"="Opaque"
        }
        
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            sampler2D _ColorTex;
            sampler2D _CameraDepthTexture;
            float _DepthMin;
            float _DepthMax;
            
            #define MOD3 float3(.1031,.11369,.13787)

            float3 hash33(float3 p3)
            {
                p3 = frac(p3 * MOD3);
                p3 += dot(p3, p3.yxz+19.19);
                return -1.0 + 2.0 * frac(float3((p3.x + p3.y)*p3.z, (p3.x+p3.z)*p3.y, (p3.y+p3.z)*p3.x));
            }

            float perlin_noise(float3 p)
            {
                float3 pi = floor(p);
                float3 pf = p - pi;
                float3 w = pf;
                
                return  lerp(
                            lerp(
                                lerp(dot(pf - float3(0, 0, 0), hash33(pi + float3(0, 0, 0))), 
                                    dot(pf - float3(1, 0, 0), hash33(pi + float3(1, 0, 0))),
                                    w.x),
                                lerp(dot(pf - float3(0, 0, 1), hash33(pi + float3(0, 0, 1))), 
                                    dot(pf - float3(1, 0, 1), hash33(pi + float3(1, 0, 1))),
                                    w.x),
                                w.z),
                            lerp(
                                lerp(dot(pf - float3(0, 1, 0), hash33(pi + float3(0, 1, 0))), 
                                    dot(pf - float3(1, 1, 0), hash33(pi + float3(1, 1, 0))),
                                    w.x),
                                lerp(dot(pf - float3(0, 1, 1), hash33(pi + float3(0, 1, 1))), 
                                    dot(pf - float3(1, 1, 1), hash33(pi + float3(1, 1, 1))),
                                    w.x),
                                w.z),
                            w.y);
            }

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
            };
            
            v2f vert (appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.texcoord;
                return o;
            }

            half4 frag (v2f i) : COLOR
            {
                float zDistance = LinearEyeDepth(tex2D(_CameraDepthTexture, i.uv).x);

                // Go over a neighborhood and see if there are any other pixels that are closer to the camera. The threshold increases further from the center of the filter.
                #define NEIGHBORHOOD_SIZE 7
                for (int x = -NEIGHBORHOOD_SIZE; x <= NEIGHBORHOOD_SIZE; x++)
                {
                    for (int y = -NEIGHBORHOOD_SIZE; y <= NEIGHBORHOOD_SIZE; y++)
                    {
                        float2 offset = float2(x, y) / _ScreenParams.xy;
                        float neighborZDistance = LinearEyeDepth(tex2D(_CameraDepthTexture, i.uv + offset).x);
                        float radius = length(offset);
                        float thres = 30.0 * radius;
                        if (neighborZDistance < zDistance - thres)
                        {
                            zDistance = 0;
                        }
                    }
                }

                // Clip and normalize
                // // (value = [0, 1] fraction that maps linearly to [0, depth max - depth min])
                // // value = 0
                float cutoff = (1 - step(_DepthMax, zDistance)) * step(_DepthMin, zDistance);
                float linearDepth = (zDistance - _DepthMin) / (_DepthMax - _DepthMin);
                linearDepth *= cutoff;
                
                // Add noise to depth
                float noise = 0.0;
                float time = _Time.y; // Unity's time variable
                noise += perlin_noise(float3(i.uv * 10.0, time*0.5)) * 0.5;
                noise -= perlin_noise(float3(i.uv * 30.0, time*7.1));
                noise += perlin_noise(float3(i.uv * 100.0, time*5.1));
                noise -= perlin_noise(float3(i.uv * 3.0, time*1.3)) * 0.1;
                noise *= 0.1;
                linearDepth += noise * linearDepth;
                linearDepth = saturate(linearDepth);

                // Add random zeros
                linearDepth *= step(perlin_noise(float3(i.uv * 20.0, time*30.3)), 0.4);

                // Black bands on both sides of the screen
                float bandWidth = 0.05;
                linearDepth *= step(abs(i.uv.x * 2.0 - 1.0), 1.0 - bandWidth);
                
                float4 color;
                color.xyz = tex2D(_ColorTex, i.uv).xyz;
                color.w = linearDepth;
                return color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}