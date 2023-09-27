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
            // float _DepthMin;
            // float _DepthMax;
            float4x4 _InvProj;
            float4x4 _DepthProj;
            
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
                // float zDistance = LinearEyeDepth(tex2D(_CameraDepthTexture, i.uv).x);
                // float value = (zDistance - _DepthMin) / (_DepthMax - _DepthMin);
                // // (value = [0, 1] fraction that maps linearly to [0, depth max - depth min])
                // // value = 0
                // float cutoff = (1 - step(_DepthMax, zDistance)) * step(_DepthMin, zDistance);
                // value *= cutoff; 
                
                // float4 color;
                // color.xyz = tex2D(_ColorTex, i.uv).xyz;
                // color.w = value;
                // return color;

                // Find the 3D position of this fragment
                float depth = tex2D(_CameraDepthTexture, i.uv).x;
                #if UNITY_REVERSED_Z
                depth = 1 - depth;
                #endif
                float4 pos = float4(float3(i.uv, depth) * 2 - 1, 1); // for some reason in Unity depth does not have to be NDC
                pos = mul(_InvProj, pos);
                pos /= pos.w;
                
                // Transform 3D pos to depth NDC
                pos = mul(_DepthProj, pos);
                pos /= pos.w;
                depth = pos.z * 0.5 + 0.5;
                
                float4 color;
                color.xyz = tex2D(_ColorTex, i.uv).xyz;
                color.w = depth;
                return color;
                
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}