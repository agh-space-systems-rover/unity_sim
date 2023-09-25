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
                float value = (zDistance - _DepthMin) / (_DepthMax - _DepthMin);
                // (value = [0, 1] fraction that maps linearly to [0, depth max - depth min])
                // value = 0
                float cutoff = (1 - step(_DepthMax, zDistance)) * step(_DepthMin, zDistance);
                value *= cutoff; 
                
                float4 color;
                color.xyz = tex2D(_ColorTex, i.uv).xyz;
                color.w = value;
                return color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}