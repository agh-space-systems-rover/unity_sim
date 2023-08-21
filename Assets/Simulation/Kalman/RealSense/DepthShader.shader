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
            
            sampler2D _CameraDepthTexture;
            float _DepthRange;
            float _DepthMin;
            float _DepthMax;
            
            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 scrPos : TEXCOORD1;
            };
            
            v2f vert (appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos (v.vertex);
                o.scrPos = ComputeScreenPos(o.pos);
                return o;
            }

            half4 frag (v2f i) : COLOR
            {
                float depth = tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos)).x;
                float linearDepth = Linear01Depth(depth);
                float distance = _DepthRange * linearDepth;
                float cutoff = (1 - step(_DepthMax, distance)) * step(_DepthMin, distance);
                float value = (distance - _DepthMin) / (_DepthMax - _DepthMin);
                value *= cutoff; 
                
                float4 color;
                color.x = value;
                color.y = value;
                color.z = value;
                color.w = 1;
                return color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}