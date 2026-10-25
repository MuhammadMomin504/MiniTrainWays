// Road surface for SplineTrack meshes (roads and junctions share this one material).
// Asphalt is mapped from the ground plane, so the paving runs continuously through road
// strips and junction pads with no stretching or seam where they meet. Markings use the
// strip UVs (u across the road, v along it in road widths; stored projectively as u·q, v·q, q
// so tapers stay straight) and are masked by vertex alpha,
// which SplineTrack sets to 0 on junction pads so lane paint never smears across a junction.
Shader "MiniTrainWays/Road Surface"
{
    Properties
    {
        [MainTexture] _AsphaltTex("Asphalt", 2D) = "gray" {}
        [MainColor] _AsphaltTint("Asphalt Tint", Color) = (1, 1, 1, 1)
        _AsphaltTiling("Asphalt Tiles Per Unit", Float) = 1
        _MarkingsTex("Markings (U across, V along)", 2D) = "black" {}
        _MarkingRepeat("Marking Repeat (road widths)", Float) = 3
        _Smoothness("Asphalt Smoothness", Range(0, 1)) = 0.18
        _MarkingSmoothness("Marking Smoothness", Range(0, 1)) = 0.35
    }

    SubShader
    {
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

        CBUFFER_START(UnityPerMaterial)
            float4 _AsphaltTex_ST;
            half4 _AsphaltTint;
            float _AsphaltTiling;
            float4 _MarkingsTex_ST;
            float _MarkingRepeat;
            half _Smoothness;
            half _MarkingSmoothness;
        CBUFFER_END
        ENDHLSL

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile_fragment _ _SHADOWS_SOFT _SHADOWS_SOFT_LOW _SHADOWS_SOFT_MEDIUM _SHADOWS_SOFT_HIGH
            #pragma multi_compile_fragment _ _SCREEN_SPACE_OCCLUSION
            #pragma multi_compile_fog

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            TEXTURE2D(_AsphaltTex);
            SAMPLER(sampler_AsphaltTex);
            TEXTURE2D(_MarkingsTex);
            SAMPLER(sampler_MarkingsTex);

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float3 uv : TEXCOORD0;
                half4 color : COLOR;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float3 uv : TEXCOORD2;
                float2 planarUV : TEXCOORD3;
                half markingMask : TEXCOORD4;
                half fogFactor : TEXCOORD5;
            };

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs positions = GetVertexPositionInputs(input.positionOS.xyz);
                output.positionCS = positions.positionCS;
                output.positionWS = positions.positionWS;
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                output.uv = input.uv;
                output.planarUV = input.positionOS.xz * _AsphaltTiling;
                output.markingMask = input.color.a;
                output.fogFactor = ComputeFogFactor(positions.positionCS.z);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                half3 asphalt = SAMPLE_TEXTURE2D(_AsphaltTex, sampler_AsphaltTex, input.planarUV).rgb * _AsphaltTint.rgb;
                float2 stripUV = input.uv.xy / max(input.uv.z, 0.0001);
                half4 marking = SAMPLE_TEXTURE2D(_MarkingsTex, sampler_MarkingsTex,
                    float2(stripUV.x, stripUV.y / max(_MarkingRepeat, 0.001)));
                half paint = marking.a * saturate(input.markingMask);

                SurfaceData surface = (SurfaceData)0;
                surface.albedo = lerp(asphalt, marking.rgb, paint);
                surface.smoothness = lerp(_Smoothness, _MarkingSmoothness, paint);
                surface.normalTS = half3(0, 0, 1);
                surface.occlusion = 1;
                surface.alpha = 1;

                InputData lighting = (InputData)0;
                lighting.positionWS = input.positionWS;
                lighting.positionCS = input.positionCS;
                lighting.normalWS = NormalizeNormalPerPixel(input.normalWS);
                lighting.viewDirectionWS = GetWorldSpaceNormalizeViewDir(input.positionWS);
                lighting.shadowCoord = TransformWorldToShadowCoord(input.positionWS);
                lighting.fogCoord = input.fogFactor;
                lighting.bakedGI = SampleSH(lighting.normalWS);
                lighting.normalizedScreenSpaceUV = GetNormalizedScreenSpaceUV(input.positionCS);
                lighting.shadowMask = half4(1, 1, 1, 1);

                half4 color = UniversalFragmentPBR(lighting, surface);
                color.rgb = MixFog(color.rgb, input.fogFactor);
                color.a = 1;
                return color;
            }
            ENDHLSL
        }

        Pass
        {
            Name "DepthOnly"
            Tags { "LightMode" = "DepthOnly" }
            ZWrite On
            ColorMask R

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            struct Attributes { float4 positionOS : POSITION; };
            struct Varyings { float4 positionCS : SV_POSITION; };

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
                return output;
            }

            half frag(Varyings input) : SV_Target { return 0; }
            ENDHLSL
        }

        Pass
        {
            Name "DepthNormals"
            Tags { "LightMode" = "DepthNormals" }
            ZWrite On

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            struct Attributes { float4 positionOS : POSITION; float3 normalOS : NORMAL; };
            struct Varyings { float4 positionCS : SV_POSITION; float3 normalWS : TEXCOORD0; };

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                return half4(NormalizeNormalPerPixel(input.normalWS), 0);
            }
            ENDHLSL
        }
    }

    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}
