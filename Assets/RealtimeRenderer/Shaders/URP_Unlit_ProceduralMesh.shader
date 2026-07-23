Shader "Custom/URP_Unlit_ProceduralMesh"
{
  Properties { }
  SubShader
  {
    Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

    Pass
    {
      HLSLPROGRAM
      #pragma vertex vert
      #pragma fragment frag

      #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

      StructuredBuffer<float3> _Positions;
      StructuredBuffer<float4> _Colors;

      struct Attributes
      {
        uint vertexID : SV_VertexID;
      };

      struct Varyings
      {
        float4 positionCS : SV_POSITION;
        float4 color : COLOR;
      };

      Varyings vert(Attributes input)
      {
        Varyings output;
        float3 worldPos = _Positions[input.vertexID];
        output.positionCS = TransformObjectToHClip(worldPos);
        output.color = _Colors[input.vertexID];
        return output;
      }

      half4 frag(Varyings input) : SV_Target
      {
        return input.color;
      }
      ENDHLSL
    }
  }
}