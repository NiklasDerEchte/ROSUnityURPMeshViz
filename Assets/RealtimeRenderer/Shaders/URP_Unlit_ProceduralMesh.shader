Shader "Custom/URP_Unlit_ProceduralMesh"
{
  Properties { }
  SubShader
  {
    Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

    Pass
    {
      // ROS→Unity-Achsentausch kann Winding umdrehen
      Cull Off
      ZWrite On
      ZTest LEqual

      HLSLPROGRAM
      #pragma vertex vert
      #pragma fragment frag
      #pragma target 4.5

      #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

      // Positionen im lokalen Raum (nach ROS→Unity-Konvertierung)
      StructuredBuffer<float3> _Positions;
      StructuredBuffer<float4> _Colors;

      // Vom GameObject-Transform gesetzt → Position / Rotation / Scale
      float4x4 _LocalToWorld;

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
        float3 localPos = _Positions[input.vertexID];
        float3 worldPos = mul(_LocalToWorld, float4(localPos, 1.0)).xyz;
        output.positionCS = TransformWorldToHClip(worldPos);
        output.color = _Colors[input.vertexID];
        return output;
      }

      half4 frag(Varyings input) : SV_Target
      {
        return half4(input.color);
      }
      ENDHLSL
    }
  }
  Fallback Off
}
