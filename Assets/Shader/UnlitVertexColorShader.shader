Shader "Unlit/VertexColor"
{
    Properties
    {
        // Keine zusätzlichen Eigenschaften nötig, aber optional könntest du Texturen oder Farben hinzufügen.
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION; // Vertex-Position
                float4 color : COLOR;    // Vertex-Farbe
            };

            struct v2f
            {
                float4 pos : SV_POSITION; // Bildschirmposition
                float4 color : COLOR;     // Vertex-Farbe
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex); // Transformiere Position
                o.color = v.color;                     // Vertex-Farbe durchreichen
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return i.color; // Vertex-Farbe als Pixel-Farbe verwenden
            }
            ENDCG
        }
    }
}
