using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Visualization;
using UnityEngine.Rendering;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

namespace RealtimeRenderer
{
  public class ROSMeshVisualizerProcedural : MonoBehaviour
  {
    [Header("ROS Settings")]
    public string rosTopicName = "/object_markers";

    [Header("Performance Settings")]
    [Tooltip("Maximale Anzahl Vertices, die gleichzeitig verarbeitet werden können")]
    public int maxVertices = 80000;

    [Header("Setup")]
    public Shader shader;

    private ROSConnection rosConnection;
    private Queue<MarkerArrayMsg> messageQueue = new Queue<MarkerArrayMsg>();

    // === Double Buffering ===
    private GraphicsBuffer[] positionBuffers = new GraphicsBuffer[2];
    private GraphicsBuffer[] colorBuffers = new GraphicsBuffer[2];
    private int currentBufferIndex = 0;

    // === Job System ===
    private JobHandle conversionJobHandle;
    private NativeArray<float3> positionsNative;
    private NativeArray<float4> colorsNative;

    // Pending Daten für Double Buffering
    private int pendingVertexCount = 0;
    private int pendingBufferIndex = 0;

    // Aktuelle Render-Daten
    private int currentVertexCount = 0;

    private Material material;

    void Start()
    {
      rosConnection = ROSConnection.GetOrCreateInstance();
      rosConnection.Subscribe<MarkerArrayMsg>(rosTopicName, OnMeshReceived);

      if (shader != null)
      {
        material = new Material(shader);
      }
      else
      {
        material = new Material(Shader.Find("Custom/URP_Unlit_ProceduralMesh"));
      }

      if (material == null)
      {
        Debug.LogError("[ROSMeshVisualizerProcedural] Shader 'Custom/URP_Unlit_ProceduralMesh' nicht gefunden!");
        enabled = false;
        return;
      }

      // GraphicsBuffers (Double Buffer) anlegen
      for (int i = 0; i < 2; i++)
      {
        positionBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 3);
        colorBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 4);
      }

      // NativeArrays einmalig allozieren (werden wiederverwendet)
      positionsNative = new NativeArray<float3>(maxVertices, Allocator.Persistent);
      colorsNative = new NativeArray<float4>(maxVertices, Allocator.Persistent);
    }

    private void OnMeshReceived(MarkerArrayMsg msg)
    {
      if (msg == null || msg.markers == null || msg.markers.Length == 0)
        return;

      messageQueue.Enqueue(msg);
    }

    void Update()
    {
      if (messageQueue.Count == 0)
      {
        return;
      }

      if (!conversionJobHandle.IsCompleted)
      { 
        return; 
      }

      conversionJobHandle.Complete(); // TODO Wieso hier schon Completed?

      var markerArray = messageQueue.Dequeue();
      MarkerMsg marker = null;
      foreach (var m in markerArray.markers)
      {
        if (m != null && m.type == MarkerMsg.TRIANGLE_LIST && m.points != null && m.points.Length > 0)
        {
          marker = m;
          break;
        }
      }

      if (marker == null)
        return;

      int vertexCount = Mathf.Min(marker.points.Length, maxVertices);

      // Job vorbereiten und starten (läuft parallel im Hintergrund)
      var job = new ConvertRosToNativeJob
      {
        rosPoints = marker.points,
        rosColors = marker.colors,
        outputPositions = positionsNative,
        outputColors = colorsNative
      };

      conversionJobHandle = job.Schedule(vertexCount, 128); // TODO Gibt conversionJobHandle sich selber frei wenn er fertig ist?

      // TODO Was ist das hier?
      pendingVertexCount = vertexCount;
      pendingBufferIndex = (currentBufferIndex + 1) % 2;
    }

    void LateUpdate()
    {
      // Daten erst hochladen, wenn der Job fertig ist
      if (!conversionJobHandle.IsCompleted) // TODO Wieso wird das alles hier in der LateUpdate aufgerufen?
        return;

      conversionJobHandle.Complete();

      if (pendingVertexCount <= 0)
        return;

      // In den vorbereiteten (inaktiven) Buffer schreiben
      positionBuffers[pendingBufferIndex].SetData(positionsNative, 0, 0, pendingVertexCount);
      colorBuffers[pendingBufferIndex].SetData(colorsNative, 0, 0, pendingVertexCount);

      // Buffer swap
      currentBufferIndex = pendingBufferIndex;
      currentVertexCount = pendingVertexCount;

      // Material aktualisieren
      material.SetBuffer("_Positions", positionBuffers[currentBufferIndex]);
      material.SetBuffer("_Colors", colorBuffers[currentBufferIndex]);

      pendingVertexCount = 0;
    }

    // Rendering (wird von Unity automatisch aufgerufen)
    void OnRenderObject() // TODO Wer ruft das auf und wieso sollte CommandBufferPool auf "ROSMeshProcedural" reagieren?
    {
      if (currentVertexCount <= 0 || material == null)
        return;

      CommandBuffer cmd = CommandBufferPool.Get("ROSMeshProcedural");

      cmd.DrawProcedural(
          Matrix4x4.identity,
          material,
          0,                              // Pass 0
          MeshTopology.Triangles,
          currentVertexCount,
          1
      );

      Graphics.ExecuteCommandBuffer(cmd);
      CommandBufferPool.Release(cmd);
    }

    void OnDestroy()
    {
      conversionJobHandle.Complete();

      for (int i = 0; i < 2; i++)
      {
        positionBuffers[i]?.Release();
        colorBuffers[i]?.Release();
      }

      if (positionsNative.IsCreated) positionsNative.Dispose();
      if (colorsNative.IsCreated) colorsNative.Dispose();

      if (rosConnection != null)
        rosConnection.Unsubscribe(rosTopicName);
    }

    [BurstCompile]
    private struct ConvertRosToNativeJob : IJobParallelFor
    {
      [ReadOnly] public PointMsg[] rosPoints;
      [ReadOnly] public ColorRGBAMsg[] rosColors;

      [WriteOnly] public NativeArray<float3> outputPositions;
      [WriteOnly] public NativeArray<float4> outputColors;

      public void Execute(int index)
      {
        var p = rosPoints[index];
        outputPositions[index] = new float3((float)p.x, (float)p.z, -(float)p.y);

        if (rosColors != null && index < rosColors.Length)
        {
          var c = rosColors[index];
          outputColors[index] = new float4((float)c.r, (float)c.g, (float)c.b, (float)c.a);
        }
        else
        {
          outputColors[index] = new float4(0f, 1f, 1f, 1f); // Cyan als Fallback
        }
      }
    }
  }
}