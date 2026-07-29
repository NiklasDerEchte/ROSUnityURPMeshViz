using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Visualization;
using UnityEngine.Rendering;

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

    [Header("Mesh Pose")]
    [Tooltip("GameObject, dessen Position/Rotation/Scale das Mesh steuert. Leer = dieses Objekt.")]
    public Transform meshPose;

    [Header("Debug")]
    [Tooltip("Logs in die Unity Console schreiben")]
    public bool enableLogs = true;

    private ROSConnection rosConnection;
    private Queue<MarkerArrayMsg> messageQueue = new Queue<MarkerArrayMsg>();

    private GraphicsBuffer[] positionBuffers = new GraphicsBuffer[2];
    private GraphicsBuffer[] colorBuffers = new GraphicsBuffer[2];
    private int currentBufferIndex = 0;

    private JobHandle conversionJobHandle;
    private NativeArray<float3> positionsNative;
    private NativeArray<float4> colorsNative;

    private int pendingVertexCount = 0;
    private int pendingBufferIndex = 0;

    private int currentVertexCount = 0;
    private Bounds localBounds = new Bounds(Vector3.zero, Vector3.one * 10f);

    private Material material;
    private static readonly int LocalToWorldId = Shader.PropertyToID("_LocalToWorld");

    // Zeit von gültiger Message bis Mesh auf GPU
    private float meshProcessStartTime = -1f;

    void Start()
    {
      rosConnection = ROSConnection.GetOrCreateInstance();
      if (rosConnection == null)
      {
        Debug.LogError("[ROS Mesh] Fehler: ROSConnection konnte nicht erstellt werden.");
        enabled = false;
        return;
      }

      rosConnection.Subscribe<MarkerArrayMsg>(rosTopicName, OnMeshReceived);

      if (shader != null)
      {
        material = new Material(shader);
      }
      else
      {
        var found = Shader.Find("Custom/URP_Unlit_ProceduralMesh");
        if (found == null)
        {
          Debug.LogError("[ROS Mesh] Fehler: Shader 'Custom/URP_Unlit_ProceduralMesh' nicht gefunden.");
          enabled = false;
          return;
        }
        material = new Material(found);
      }

      if (material == null || material.shader == null || !material.shader.isSupported)
      {
        Debug.LogError("[ROS Mesh] Fehler: Material/Shader ungültig oder nicht supported.");
        enabled = false;
        return;
      }

      for (int i = 0; i < 2; i++)
      {
        positionBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 3);
        colorBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 4);
      }

      positionsNative = new NativeArray<float3>(maxVertices, Allocator.Persistent);
      colorsNative = new NativeArray<float4>(maxVertices, Allocator.Persistent);
    }

    private void OnMeshReceived(MarkerArrayMsg msg)
    {
      if (msg == null || msg.markers == null || msg.markers.Length == 0)
        return;

      messageQueue.Enqueue(msg);
      Log($"Neue Nachricht. Queue: {messageQueue.Count}");
    }

    void Update()
    {
      if (messageQueue.Count == 0)
        return;

      if (!conversionJobHandle.IsCompleted)
        return;

      conversionJobHandle.Complete();

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
      {
        Log($"Ungültig: kein Triangle-List. Queue: {messageQueue.Count}");
        return;
      }

      int vertexCount = Mathf.Min(marker.points.Length, maxVertices);
      if (vertexCount < 3)
      {
        Log($"Ungültig: zu wenige Vertices ({vertexCount}). Queue: {messageQueue.Count}");
        return;
      }

      vertexCount -= vertexCount % 3;

      var inputX = new NativeArray<double>(vertexCount, Allocator.TempJob);
      var inputY = new NativeArray<double>(vertexCount, Allocator.TempJob);
      var inputZ = new NativeArray<double>(vertexCount, Allocator.TempJob);
      var inputR = new NativeArray<float>(vertexCount, Allocator.TempJob);
      var inputG = new NativeArray<float>(vertexCount, Allocator.TempJob);
      var inputB = new NativeArray<float>(vertexCount, Allocator.TempJob);
      var inputA = new NativeArray<float>(vertexCount, Allocator.TempJob);

      bool hasColors = marker.colors != null && marker.colors.Length > 0;

      for (int i = 0; i < vertexCount; i++)
      {
        inputX[i] = marker.points[i].x;
        inputY[i] = marker.points[i].y;
        inputZ[i] = marker.points[i].z;

        if (hasColors && i < marker.colors.Length)
        {
          inputR[i] = (float)marker.colors[i].r;
          inputG[i] = (float)marker.colors[i].g;
          inputB[i] = (float)marker.colors[i].b;
          inputA[i] = (float)marker.colors[i].a;
        }
        else
        {
          inputR[i] = 0f;
          inputG[i] = 1f;
          inputB[i] = 1f;
          inputA[i] = 1f;
        }
      }
      //TODO Ist ConvertRosToNativeJob nicht überflüssig, weil ich doch schon eimal über die vertexCount iterriere und damit auch einfach die float3 erzeugen könnte?
      var job = new ConvertRosToNativeJob
      {
        inputX = inputX,
        inputY = inputY,
        inputZ = inputZ,
        inputR = inputR,
        inputG = inputG,
        inputB = inputB,
        inputA = inputA,
        hasColors = hasColors,
        outputPositions = positionsNative,
        outputColors = colorsNative
      };
      // TODO Was mach das hier ?
      conversionJobHandle = job.Schedule(vertexCount, 128);
      conversionJobHandle = inputX.Dispose(conversionJobHandle);
      conversionJobHandle = inputY.Dispose(conversionJobHandle);
      conversionJobHandle = inputZ.Dispose(conversionJobHandle);
      conversionJobHandle = inputR.Dispose(conversionJobHandle);
      conversionJobHandle = inputG.Dispose(conversionJobHandle);
      conversionJobHandle = inputB.Dispose(conversionJobHandle);
      conversionJobHandle = inputA.Dispose(conversionJobHandle);

      pendingVertexCount = vertexCount;
      pendingBufferIndex = (currentBufferIndex + 1) % 2;
      meshProcessStartTime = Time.realtimeSinceStartup;

      string ns = marker.ns ?? "";
      Log($"Gültig: ns=\"{ns}\" id={marker.id}, Vertices={vertexCount}. Queue: {messageQueue.Count}");
    }

    void LateUpdate()
    {
      if (conversionJobHandle.IsCompleted && pendingVertexCount > 0)
      {
        conversionJobHandle.Complete();

        int count = pendingVertexCount;
        positionBuffers[pendingBufferIndex].SetData(positionsNative, 0, 0, count);
        colorBuffers[pendingBufferIndex].SetData(colorsNative, 0, 0, count);

        localBounds = ComputeLocalBounds(positionsNative, count);

        currentBufferIndex = pendingBufferIndex;
        currentVertexCount = count;

        material.SetBuffer("_Positions", positionBuffers[currentBufferIndex]);
        material.SetBuffer("_Colors", colorBuffers[currentBufferIndex]);

        pendingVertexCount = 0;

        float ms = meshProcessStartTime >= 0f
          ? (Time.realtimeSinceStartup - meshProcessStartTime) * 1000f
          : 0f;
        meshProcessStartTime = -1f;
        Log($"Mesh fertig: {count} Vertices in {ms:F1} ms");
      }

      if (currentVertexCount <= 0 || material == null)
        return;

      Transform pose = meshPose != null ? meshPose : transform;
      Matrix4x4 localToWorld = pose.localToWorldMatrix;
      material.SetMatrix(LocalToWorldId, localToWorld);

      Bounds worldBounds = TransformBounds(localBounds, localToWorld);

      Graphics.DrawProcedural(
        material,
        worldBounds,
        MeshTopology.Triangles,
        currentVertexCount,
        1,
        null,
        null,
        ShadowCastingMode.Off,
        false,
        gameObject.layer
      );
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

      if (material != null)
      {
        if (Application.isPlaying)
          Destroy(material);
        else
          DestroyImmediate(material);
      }

      if (rosConnection != null)
        rosConnection.Unsubscribe(rosTopicName);
    }

    private void Log(string message)
    {
      if (!enableLogs)
        return;
      Debug.Log($"[ROS Mesh] {message}");
    }

    private static Bounds ComputeLocalBounds(NativeArray<float3> positions, int count)
    {
      if (count <= 0)
        return new Bounds(Vector3.zero, Vector3.one);

      float3 minP = positions[0];
      float3 maxP = positions[0];
      for (int i = 1; i < count; i++)
      {
        minP = math.min(minP, positions[i]);
        maxP = math.max(maxP, positions[i]);
      }

      Vector3 center = (Vector3)((minP + maxP) * 0.5f);
      Vector3 size = (Vector3)(maxP - minP);
      size.x = Mathf.Max(size.x, 0.01f);
      size.y = Mathf.Max(size.y, 0.01f);
      size.z = Mathf.Max(size.z, 0.01f);
      size *= 1.05f;
      return new Bounds(center, size);
    }

    private static Bounds TransformBounds(Bounds local, Matrix4x4 localToWorld)
    {
      Vector3 center = localToWorld.MultiplyPoint3x4(local.center);
      Vector3 ext = local.extents;
      Vector3 axisX = localToWorld.MultiplyVector(new Vector3(ext.x, 0f, 0f));
      Vector3 axisY = localToWorld.MultiplyVector(new Vector3(0f, ext.y, 0f));
      Vector3 axisZ = localToWorld.MultiplyVector(new Vector3(0f, 0f, ext.z));
      Vector3 worldExt = new Vector3(
        Mathf.Abs(axisX.x) + Mathf.Abs(axisY.x) + Mathf.Abs(axisZ.x),
        Mathf.Abs(axisX.y) + Mathf.Abs(axisY.y) + Mathf.Abs(axisZ.y),
        Mathf.Abs(axisX.z) + Mathf.Abs(axisY.z) + Mathf.Abs(axisZ.z)
      );
      return new Bounds(center, worldExt * 2f);
    }

    [BurstCompile]
    private struct ConvertRosToNativeJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<double> inputX;
      [ReadOnly] public NativeArray<double> inputY;
      [ReadOnly] public NativeArray<double> inputZ;
      [ReadOnly] public NativeArray<float> inputR;
      [ReadOnly] public NativeArray<float> inputG;
      [ReadOnly] public NativeArray<float> inputB;
      [ReadOnly] public NativeArray<float> inputA;
      public bool hasColors;

      [WriteOnly] public NativeArray<float3> outputPositions;
      [WriteOnly] public NativeArray<float4> outputColors;

      public void Execute(int index)
      {
        outputPositions[index] = new float3(
          (float)inputX[index],
          (float)inputZ[index],
          -(float)inputY[index]
        );

        if (hasColors)
        {
          outputColors[index] = new float4(
            inputR[index],
            inputG[index],
            inputB[index],
            inputA[index]
          );
        }
        else
        {
          outputColors[index] = new float4(0f, 1f, 1f, 1f);
        }
      }
    }
  }
}
