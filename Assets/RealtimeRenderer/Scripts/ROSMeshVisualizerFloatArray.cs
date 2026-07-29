using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine.Rendering;

namespace RealtimeRenderer
{
  /// <summary>
  /// Procedural mesh visualizer for std_msgs/Float32MultiArray:
  /// interleaved [x,y,z,r,g,b,a] per vertex, TRIANGLE_LIST order.
  /// </summary>
  public class ROSMeshVisualizerFloatArray : MonoBehaviour
  {
    private const int FloatsPerVertex = 7;

    [Header("ROS Settings")]
    public string rosTopicName = "/object_mesh/vertices";

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
    private Queue<Float32MultiArrayMsg> messageQueue = new Queue<Float32MultiArrayMsg>();

    private GraphicsBuffer[] positionBuffers = new GraphicsBuffer[2];
    private GraphicsBuffer[] colorBuffers = new GraphicsBuffer[2];
    private int currentBufferIndex = 0;

    private JobHandle conversionJobHandle;
    private NativeArray<float> interleavedNative;
    private NativeArray<float3> positionsNative;
    private NativeArray<float4> colorsNative;

    private int pendingVertexCount = 0;
    private int pendingBufferIndex = 0;

    private int currentVertexCount = 0;
    private Bounds localBounds = new Bounds(Vector3.zero, Vector3.one * 10f);

    private Material material;
    private static readonly int LocalToWorldId = Shader.PropertyToID("_LocalToWorld");

    private float meshProcessStartTime = -1f;

    void Start()
    {
      rosConnection = ROSConnection.GetOrCreateInstance();
      if (rosConnection == null)
      {
        Debug.LogError("[ROS Mesh FA] Fehler: ROSConnection konnte nicht erstellt werden.");
        enabled = false;
        return;
      }

      rosConnection.Subscribe<Float32MultiArrayMsg>(rosTopicName, OnMeshReceived);

      if (shader != null)
      {
        material = new Material(shader);
      }
      else
      {
        var found = Shader.Find("Custom/URP_Unlit_ProceduralMesh");
        if (found == null)
        {
          Debug.LogError("[ROS Mesh FA] Fehler: Shader 'Custom/URP_Unlit_ProceduralMesh' nicht gefunden.");
          enabled = false;
          return;
        }
        material = new Material(found);
      }

      if (material == null || material.shader == null || !material.shader.isSupported)
      {
        Debug.LogError("[ROS Mesh FA] Fehler: Material/Shader ungültig oder nicht supported.");
        enabled = false;
        return;
      }

      for (int i = 0; i < 2; i++)
      {
        positionBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 3);
        colorBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.Structured, maxVertices, sizeof(float) * 4);
      }

      interleavedNative = new NativeArray<float>(maxVertices * FloatsPerVertex, Allocator.Persistent);
      positionsNative = new NativeArray<float3>(maxVertices, Allocator.Persistent);
      colorsNative = new NativeArray<float4>(maxVertices, Allocator.Persistent);
    }

    private void OnMeshReceived(Float32MultiArrayMsg msg)
    {
      if (msg == null || msg.data == null || msg.data.Length == 0)
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

      var msg = messageQueue.Dequeue();
      float[] data = msg.data;

      if (data == null || data.Length < FloatsPerVertex * 3)
      {
        Log($"Ungültig: zu wenig Daten ({(data == null ? 0 : data.Length)}). Queue: {messageQueue.Count}");
        return;
      }

      if (data.Length % FloatsPerVertex != 0)
      {
        Log($"Ungültig: Länge {data.Length} nicht durch {FloatsPerVertex} teilbar. Queue: {messageQueue.Count}");
        return;
      }

      int vertexCount = data.Length / FloatsPerVertex;
      vertexCount = Mathf.Min(vertexCount, maxVertices);
      vertexCount -= vertexCount % 3;

      if (vertexCount < 3)
      {
        Log($"Ungültig: zu wenige Vertices ({vertexCount}). Queue: {messageQueue.Count}");
        return;
      }

      int floatCount = vertexCount * FloatsPerVertex;

      // Block-Copy managed float[] → NativeArray (kein PointMsg-Loop)
      NativeArray<float>.Copy(data, 0, interleavedNative, 0, floatCount);

      var job = new InterleavedToVertexJob
      {
        interleaved = interleavedNative,
        positions = positionsNative,
        colors = colorsNative
      };

      conversionJobHandle = job.Schedule(vertexCount, 128);

      pendingVertexCount = vertexCount;
      pendingBufferIndex = (currentBufferIndex + 1) % 2;
      meshProcessStartTime = Time.realtimeSinceStartup;

      Log($"Gültig: Vertices={vertexCount}. Queue: {messageQueue.Count}");
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

      if (interleavedNative.IsCreated) interleavedNative.Dispose();
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
      Debug.Log($"[ROS Mesh FA] {message}");
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
    private struct InterleavedToVertexJob : IJobParallelFor
    {
      [ReadOnly] public NativeArray<float> interleaved;
      [WriteOnly] public NativeArray<float3> positions;
      [WriteOnly] public NativeArray<float4> colors;

      public void Execute(int index)
      {
        int o = index * 7;
        float x = interleaved[o];
        float y = interleaved[o + 1];
        float z = interleaved[o + 2];

        // ROS (x,y,z) → Unity (x, z, -y)
        positions[index] = new float3(x, z, -y);

        colors[index] = new float4(
          interleaved[o + 3],
          interleaved[o + 4],
          interleaved[o + 5],
          interleaved[o + 6]
        );
      }
    }
  }
}
