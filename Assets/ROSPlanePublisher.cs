using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// Publishes this GameObject's world pose as geometry_msgs/PoseStamped.
/// Position/rotation use the same ROS↔Unity axis convention as the mesh visualizers:
///   ROS (x,y,z) → Unity (x, z, -y)
///   Unity (x,y,z) → ROS (x, -z, y)
/// </summary>
public class ROSPlanePublisher : MonoBehaviour
{
  [Header("ROS")]
  public string topicName = "/unity/clipping_plane";
  [Tooltip("Must match the frame of your point cloud / mesh (e.g. camera optical frame).")]
  public string frameId = "zivid_optical_frame";

  [Header("Publish")]
  [Tooltip("Messages per second. 0 = every Update.")]
  public float publishRateHz = 20f;
  [Tooltip("Only publish when position/rotation changed (beyond epsilon).")]
  public bool publishOnlyOnChange = false;
  public float positionEpsilon = 0.0001f;
  public float rotationEpsilonDeg = 0.05f;

  [Header("Debug")]
  public bool enableLogs = false;

  private ROSConnection ros;
  private float nextPublishTime;
  private Vector3 lastPos;
  private Quaternion lastRot;
  private bool hasLast;

  // ROS (x,y,z) → Unity (x, z, -y) as 3x3 (columns = images of basis)
  private static readonly Matrix4x4 RosToUnityBasis = CreateRosToUnityBasis();

  void Start()
  {
    ros = ROSConnection.GetOrCreateInstance();
    if (ros == null)
    {
      Debug.LogError("[ROSPlanePublisher] ROSConnection failed.");
      enabled = false;
      return;
    }

    ros.RegisterPublisher<PoseStampedMsg>(topicName);
    nextPublishTime = Time.time;

    if (enableLogs)
      Debug.Log($"[ROSPlanePublisher] Publishing PoseStamped on {topicName}, frame_id={frameId}");
  }

  void Update()
  {
    if (publishRateHz > 0f && Time.time < nextPublishTime)
      return;

    if (publishRateHz > 0f)
      nextPublishTime = Time.time + 1f / publishRateHz;

    Vector3 pos = transform.position;
    Quaternion rot = transform.rotation;

    if (publishOnlyOnChange && hasLast)
    {
      if ((pos - lastPos).sqrMagnitude < positionEpsilon * positionEpsilon &&
          Quaternion.Angle(rot, lastRot) < rotationEpsilonDeg)
        return;
    }

    lastPos = pos;
    lastRot = rot;
    hasLast = true;

    PublishPose(pos, rot);
  }

  void PublishPose(Vector3 unityPos, Quaternion unityRot)
  {
    Vector3 rosPos = UnityToRosPosition(unityPos);
    Quaternion rosRot = UnityToRosRotation(unityRot);

    double t = Time.realtimeSinceStartupAsDouble;
    int sec = (int)t;
    uint nanosec = (uint)((t - sec) * 1e9);

    var msg = new PoseStampedMsg
    {
      header = new HeaderMsg
      {
        stamp = new TimeMsg(sec, nanosec),
        frame_id = frameId
      },
      pose = new PoseMsg
      {
        position = new PointMsg(rosPos.x, rosPos.y, rosPos.z),
        orientation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
      }
    };

    ros.Publish(topicName, msg);

    if (enableLogs)
    {
      Debug.Log(
        $"[ROSPlanePublisher] pos_ros=({rosPos.x:F3},{rosPos.y:F3},{rosPos.z:F3}) " +
        $"quat=({rosRot.x:F3},{rosRot.y:F3},{rosRot.z:F3},{rosRot.w:F3})");
    }
  }

  /// <summary>Unity (x,y,z) → ROS (x, -z, y)  [inverse of mesh: ROS→Unity (x,z,-y)]</summary>
  public static Vector3 UnityToRosPosition(Vector3 unity)
  {
    return new Vector3(unity.x, -unity.z, unity.y);
  }

  /// <summary>Rotation consistent with UnityToRosPosition basis change.</summary>
  public static Quaternion UnityToRosRotation(Quaternion unityRot)
  {
    // R_ros = L^{-1} * R_unity * L, with L = Ros→Unity position map
    Matrix4x4 L = RosToUnityBasis;
    Matrix4x4 Linv = L.inverse;
    Matrix4x4 Ru = Matrix4x4.Rotate(unityRot);
    Matrix4x4 Rr = Linv * Ru * L;
    return Rr.rotation;
  }

  static Matrix4x4 CreateRosToUnityBasis()
  {
    // columns: L*(1,0,0)=(1,0,0), L*(0,1,0)=(0,0,-1), L*(0,0,1)=(0,1,0)
    var m = Matrix4x4.identity;
    m.SetColumn(0, new Vector4(1f, 0f, 0f, 0f));
    m.SetColumn(1, new Vector4(0f, 0f, -1f, 0f));
    m.SetColumn(2, new Vector4(0f, 1f, 0f, 0f));
    m.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
    return m;
  }
}
