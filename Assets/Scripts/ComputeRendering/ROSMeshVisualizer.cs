using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Visualization;
using UnityEngine.Serialization;

namespace ComputeRendering {
    public struct RenderObject {
        public GameObject gameObject;
        public Mesh mesh;
    }
    
    public class ROSMeshVisualizer : MonoBehaviour {
        public GameObject renderTargetObject;
        public string rosTopicName = "/object_markers";
        public int meshPerMessage = 1; // how many meshes to process per message and been shown
        public bool enableAnalysis = false;

        private RenderObject[] renderObjects;
        private Queue<MarkerArrayMsg> messageQueue = new Queue<MarkerArrayMsg>();
        private bool semaphore = true;
        private ROSConnection rosConnection;
        private Material meshMaterial;
        
        private PerformanceAnalysis performanceAnalysis;

        void Start() {
            this.performanceAnalysis = new PerformanceAnalysis();
            this.rosConnection = ROSConnection.GetOrCreateInstance();
            if (this.rosConnection == null) {
                Debug.LogError("ROSConnection init failed.");
                return;
            }

            this.meshMaterial = new Material(Shader.Find("Unlit/VertexColor"));
            if (this.meshMaterial == null) {
                Debug.LogError("UnlitVertexColorShader not found.");
                return;
            }

            this.renderObjects = new RenderObject[this.meshPerMessage];
            GameObject parentMeshObject = this.renderTargetObject == null
                ? new GameObject("VisualizedMesh")
                : this.renderTargetObject;
            for (int i = 0; i < this.meshPerMessage; i++) {
                this.renderObjects[i] = new RenderObject {
                    gameObject = new GameObject(i.ToString().PadLeft(this.meshPerMessage.ToString().Length, '0')),
                };
                this.renderObjects[i].gameObject.AddComponent<MeshFilter>();
                this.renderObjects[i].gameObject.transform.SetParent(parentMeshObject.transform);
                this.renderObjects[i].gameObject.transform.localPosition = Vector3.zero;
                this.renderObjects[i].gameObject.transform.localRotation = Quaternion.identity;
                MeshRenderer meshRenderer = this.renderObjects[i].gameObject.AddComponent<MeshRenderer>();
                meshRenderer.material = this.meshMaterial;
                this.renderObjects[i].mesh = new Mesh {
                    indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
                };
            }

            rosConnection.Subscribe<MarkerArrayMsg>(rosTopicName, OnMeshReceived);
        }

        void OnMeshReceived(MarkerArrayMsg markerArrayMsg) {
            if (markerArrayMsg == null || markerArrayMsg.markers.Length == 0) {
                Debug.LogWarning("MarkerArrayMsg is empty.");
                return;
            }

            this.messageQueue.Enqueue(markerArrayMsg);
            if (this.enableAnalysis) {
                this.performanceAnalysis.Tick("message-received");
            }
        }

        void Update() {
            if (this.messageQueue.Count == 0) {
                return;
            }

            this.UpdateQueue();
        }


        void UpdateQueue() {
            if (!this.semaphore) {
                return;
            }

            this.semaphore = false;
            Debug.Log("Processing message. Messages remaining " + this.messageQueue.Count);
            MarkerArrayMsg markerArrayMsg = this.messageQueue.Dequeue();
            bool processed = false;
            for (int index = 0; index < this.renderObjects.Length; index++) {
                var renderObject = this.renderObjects[index];
                renderObject.gameObject.SetActive(true);
                var marker = markerArrayMsg.markers[index];
                if (marker == null || 
                    marker.type != MarkerMsg.TRIANGLE_LIST || 
                    marker.points == null ||
                    marker.points.Length == 0) {
                    Debug.LogWarning("Marker " + index.ToString() + " is invalid or is empty.");
                    renderObject.gameObject.SetActive(false);
                    continue;
                }
                processed = true;
                StartCoroutine(ProcessMarker(marker, renderObject));
            }

            if (!processed) {
                this.semaphore = true;
            }
        }

        IEnumerator ProcessMarker(MarkerMsg marker, RenderObject renderObject) {
            if (this.enableAnalysis) {
                this.performanceAnalysis.StartRecord("mesh-processing");
            }

            int vertexCount = marker.points.Length;
            Color lastColor = Color.cyan;
            Debug.Log("Processing " + vertexCount + " vertices.");
            
            Vector3[] vertices = new Vector3[vertexCount];
            Color[] vertexColors = new Color[vertexCount];
            List<int> triangles = new List<int>();

            for (int i = 0; i < vertexCount; i++) {
                vertices[i] = new Vector3(
                    (float)marker.points[i].x, 
                    (float)marker.points[i].z,
                    (float)marker.points[i].y * (-1f) // ROS z is inverted to Unity y
                );
                
                if (marker.colors != null && marker.colors.Length > i) {
                    var rosColor = marker.colors[i];
                    lastColor = new Color(
                        (float)rosColor.r,
                        (float)rosColor.g,
                        (float)rosColor.b,
                        (float)rosColor.a
                    );
                    vertexColors[i] = lastColor;
                }
                else {
                    vertexColors[i] = lastColor;
                }
                
                if (i % 3 == 0 && i + 2 < vertexCount) {
                    triangles.Add(i);
                    triangles.Add(i + 1);
                    triangles.Add(i + 2);
                }
                
                if (i % 300 == 0) {
                    yield return null;
                }
            }
            
            renderObject.mesh.Clear();
            renderObject.mesh.vertices = vertices;
            renderObject.mesh.triangles = triangles.ToArray();
            renderObject.mesh.colors = vertexColors;
            renderObject.mesh.RecalculateNormals();
            renderObject.mesh.RecalculateBounds();
            
            if (this.enableAnalysis) {
                this.performanceAnalysis.StopRecord("mesh-processing");
            }
            
            var meshFilter = renderObject.gameObject.GetComponent<MeshFilter>();
            if (meshFilter != null) {
                meshFilter.mesh = renderObject.mesh;
            }
            else {
                Debug.LogError("MeshFilter not found.");
                StartCoroutine(ResetSemaphore(0));
            } 
            StartCoroutine(ResetSemaphore());
        }
        
        IEnumerator ResetSemaphore(float seconds = 0.1f) {
            yield return new WaitForSeconds(seconds);
            this.semaphore = true;
            Debug.Log("Semaphore reset.");
        }

        void OnApplicationQuit() {
            if (rosConnection != null) {
                rosConnection.Unsubscribe(rosTopicName);
            }
        }

        void OnDestroy() {
            if (rosConnection != null) {
                rosConnection.Unsubscribe(rosTopicName);
            }
            if (this.enableAnalysis) {
                this.performanceAnalysis.SaveToFile();
            }
        }
    }
}
