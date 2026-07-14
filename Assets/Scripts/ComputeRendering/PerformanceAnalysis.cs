using System;
using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;

namespace ComputeRendering {
    [Serializable]
    public class PerformanceData {
        public string name;
        public List<float> tickTimes;
        public float lastTickTime;
    }
    [Serializable]
    public class PerformanceDataWrapper {
        public List<PerformanceData> data;
    }
    public class PerformanceAnalysis {
        private string fileDateFormat = "yyyy_MM_dd_HH_mm_ss";
        private Dictionary<string, PerformanceData> batchData = new Dictionary<string, PerformanceData>();
        private Dictionary<string, float> records = new Dictionary<string, float>();

        public void Tick(string key) {
            float currentTime = Time.realtimeSinceStartup;
            PerformanceData performanceData;
            if (this.batchData.TryGetValue(key, out var value)) {
                performanceData = value;
            }
            else {
                performanceData = new PerformanceData();
                performanceData.name = key;
                performanceData.tickTimes = new List<float>();
                performanceData.lastTickTime = currentTime;
                this.batchData.Add(key, performanceData);
            }
            
            float deltaTime = currentTime - performanceData.lastTickTime;
            performanceData.lastTickTime = currentTime;

            performanceData.tickTimes.Add(deltaTime);
        }

        public void StartRecord(string key) {
            this.records[key] = Time.realtimeSinceStartup;
        }

        public void StopRecord(string key) {
            if (this.records.TryGetValue(key, out float currentTime)) {
                this.records.Remove(key);
                PerformanceData performanceData;
                if (this.batchData.TryGetValue(key, out var value)) {
                    performanceData = value;
                }
                else {
                    performanceData = new PerformanceData();
                    performanceData.name = key;
                    performanceData.tickTimes = new List<float>();
                    performanceData.lastTickTime = currentTime;
                    this.batchData.Add(key, performanceData);
                }
            
                float deltaTime = currentTime - performanceData.lastTickTime;
                performanceData.lastTickTime = currentTime;

                performanceData.tickTimes.Add(deltaTime);
            }
            
        }

        public void SaveToFile() {
            try {
                string logFilePath = Path.ChangeExtension($"PerformanceAnalysis/{DateTime.Now.ToString(fileDateFormat)}", ".json");
                PerformanceDataWrapper wrapper = new PerformanceDataWrapper { data = new List<PerformanceData>(this.batchData.Values) };
                string json = JsonUtility.ToJson(wrapper, true);
                Debug.Log(json);
                File.WriteAllText(logFilePath, json);

                Debug.Log($"[PerformanceAnalysis] Data saved to {logFilePath}");
            } catch (Exception ex) {
                Debug.LogError($"Error saving performance data: {ex.Message}");
            }
        }
    }
}
