using System;
using UnityEditor;
using UnityEngine;
using WalkEdgeLight.Validation.UnitySimulation;

namespace WalkEdgeLight.Validation.Editor
{
    internal static class ProbeSetup
    {
        private const string RootName = "WalkEdgeLightP0AProbe";

        [MenuItem("WalkEdgeLight/Probe/Setup P0-A")]
        internal static void Setup()
        {
            try
            {
                GameObject previous = GameObject.Find(RootName);
                if (previous != null)
                {
                    Undo.DestroyObjectImmediate(previous);
                }

                var root = new GameObject(RootName);
                Undo.RegisterCreatedObjectUndo(root, "Create WalkEdgeLight P0-A Probe");

                var sceneObject = new GameObject("SingleStepScene");
                sceneObject.transform.SetParent(root.transform, false);
                var scene = Undo.AddComponent<SingleStepScene>(sceneObject);

                var cameraObject = new GameObject("SensorCamera");
                cameraObject.transform.SetParent(root.transform, false);
                cameraObject.transform.position = new Vector3(0f, 1.2f, 0f);
                cameraObject.transform.rotation = Quaternion.Euler(45f, 0f, 0f);

                var camera = Undo.AddComponent<Camera>(cameraObject);
                camera.fieldOfView = 60f;
                camera.nearClipPlane = 0.01f;
                camera.farClipPlane = 10f;

                var runner = Undo.AddComponent<P0ARunner>(root);
                SetObject(runner, "sceneDefinition", scene);
                SetObject(runner, "sensorCamera", camera);

                var groundPlaneRunner = Undo.AddComponent<P0BRunner>(root);
                SetObject(groundPlaneRunner, "sceneDefinition", scene);
                SetObject(groundPlaneRunner, "sensorCamera", camera);

                var stepRunner = Undo.AddComponent<P0CRunner>(root);
                SetObject(stepRunner, "sceneDefinition", scene);
                SetObject(stepRunner, "sensorCamera", camera);

                var edgeRunner = Undo.AddComponent<P0DRunner>(root);
                SetObject(edgeRunner, "sceneDefinition", scene);
                SetObject(edgeRunner, "sensorCamera", camera);

                scene.Build();
                Physics.SyncTransforms();

                EditorUtility.SetDirty(root);
                Selection.activeGameObject = root;

                Log("SETUP.CAMERA_HEIGHT_M", "1.2");
                Log("SETUP.STEP_HEIGHT_MM", "20");
                Log("SETUP.DEPTH_RESOLUTION", "160x120");
                Log("SETUP.RESULT", "PASS");
            }
            catch (Exception exception)
            {
                Debug.LogError($"WALKEDGE_PROBE|SETUP.RESULT|FAIL|{exception.Message}");
                throw;
            }
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-A")]
        internal static void RunP0A()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run WalkEdgeLight > Probe > Setup P0-A first");

            P0ARunner runner = root.GetComponent<P0ARunner>();
            Require(runner != null, "P0ARunner component is missing");
            runner.Run();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-B")]
        internal static void RunP0B()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run WalkEdgeLight > Probe > Setup P0-A first");

            P0BRunner runner = root.GetComponent<P0BRunner>();
            Require(runner != null, "P0BRunner component is missing; rerun Setup P0-A");
            runner.Run();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-B Slope")]
        internal static void RunP0BSlope()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run WalkEdgeLight > Probe > Setup P0-A first");

            P0BRunner runner = root.GetComponent<P0BRunner>();
            Require(runner != null, "P0BRunner component is missing; rerun Setup P0-A");
            runner.RunSlope();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-C")]
        internal static void RunP0C()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            P0CRunner runner = root.GetComponent<P0CRunner>();
            Require(runner != null, "P0CRunner is missing; rerun Setup P0-A");
            runner.Run();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-C Auto")]
        internal static void RunP0CAuto()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            P0CRunner runner = root.GetComponent<P0CRunner>();
            Require(runner != null, "P0CRunner is missing; rerun Setup P0-A");
            runner.RunAuto();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-D")]
        internal static void RunP0D()
        {
            GameObject root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            P0DRunner runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing; rerun Setup P0-A");
            runner.Run();
        }

        private static void SetObject(
            UnityEngine.Object target,
            string propertyName,
            UnityEngine.Object value)
        {
            var serialized = new SerializedObject(target);
            SerializedProperty property = serialized.FindProperty(propertyName);
            Require(property != null, $"Serialized property is missing: {propertyName}");
            property.objectReferenceValue = value;
            serialized.ApplyModifiedProperties();
        }

        private static void Require(bool condition, string message)
        {
            if (!condition)
            {
                throw new InvalidOperationException(message);
            }
        }

        private static void Log(string key, object value)
        {
            Debug.Log($"WALKEDGE_PROBE|{key}|{value}");
        }
    }
}
