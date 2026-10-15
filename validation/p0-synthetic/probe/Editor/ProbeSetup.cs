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

        [MenuItem("WalkEdgeLight/Probe/Run P0-D Slanted")]
        internal static void RunP0DSlanted()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunSlanted();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Offset")]
        internal static void RunP0EOffset()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunOffset();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Sweep")]
        internal static void RunP0ESweep()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunSweep();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Resolution")]
        internal static void RunP0EResolution()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunResolution();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Pose")]
        internal static void RunP0EPose()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunPose();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Bound Diagnostic")]
        internal static void RunP0EBoundDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunBoundDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Tolerance")]
        internal static void RunP0ETolerance()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunTolerance();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-E Bounds Regression")]
        internal static void RunP0EBoundsRegression()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunBoundsRegression();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Gaussian")]
        internal static void RunP0FGaussian()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunGaussianNoise();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Noise Bounds Diagnostic")]
        internal static void RunP0FNoiseBoundsDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunNoiseBoundsDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Percentile Comparison")]
        internal static void RunP0FPercentileComparison()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunPercentileComparison();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Quantile Seed Sweep")]
        internal static void RunP0FQuantileSeedSweep()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunQuantileSeedSweep();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Default Regression")]
        internal static void RunP0FDefaultRegression()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunDefaultRegression();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Dropout")]
        internal static void RunP0FDropout()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunDropout();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Projected Far Quantile")]
        internal static void RunP0FProjectedFarQuantile()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunProjectedFarQuantile();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Far Quantile Accuracy")]
        internal static void RunP0FFarQuantileAccuracy()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunFarQuantileAccuracy();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Far Quantile Diagnostic")]
        internal static void RunP0FFarQuantileDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunFarQuantileDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Outlier Diagnostic")]
        internal static void RunP0FOutlierDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunOutlierDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Correlated Band Sequence Diagnostic")]
        internal static void RunP0FCorrelatedBandSequenceDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandSequenceDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Correlated Band Failure Detail")]
        internal static void RunP0FCorrelatedBandFailureDetail()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandFailureDetail();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Split Support Generalization")]
        internal static void RunP0FSplitSupportGeneralization()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunSplitSupportGeneralizationDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Split Support Diagnostic")]
        internal static void RunP0FSplitSupportDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandSplitSupportDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Spatial Consensus Diagnostic")]
        internal static void RunP0FSpatialConsensusDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandSpatialConsensusDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Cross-Session Fingerprint")]
        internal static void RunP0FCrossSessionFingerprint()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandCrossSessionFingerprint();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Repeatability Diagnostic")]
        internal static void RunP0FRepeatabilityDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandRepeatability();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Correlated Band Stress")]
        internal static void RunP0FCorrelatedBandStress()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunCorrelatedBandStress();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Spatial Recovery Stress")]
        internal static void RunP0FSpatialRecoveryStress()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunSpatialRecoveryStress();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Upper Recovery Diagnostic")]
        internal static void RunP0FUpperRecoveryDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunUpperRecoveryDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Upper Disagreement Diagnostic")]
        internal static void RunP0FUpperDisagreementDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunUpperDisagreementDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Stress Failure Detail")]
        internal static void RunP0FStressFailureDetail()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunStressFailureDetail();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Cluster Stress Diagnostic")]
        internal static void RunP0FClusterStressDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunClusterStressDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Supported Interval Diagnostic")]
        internal static void RunP0FSupportedIntervalDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunSupportedIntervalDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Concentrated Failure Detail")]
        internal static void RunP0FConcentratedFailureDetail()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunConcentratedFailureDetail();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Concentrated Cluster Diagnostic")]
        internal static void RunP0FConcentratedClusterDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunConcentratedClusterDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Clustered Quorum Diagnostic")]
        internal static void RunP0FClusteredQuorumDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunClusteredQuorumDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Clustered Failure Diagnostic")]
        internal static void RunP0FClusteredFailureDiagnostic()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunClusteredFailureDiagnostic();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Clustered Outliers")]
        internal static void RunP0FClusteredOutliers()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunClusteredOutliers();
        }

        [MenuItem("WalkEdgeLight/Probe/Run P0-F Outliers")]
        internal static void RunP0FOutliers()
        {
            var root = GameObject.Find(RootName);
            Require(root != null, "Run Setup P0-A first");
            var runner = root.GetComponent<P0DRunner>();
            Require(runner != null, "P0DRunner missing");
            runner.RunOutliers();
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
