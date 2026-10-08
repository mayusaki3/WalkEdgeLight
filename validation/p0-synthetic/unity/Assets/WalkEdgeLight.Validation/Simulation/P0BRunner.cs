using System;
using System.Collections.Generic;
using NumericsVector3 = System.Numerics.Vector3;
using UnityEngine;
using WalkEdgeLight.Validation.Reconstruction;

namespace WalkEdgeLight.Validation.UnitySimulation
{
    public sealed class P0BRunner : MonoBehaviour
    {
        [SerializeField] private SingleStepScene sceneDefinition;
        [SerializeField] private Camera sensorCamera;
        [SerializeField] private int depthWidth = 160;
        [SerializeField] private int depthHeight = 120;
        [SerializeField] private float maxRangeMetres = 10f;

        [ContextMenu("Run P0-B")]
        public void Run()
        {
            RunInternal(false);
        }

        [ContextMenu("Run P0-B Slope")]
        public void RunSlope()
        {
            RunInternal(true);
        }

        private void RunInternal(bool slope)
        {
            if (sceneDefinition == null) throw new InvalidOperationException("SingleStepScene is not assigned.");
            if (sensorCamera == null) throw new InvalidOperationException("Sensor Camera is not assigned.");

            sceneDefinition.Build();
            if (slope)
            {
                var nearFloor = sceneDefinition.transform.Find("NearFloor");
                if (nearFloor == null) throw new InvalidOperationException("NearFloor is missing.");
                nearFloor.localRotation = Quaternion.Euler(8f, 0f, 0f);
            }
            Physics.SyncTransforms();

            var truth = SingleStepGroundTruth.FromSceneDefinition(sceneDefinition);
            var generated = new CpuRaycastPerfectDepthGenerator().Generate(
                sensorCamera, depthWidth, depthHeight, maxRangeMetres);
            var frame = P0ASensorFrameAdapter.Create(
                0, Time.realtimeSinceStartupAsDouble, sensorCamera, generated);

            const float edgeExclusionMetres = 0.10f;
            const float sampleBandMetres = 0.60f;
            var samples = new List<NumericsVector3>();

            for (var v = 0; v < depthHeight; ++v)
            {
                for (var u = 0; u < depthWidth; ++u)
                {
                    var world = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                    if (!world.HasValue) continue;

                    var z = world.Value.Z;
                    if (z >= truth.EdgeZMetres - sampleBandMetres &&
                        z <= truth.EdgeZMetres - edgeExclusionMetres)
                    {
                        samples.Add(world.Value);
                    }
                }
            }

            var plane = GroundPlaneEstimator.FitLeastSquares(samples);
            var expectedUnity = slope
                ? sceneDefinition.transform.Find("NearFloor").up
                : UnityEngine.Vector3.up;
            var expectedNormal = new NumericsVector3(expectedUnity.x, expectedUnity.y, expectedUnity.z);
            var dot = Math.Max(-1.0, Math.Min(1.0, NumericsVector3.Dot(plane.Normal, expectedNormal)));
            var normalErrorDegrees = Math.Acos(dot) * 180.0 / Math.PI;
            var groundHeight = -plane.Offset / plane.Normal.Y;

            double squaredErrorSum = 0.0;
            foreach (var point in samples)
            {
                var distance = plane.SignedDistance(point);
                squaredErrorSum += distance * distance;
            }
            var rmsMillimetres = Math.Sqrt(squaredErrorSum / samples.Count) * 1000.0;

            Debug.Log(
                (slope ? "[WalkEdgeLight P0-B Slope] samples=" : "[WalkEdgeLight P0-B] samples=") + samples.Count +
                ", normal=(" +
                plane.Normal.X.ToString("F8") + "," +
                plane.Normal.Y.ToString("F8") + "," +
                plane.Normal.Z.ToString("F8") + ")" +
                ", normalError=" + normalErrorDegrees.ToString("F8") + " deg" +
                ", groundHeight=" + (groundHeight * 1000.0).ToString("F6") + " mm" +
                ", planeRms=" + rmsMillimetres.ToString("F6") + " mm");

            if (slope)
            {
                // Restore the standard scene for subsequent P0-A/P0-B runs.
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }
    }
}
