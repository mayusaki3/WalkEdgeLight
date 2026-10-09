using System;
using System.Collections.Generic;
using NumericsVector3 = System.Numerics.Vector3;
using UnityEngine;
using WalkEdgeLight.Validation.Reconstruction;

namespace WalkEdgeLight.Validation.UnitySimulation
{
    public sealed class P0CRunner : MonoBehaviour
    {
        [SerializeField] private SingleStepScene sceneDefinition;
        [SerializeField] private Camera sensorCamera;
        [SerializeField] private int depthWidth = 160;
        [SerializeField] private int depthHeight = 120;
        [SerializeField] private float maxRangeMetres = 10f;

        [ContextMenu("Run P0-C")]
        public void Run()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-C references are not assigned.");

            sceneDefinition.Build();
            Physics.SyncTransforms();
            var truth = SingleStepGroundTruth.FromSceneDefinition(sceneDefinition);
            var depth = new CpuRaycastPerfectDepthGenerator().Generate(
                sensorCamera, depthWidth, depthHeight, maxRangeMetres);
            var frame = P0ASensorFrameAdapter.Create(
                0, Time.realtimeSinceStartupAsDouble, sensorCamera, depth);

            const float edgeExclusion = 0.10f;
            const float bandWidth = 0.60f;
            var near = new List<NumericsVector3>();
            var far = new List<NumericsVector3>();

            for (var v = 0; v < depthHeight; ++v)
            {
                for (var u = 0; u < depthWidth; ++u)
                {
                    var point = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                    if (!point.HasValue) continue;
                    var z = point.Value.Z;
                    if (z >= truth.EdgeZMetres - bandWidth &&
                        z <= truth.EdgeZMetres - edgeExclusion)
                        near.Add(point.Value);
                    else if (z >= truth.EdgeZMetres + edgeExclusion &&
                             z <= truth.EdgeZMetres + bandWidth)
                        far.Add(point.Value);
                }
            }

            if (near.Count < 3 || far.Count == 0)
                throw new InvalidOperationException("Insufficient P0-C point samples.");

            var ground = GroundPlaneEstimator.FitLeastSquares(near);
            double distanceSum = 0.0;
            foreach (var point in far)
                distanceSum += ground.SignedDistance(point);

            var measuredMetres = distanceSum / far.Count;
            var errorMillimetres = (measuredMetres - truth.HeightDifferenceMetres) * 1000.0;
            var result = measuredMetres < 0 ? "STEP_DOWN" : "STEP_UP";

            Debug.Log("[WalkEdgeLight P0-C] " +
                "nearSamples=" + near.Count +
                ", farSamples=" + far.Count +
                ", stepTruth=" + (truth.HeightDifferenceMetres * 1000.0).ToString("F3") + " mm" +
                ", stepMeasured=" + (measuredMetres * 1000.0).ToString("F6") + " mm" +
                ", stepError=" + errorMillimetres.ToString("F6") + " mm" +
                ", classification=" + result);
        }
    }
}
