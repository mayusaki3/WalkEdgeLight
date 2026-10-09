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


        [ContextMenu("Run P0-C Auto")]
        public void RunAuto()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-C references are not assigned.");

            sceneDefinition.Build();
            Physics.SyncTransforms();
            var depth = new CpuRaycastPerfectDepthGenerator().Generate(
                sensorCamera, depthWidth, depthHeight, maxRangeMetres);
            var frame = P0ASensorFrameAdapter.Create(
                0, Time.realtimeSinceStartupAsDouble, sensorCamera, depth);

            const float binWidth = 0.025f;
            const int binCount = 80;
            var bins = new List<NumericsVector3>[binCount];
            for (var i = 0; i < binCount; ++i) bins[i] = new List<NumericsVector3>();
            for (var v = 0; v < depthHeight; ++v)
                for (var u = 0; u < depthWidth; ++u)
                {
                    var p = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                    if (!p.HasValue) continue;
                    var bin = (int)Math.Floor(p.Value.Z / binWidth);
                    if (bin >= 0 && bin < binCount) bins[bin].Add(p.Value);
                }

            // Fixed near-field reference: independent of the known edge location.
            var reference = new List<NumericsVector3>();
            for (var i = 0; i < 16; ++i) reference.AddRange(bins[i]);
            if (reference.Count < 3) throw new InvalidOperationException("Insufficient reference ground.");
            var plane = GroundPlaneEstimator.FitLeastSquares(reference);

            var means = new double[binCount];
            var valid = new bool[binCount];
            for (var i = 0; i < binCount; ++i)
            {
                if (bins[i].Count < 10) continue;
                double sum = 0;
                foreach (var p in bins[i]) sum += plane.SignedDistance(p);
                means[i] = sum / bins[i].Count;
                valid[i] = true;
            }

            // Allow empty depth bins near occlusion boundaries.
            // Use the closest two populated bins on each side, within 0.20 m.
            const int searchRadius = 8;
            const int requiredBins = 2;
            const double threshold = 0.010;
            var bestScore = threshold;
            var bestBin = -1;
            var bestDelta = 0.0;
            var evaluatedCandidates = 0;
            var validBinCount = 0;
            for (var i = 0; i < binCount; ++i)
                if (valid[i]) ++validBinCount;

            for (var i = 16; i < binCount; ++i)
            {
                double before = 0, after = 0;
                var beforeCount = 0;
                var afterCount = 0;
                for (var j = 1; j <= searchRadius && i - j >= 0; ++j)
                {
                    if (!valid[i - j]) continue;
                    before += means[i - j];
                    if (++beforeCount == requiredBins) break;
                }
                for (var j = 0; j < searchRadius && i + j < binCount; ++j)
                {
                    if (!valid[i + j]) continue;
                    after += means[i + j];
                    if (++afterCount == requiredBins) break;
                }
                if (beforeCount < requiredBins || afterCount < requiredBins) continue;
                ++evaluatedCandidates;
                var delta = after / afterCount - before / beforeCount;
                if (Math.Abs(delta) > bestScore)
                {
                    bestScore = Math.Abs(delta);
                    bestBin = i;
                    bestDelta = delta;
                }
            }

            if (bestBin < 0)
            {
                Debug.Log("[WalkEdgeLight P0-C Auto] detection=NONE" +
                    ", validBins=" + validBinCount +
                    ", evaluatedCandidates=" + evaluatedCandidates +
                    ", maxAbsDeltaMm=" + (bestScore * 1000.0).ToString("F3"));
                return;
            }

            var edgeMetres = bestBin * binWidth;
            // Ground truth is read only after detection for validation.
            var truth = SingleStepGroundTruth.FromSceneDefinition(sceneDefinition);
            Debug.Log("[WalkEdgeLight P0-C Auto] detection=" +
                (bestDelta < 0 ? "STEP_DOWN" : "STEP_UP") +
                ", edgeZ=" + edgeMetres.ToString("F4") + " m" +
                ", edgeTruth=" + truth.EdgeZMetres.ToString("F4") + " m" +
                ", edgeError=" + ((edgeMetres - truth.EdgeZMetres) * 1000.0).ToString("F3") + " mm" +
                ", heightDelta=" + (bestDelta * 1000.0).ToString("F3") + " mm" +
                ", validBins=" + validBinCount +
                ", evaluatedCandidates=" + evaluatedCandidates);
        }

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
