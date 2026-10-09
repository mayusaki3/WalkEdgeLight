using System;
using System.Collections.Generic;
using NumericsVector3 = System.Numerics.Vector3;
using UnityEngine;
using WalkEdgeLight.Validation.Reconstruction;

namespace WalkEdgeLight.Validation.UnitySimulation
{
    public sealed class P0DRunner : MonoBehaviour
    {
        [SerializeField] private SingleStepScene sceneDefinition;
        [SerializeField] private Camera sensorCamera;
        private const int Strips = 8;
        private const int Bins = 80;
        private const float BinWidth = 0.025f;

        [ContextMenu("Run P0-D")]
        public void Run()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-D references missing.");
            sceneDefinition.Build();
            Physics.SyncTransforms();
            var depth = new CpuRaycastPerfectDepthGenerator().Generate(sensorCamera, 160, 120, 10f);
            var frame = P0ASensorFrameAdapter.Create(
                0, Time.realtimeSinceStartupAsDouble, sensorCamera, depth);
            var cells = new List<NumericsVector3>[Strips, Bins];
            for (int s = 0; s < Strips; ++s)
                for (int b = 0; b < Bins; ++b)
                    cells[s, b] = new List<NumericsVector3>();

            var groundPoints = new List<NumericsVector3>();
            for (int v = 0; v < 120; ++v)
                for (int u = 0; u < 160; ++u)
                {
                    var p = PointReconstructor.ReconstructWorldPoint(frame, u, v);
                    if (!p.HasValue) continue;
                    var s = (int)Math.Floor((p.Value.X + 1f) / 0.25f);
                    var b = (int)Math.Floor(p.Value.Z / BinWidth);
                    if (p.Value.Z >= 0 && p.Value.Z < 0.4f) groundPoints.Add(p.Value);
                    if (s >= 0 && s < Strips && b >= 0 && b < Bins)
                        cells[s, b].Add(p.Value);
                }
            if (groundPoints.Count < 3) throw new InvalidOperationException("Insufficient ground points.");
            var plane = GroundPlaneEstimator.FitLeastSquares(groundPoints);
            var xs = new List<double>();
            var zs = new List<double>();
            for (int s = 0; s < Strips; ++s)
            {
                var means = new double[Bins];
                var valid = new bool[Bins];
                for (int b = 0; b < Bins; ++b)
                {
                    if (cells[s, b].Count < 3) continue;
                    double sum = 0;
                    foreach (var p in cells[s, b]) sum += plane.SignedDistance(p);
                    means[b] = sum / cells[s, b].Count;
                    valid[b] = true;
                }
                var best = 0.010;
                var bestBin = -1;
                for (int b = 16; b < Bins; ++b)
                {
                    double before = 0, after = 0;
                    int nb = 0, na = 0;
                    for (int j = b - 1; j >= Math.Max(0, b - 8) && nb < 2; --j)
                        if (valid[j]) { before += means[j]; ++nb; }
                    for (int j = b; j < Math.Min(Bins, b + 8) && na < 2; ++j)
                        if (valid[j]) { after += means[j]; ++na; }
                    if (nb < 2 || na < 2) continue;
                    var delta = after / na - before / nb;
                    if (delta < -best)
                    {
                        best = -delta;
                        bestBin = b;
                    }
                }
                if (bestBin < 0) continue;
                xs.Add(-1.0 + (s + 0.5) * 0.25);
                zs.Add(bestBin * BinWidth);
            }
            if (xs.Count < 2)
            {
                Debug.Log("[WalkEdgeLight P0-D] detection=NONE, stripsDetected=" + xs.Count);
                return;
            }
            double mx = 0, mz = 0;
            for (int i = 0; i < xs.Count; ++i) { mx += xs[i]; mz += zs[i]; }
            mx /= xs.Count; mz /= zs.Count;
            double xx = 0, xz = 0, rms = 0;
            for (int i = 0; i < xs.Count; ++i)
            {
                xx += (xs[i] - mx) * (xs[i] - mx);
                xz += (xs[i] - mx) * (zs[i] - mz);
            }
            if (xx < 1e-9) throw new InvalidOperationException("Degenerate edge points.");
            var slope = xz / xx;
            var intercept = mz - slope * mx;
            for (int i = 0; i < xs.Count; ++i)
            {
                var residual = zs[i] - (slope * xs[i] + intercept);
                rms += residual * residual;
            }
            rms = Math.Sqrt(rms / xs.Count);
            var truth = SingleStepGroundTruth.FromSceneDefinition(sceneDefinition);
            Debug.Log("[WalkEdgeLight P0-D] detection=EDGE" +
                ", stripsDetected=" + xs.Count + "/" + Strips +
                ", slope=" + slope.ToString("F6") +
                ", edgeZAtX0=" + intercept.ToString("F4") + " m" +
                ", edgeTruth=" + truth.EdgeZMetres.ToString("F4") + " m" +
                ", positionError=" + ((intercept - truth.EdgeZMetres) * 1000).ToString("F3") + " mm" +
                ", lineRms=" + (rms * 1000).ToString("F3") + " mm");
        }
    }
}
