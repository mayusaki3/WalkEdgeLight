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
        private double lastLowerBound = double.NaN;
        private double lastUpperBound = double.NaN;
        private double lastProjectedUpper = double.NaN;
        private double lastProjectedMidpointError = double.NaN;
        private const int Strips = 8;
        private const int Bins = 80;
        private const float BinWidth = 0.025f;

        [ContextMenu("Run P0-D")]
        public void Run()
        {
            RunInternal(false);
        }

        [ContextMenu("Run P0-D Slanted")]
        public void RunSlanted()
        {
            RunInternal(true);
        }

        [ContextMenu("Run P0-E Offset")]
        public void RunOffset()
        {
            RunInternal(false, 0.013f);
        }

        [ContextMenu("Run P0-E Sweep")]
        public void RunSweep()
        {
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int detected = 0, covered = 0, bounded = 0, projectedCovered = 0, projectedBounded = 0;
            double projectedWidthSum = 0, projectedAbsSum = 0, projectedMaxAbs = 0;
            int projectedEstimates = 0;
            double sumWidth = 0;
            double sumAbs = 0, maxAbs = 0;
            foreach (float offset in offsets)
            {
                double error = RunInternal(false, offset, false);
                if (double.IsNaN(error))
                {
                    Debug.Log("[WalkEdgeLight P0-E Sweep] edgeTruth=" +
                        (sceneDefinition.EdgeZMetres + offset).ToString("F4") + " m, detection=NONE");
                    continue;
                }
                ++detected;
                double truthZ = sceneDefinition.EdgeZMetres + offset;
                bool hasBounds = !double.IsNaN(lastLowerBound) && !double.IsNaN(lastUpperBound);
                bool containsTruth = hasBounds && lastLowerBound <= truthZ && truthZ <= lastUpperBound;
                if (hasBounds) { ++bounded; sumWidth += (lastUpperBound - lastLowerBound) * 1000.0; }
                if (containsTruth) ++covered;
                bool hasProjected = hasBounds && !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                bool projectedContains = hasProjected && lastLowerBound <= truthZ && truthZ <= lastProjectedUpper;
                if (hasProjected) { ++projectedBounded; projectedWidthSum += (lastProjectedUpper - lastLowerBound) * 1000.0; }
                if (projectedContains) ++projectedCovered;
                if (hasProjected && !double.IsNaN(lastProjectedMidpointError))
                {
                    ++projectedEstimates;
                    double projectedAbs = Math.Abs(lastProjectedMidpointError);
                    projectedAbsSum += projectedAbs;
                    projectedMaxAbs = Math.Max(projectedMaxAbs, projectedAbs);
                }
                double absolute = Math.Abs(error);
                sumAbs += absolute;
                maxAbs = Math.Max(maxAbs, absolute);
                Debug.Log("[WalkEdgeLight P0-E Sweep] edgeTruth=" +
                    (sceneDefinition.EdgeZMetres + offset).ToString("F4") +
                    " m, positionError=" + error.ToString("F3") + " mm" +
                    ", boundNear=" + (hasBounds ? lastLowerBound.ToString("F6") : "N/A") + " m" +
                    ", boundFar=" + (hasBounds ? lastUpperBound.ToString("F6") : "N/A") + " m" +
                    ", boundWidth=" + (hasBounds ? ((lastUpperBound - lastLowerBound) * 1000).ToString("F3") : "N/A") + " mm" +
                    ", containsTruth=" + containsTruth +
                    ", projectedUpper=" + (hasProjected ? lastProjectedUpper.ToString("F6") : "N/A") + " m" +
                    ", projectedWidth=" + (hasProjected ? ((lastProjectedUpper - lastLowerBound) * 1000).ToString("F3") : "N/A") + " mm" +
                    ", projectedContainsTruth=" + projectedContains +
                    ", projectedMidpointError=" + (hasProjected ? lastProjectedMidpointError.ToString("F3") : "N/A") + " mm");
            }
            sceneDefinition.Build();
            Physics.SyncTransforms();
            Debug.Log("[WalkEdgeLight P0-E Sweep] summary=" + detected + "/" + offsets.Length +
                ", meanAbsError=" + (detected > 0 ? (sumAbs / detected).ToString("F3") : "N/A") +
                " mm, maxAbsError=" + (detected > 0 ? maxAbs.ToString("F3") : "N/A") +
                " mm, bounded=" + bounded + "/" + detected +
                ", coverage=" + covered + "/" + offsets.Length +
                ", meanBoundWidth=" + (bounded > 0 ? (sumWidth / bounded).ToString("F3") : "N/A") +
                " mm, projectedBounded=" + projectedBounded + "/" + detected +
                ", projectedCoverage=" + projectedCovered + "/" + offsets.Length +
                ", meanProjectedWidth=" + (projectedBounded > 0 ? (projectedWidthSum / projectedBounded).ToString("F3") : "N/A") +
                " mm, projectedMidpointMAE=" + (projectedEstimates > 0 ? (projectedAbsSum / projectedEstimates).ToString("F3") : "N/A") +
                " mm, projectedMidpointMaxAbsError=" + (projectedEstimates > 0 ? projectedMaxAbs.ToString("F3") : "N/A") + " mm");
        }

        [ContextMenu("Run P0-E Resolution")]
        public void RunResolution()
        {
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] widths = { 160, 320 };
            foreach (int width in widths)
            {
                int detected = 0, bounded = 0, covered = 0;
                double sumError = 0, maxError = 0, sumWidth = 0;
                foreach (float offset in offsets)
                {
                    double originalError = RunInternal(false, offset, false, width, width * 3 / 4);
                    double truthZ = sceneDefinition.EdgeZMetres + offset;
                    bool hasBounds = !double.IsNaN(lastLowerBound) &&
                        !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                    bool contains = hasBounds && truthZ >= lastLowerBound && truthZ <= lastProjectedUpper;
                    if (!double.IsNaN(originalError)) ++detected;
                    if (hasBounds)
                    {
                        ++bounded;
                        sumWidth += (lastProjectedUpper - lastLowerBound) * 1000.0;
                        double error = Math.Abs(lastProjectedMidpointError);
                        sumError += error;
                        maxError = Math.Max(maxError, error);
                    }
                    if (contains) ++covered;
                    Debug.Log("[WalkEdgeLight P0-E Resolution] size=" + width + "x" + (width * 3 / 4) +
                        ", edgeTruth=" + truthZ.ToString("F4") + " m" +
                        ", detected=" + !double.IsNaN(originalError) +
                        ", projectedWidth=" + (hasBounds ? ((lastProjectedUpper - lastLowerBound) * 1000).ToString("F3") : "N/A") + " mm" +
                        ", projectedMidpointError=" + (hasBounds ? lastProjectedMidpointError.ToString("F3") : "N/A") + " mm" +
                        ", containsTruth=" + contains);
                }
                Debug.Log("[WalkEdgeLight P0-E Resolution] summary size=" + width + "x" + (width * 3 / 4) +
                    ", detected=" + detected + "/" + offsets.Length +
                    ", bounded=" + bounded + "/" + offsets.Length +
                    ", coverage=" + covered + "/" + offsets.Length +
                    ", meanProjectedWidth=" + (bounded > 0 ? (sumWidth / bounded).ToString("F3") : "N/A") + " mm" +
                    ", projectedMidpointMAE=" + (bounded > 0 ? (sumError / bounded).ToString("F3") : "N/A") + " mm" +
                    ", projectedMidpointMaxAbsError=" + (bounded > 0 ? maxError.ToString("F3") : "N/A") + " mm");
            }
            sceneDefinition.Build();
            Physics.SyncTransforms();
        }

        private double RunInternal(bool slanted, float edgeOffset = 0f, bool verbose = true, int imageWidth = 160, int imageHeight = 120)
        {
            lastLowerBound = double.NaN;
            lastUpperBound = double.NaN;
            lastProjectedUpper = double.NaN;
            lastProjectedMidpointError = double.NaN;
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-D references missing.");
            if (slanted) sceneDefinition.BuildSlanted(0.2f);
            else if (edgeOffset != 0f) sceneDefinition.BuildAtEdge(sceneDefinition.EdgeZMetres + edgeOffset);
            else sceneDefinition.Build();
            Physics.SyncTransforms();
            var depth = new CpuRaycastPerfectDepthGenerator().Generate(sensorCamera, imageWidth, imageHeight, 10f);
            var frame = P0ASensorFrameAdapter.Create(
                0, Time.realtimeSinceStartupAsDouble, sensorCamera, depth);
            var cells = new List<NumericsVector3>[Strips, Bins];
            for (int s = 0; s < Strips; ++s)
                for (int b = 0; b < Bins; ++b)
                    cells[s, b] = new List<NumericsVector3>();

            var groundPoints = new List<NumericsVector3>();
            for (int v = 0; v < imageHeight; ++v)
                for (int u = 0; u < imageWidth; ++u)
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
            double commonLower = double.NegativeInfinity;
            double commonUpper = double.PositiveInfinity;
            double commonProjectedUpper = double.PositiveInfinity;
            int projectedStrips = 0;
            int boundedStrips = 0;
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
                // Refine the coarse 25 mm bin using raw reconstructed points.
                // Only classify samples close to the candidate edge; do not use truth.
                var coarseZ = bestBin * BinWidth;
                double nearMax = double.NegativeInfinity;
                double farMin = double.PositiveInfinity;
                double projectedUpper = double.PositiveInfinity;
                for (int b = Math.Max(0, bestBin - 4); b < Math.Min(Bins, bestBin + 5); ++b)
                    foreach (var p in cells[s, b])
                    {
                        var distance = plane.SignedDistance(p);
                        if (distance > -0.005 && distance < 0.005 && p.Z <= coarseZ + 0.10)
                            nearMax = Math.Max(nearMax, p.Z);
                        else if (distance < -0.010 && p.Z >= coarseZ - 0.10)
                        {
                            farMin = Math.Min(farMin, p.Z);
                            // Ray through a far-floor hit, intersected with reference Y=0.
                            double cameraY = sensorCamera.transform.position.y;
                            double cameraZ = sensorCamera.transform.position.z;
                            double dy = p.Y - cameraY;
                            if (cameraY > 0 && dy < -1e-6 && distance > -0.030)
                            {
                                double intersectionZ = cameraZ + (p.Z - cameraZ) * (-cameraY / dy);
                                if (intersectionZ >= coarseZ - 0.10)
                                    projectedUpper = Math.Min(projectedUpper, intersectionZ);
                            }
                        }
                    }
                var refinedZ = double.IsNegativeInfinity(nearMax) || double.IsPositiveInfinity(farMin)
                    || farMin < nearMax ? coarseZ : (nearMax + farMin) * 0.5;
                if (!double.IsNegativeInfinity(nearMax) && !double.IsPositiveInfinity(farMin) && farMin >= nearMax)
                {
                    commonLower = Math.Max(commonLower, nearMax);
                    commonUpper = Math.Min(commonUpper, farMin);
                    ++boundedStrips;
                    if (!double.IsPositiveInfinity(projectedUpper))
                    {
                        commonProjectedUpper = Math.Min(commonProjectedUpper, projectedUpper);
                        ++projectedStrips;
                    }
                }
                xs.Add(-1.0 + (s + 0.5) * 0.25);
                zs.Add(refinedZ);
            }
            if (xs.Count < 2)
            {
                if (verbose) Debug.Log((slanted ? "[WalkEdgeLight P0-D Slanted]" : edgeOffset != 0f ? "[WalkEdgeLight P0-E Offset]" : "[WalkEdgeLight P0-D]") + " detection=NONE, stripsDetected=" + xs.Count);
                return double.NaN;
            }
            if (!slanted && boundedStrips == xs.Count && commonLower <= commonUpper)
            {
                lastLowerBound = commonLower;
                lastUpperBound = commonUpper;
                if (projectedStrips == xs.Count && commonProjectedUpper >= commonLower)
                    lastProjectedUpper = Math.Min(commonUpper, commonProjectedUpper);
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
            var expectedEdge = truth.EdgeZMetres + edgeOffset;
            if (!double.IsNaN(lastProjectedUpper))
                lastProjectedMidpointError = ((lastLowerBound + lastProjectedUpper) * 0.5 - expectedEdge) * 1000.0;
            if (verbose) Debug.Log((slanted ? "[WalkEdgeLight P0-D Slanted]" : edgeOffset != 0f ? "[WalkEdgeLight P0-E Offset]" : "[WalkEdgeLight P0-D]") + " detection=EDGE" +
                ", stripsDetected=" + xs.Count + "/" + Strips +
                ", slope=" + slope.ToString("F6") +
                ", edgeZAtX0=" + intercept.ToString("F4") + " m" +
                ", edgeTruth=" + expectedEdge.ToString("F4") + " m" +
                ", positionError=" + ((intercept - expectedEdge) * 1000).ToString("F3") + " mm" +
                ", lineRms=" + (rms * 1000).ToString("F3") + " mm" +
                (slanted ? ", slopeTruth=0.200000, slopeError=" + (slope - 0.2).ToString("F6") : ""));
            if (slanted) { sceneDefinition.Build(); Physics.SyncTransforms(); }
            return (intercept - expectedEdge) * 1000.0;
        }
    }
}
