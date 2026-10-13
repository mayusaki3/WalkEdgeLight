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
        private double lastRawLowerBound = double.NaN;
        private int lastValidBeforeDropout;
        private int lastValidAfterDropout;
        private int lastReconstructedPoints;
        private int lastNearBoundaryPoints;
        private int lastFarBoundaryPoints;
        private int lastOutlierCount;
        private int lastOutlierCandidates;
        private readonly double[] lastFarQuantileLowers = new double[4];
        private readonly double[] lastFarQuantileUppers = new double[4];
        private readonly int[] lastFarQuantileValidStrips = new int[4];
        private readonly double[] lastProjectedFarQuantileUppers = new double[4];
        private readonly int[] lastProjectedFarQuantileValidStrips = new int[4];
        private double lastPercentileLowerBound = double.NaN;
        private readonly double[] lastQuantileLowers = new double[4];
        private double lastUpperBound = double.NaN;
        private double lastProjectedUpper = double.NaN;
        private double lastProjectedMidpointError = double.NaN;
        private int lastQuorumValidStrips;
        private double lastQuorumLower = double.NaN;
        private double lastQuorumUpper = double.NaN;
        private double lastQuorumMidpointSpread = double.NaN;
        // Validation-only numerical tolerance; not a depth sensor accuracy claim.
        private const double BoundToleranceMetres = 0.000010;
        private static bool ContainsWithTolerance(double lower, double upper, double value)
        {
            return value >= lower - BoundToleranceMetres &&
                value <= upper + BoundToleranceMetres;
        }

        private static double Quantile(List<double> sortedValues, double fraction)
        {
            if (sortedValues.Count == 0) return double.NaN;
            sortedValues.Sort();
            double position = (sortedValues.Count - 1) * fraction;
            int lo = (int)Math.Floor(position);
            int hi = (int)Math.Ceiling(position);
            return sortedValues[lo] + (sortedValues[hi] - sortedValues[lo]) * (position - lo);
        }

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

        [ContextMenu("Run P0-E Pose")]
        public void RunPose()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-E references missing.");
            float[] heights = { 1.0f, 1.2f, 1.4f };
            float[] pitches = { 35f, 45f, 55f };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            Vector3 originalPosition = sensorCamera.transform.position;
            Quaternion originalRotation = sensorCamera.transform.rotation;
            try
            {
                foreach (float height in heights)
                    foreach (float pitch in pitches)
                    {
                        sensorCamera.transform.position = new Vector3(0f, height, 0f);
                        sensorCamera.transform.rotation = Quaternion.Euler(pitch, 0f, 0f);
                        int detected = 0, bounded = 0, covered = 0;
                        double sumWidth = 0, sumAbs = 0, maxAbs = 0;
                        foreach (float offset in offsets)
                        {
                            double oldError = RunInternal(false, offset, false, 320, 240);
                            double truthZ = sceneDefinition.EdgeZMetres + offset;
                            bool hasBounds = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                            bool contains = hasBounds && lastLowerBound <= truthZ && truthZ <= lastProjectedUpper;
                            if (!double.IsNaN(oldError)) ++detected;
                            if (hasBounds)
                            {
                                ++bounded;
                                sumWidth += (lastProjectedUpper - lastLowerBound) * 1000.0;
                                double absError = Math.Abs(lastProjectedMidpointError);
                                sumAbs += absError;
                                maxAbs = Math.Max(maxAbs, absError);
                            }
                            if (contains) ++covered;
                            Debug.Log("[WalkEdgeLight P0-E Pose] height=" + height.ToString("F2") +
                                " m, pitch=" + pitch.ToString("F0") + " deg, edgeTruth=" + truthZ.ToString("F4") +
                                " m, detected=" + !double.IsNaN(oldError) +
                                ", bounded=" + hasBounds + ", containsTruth=" + contains +
                                ", width=" + (hasBounds ? ((lastProjectedUpper - lastLowerBound) * 1000).ToString("F3") : "N/A") +
                                " mm, midpointError=" + (hasBounds ? lastProjectedMidpointError.ToString("F3") : "N/A") + " mm");
                        }
                        Debug.Log("[WalkEdgeLight P0-E Pose] summary height=" + height.ToString("F2") +
                            " m, pitch=" + pitch.ToString("F0") + " deg, size=320x240" +
                            ", detected=" + detected + "/" + offsets.Length +
                            ", bounded=" + bounded + "/" + offsets.Length +
                            ", coverage=" + covered + "/" + offsets.Length +
                            ", meanWidth=" + (bounded > 0 ? (sumWidth / bounded).ToString("F3") : "N/A") +
                            " mm, midpointMAE=" + (bounded > 0 ? (sumAbs / bounded).ToString("F3") : "N/A") +
                            " mm, maxAbsError=" + (bounded > 0 ? maxAbs.ToString("F3") : "N/A") + " mm");
                    }
            }
            finally
            {
                sensorCamera.transform.position = originalPosition;
                sensorCamera.transform.rotation = originalRotation;
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        // P0-F-1: fixed-seed synthetic Z-depth noise; independent of ground truth.
        private static GeneratedDepth AddGaussianDepthNoise(GeneratedDepth source, double sigmaMetres, int seed)
        {
            var values = (float[])source.DepthMetres.Clone();
            var valid = (bool[])source.Valid.Clone();
            var random = new System.Random(seed);
            for (int i = 0; i < values.Length; ++i)
            {
                if (!valid[i]) continue;
                // Box-Muller with a deterministic per-run random sequence.
                double u1 = Math.Max(random.NextDouble(), 1e-12);
                double u2 = random.NextDouble();
                double standardNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                    Math.Cos(2.0 * Math.PI * u2);
                double noisy = values[i] + sigmaMetres * standardNormal;
                if (noisy <= 0 || double.IsNaN(noisy) || double.IsInfinity(noisy))
                {
                    valid[i] = false;
                    values[i] = 0f;
                }
                else values[i] = (float)noisy;
            }
            return new GeneratedDepth(source.Width, source.Height, values, valid);
        }

        // P0-F-6: deterministic missing depth pixels, applied before reconstruction.
        private static GeneratedDepth DropDepthPixels(GeneratedDepth source, double dropoutFraction, int seed)
        {
            var values = (float[])source.DepthMetres.Clone();
            var valid = (bool[])source.Valid.Clone();
            var random = new System.Random(seed);
            for (int i = 0; i < valid.Length; ++i)
            {
                if (valid[i] && random.NextDouble() < dropoutFraction)
                {
                    valid[i] = false;
                    values[i] = 0f;
                }
            }
            return new GeneratedDepth(source.Width, source.Height, values, valid);
        }

        // P0-F-7: replace a fraction of valid Z-depth values with large signed errors.
        // Fixed-seed corruption is independent of the ground-truth edge position.
        private static GeneratedDepth InjectDepthOutliers(GeneratedDepth source,
            double fraction, double magnitudeMetres, int seed, out int candidates, out int changed, int placementMode = 0)
        {
            var values = (float[])source.DepthMetres.Clone();
            var valid = (bool[])source.Valid.Clone();
            var random = new System.Random(seed);
            candidates = 0;
            changed = 0;
            for (int i = 0; i < valid.Length; ++i)
            {
                if (!valid[i]) continue;
                ++candidates;
                if (random.NextDouble() >= fraction) continue;
                double sign = random.NextDouble() < 0.5 ? -1.0 : 1.0;
                double modified = values[i] + sign * magnitudeMetres;
                if (modified <= 0) continue;
                values[i] = (float)modified;
                ++changed;
            }
            return new GeneratedDepth(source.Width, source.Height, values, valid);
        }

        // P0-F-13: contiguous rectangular depth errors, independent of edge truth.
        private static GeneratedDepth InjectClusteredDepthOutliers(GeneratedDepth source,
            int patchWidth, int patchHeight, int patchCount, double magnitudeMetres,
            int seed, out int candidates, out int changed)
        {
            var values = (float[])source.DepthMetres.Clone();
            var valid = (bool[])source.Valid.Clone();
            var touched = new bool[valid.Length];
            var random = new System.Random(seed);
            candidates = 0;
            changed = 0;
            for (int i = 0; i < valid.Length; ++i)
                if (valid[i]) ++candidates;
            for (int patch = 0; patch < patchCount; ++patch)
            {
                int left = random.Next(0, source.Width - patchWidth + 1);
                int top = random.Next(0, source.Height - patchHeight + 1);
                // Modes: 1=left region, 2=center region, 3=right region.
                // Keep placement independent of the ground-truth edge location.
                if (placementMode > 0)
                {
                    int regionWidth = Math.Max(patchWidth, source.Width / 3);
                    int regionLeft = placementMode == 1 ? 0 :
                        placementMode == 2 ? (source.Width - regionWidth) / 2 :
                        source.Width - regionWidth;
                    left = regionLeft + random.Next(0, regionWidth - patchWidth + 1);
                }
                double sign = random.NextDouble() < 0.5 ? -1.0 : 1.0;
                for (int v = top; v < top + patchHeight; ++v)
                    for (int u = left; u < left + patchWidth; ++u)
                    {
                        int i = v * source.Width + u;
                        if (!valid[i] || touched[i]) continue;
                        double modified = values[i] + sign * magnitudeMetres;
                        if (modified <= 0) continue;
                        values[i] = (float)modified;
                        touched[i] = true;
                        ++changed;
                    }
            }
            return new GeneratedDepth(source.Width, source.Height, values, valid);
        }

        // P0-F-16: concentrated spatial errors and strip disagreement, diagnostic only.
        [ContextMenu("Run P0-F Concentrated Cluster Diagnostic")]
        public void RunConcentratedClusterDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            int[] modes = { 1, 2, 3 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (int mode in modes)
                {
                    int detected = 0, bounded = 0, covered = 0;
                    int spreadCount = 0, insufficient = 0;
                    double spreadSum = 0, spreadMax = 0, widthSum = 0, errorSum = 0, errorMax = 0;
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double result = RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                0, 20261009, 0.05, false,
                                8, 8, 12, seeds[si] + k, true, mode);
                            if (!double.IsNaN(result)) ++detected;
                            if (lastQuorumValidStrips < 5) ++insufficient;
                            if (!double.IsNaN(lastQuorumMidpointSpread))
                            {
                                ++spreadCount;
                                spreadSum += lastQuorumMidpointSpread * 1000.0;
                                spreadMax = Math.Max(spreadMax, lastQuorumMidpointSpread * 1000.0);
                            }
                            if (lastQuorumValidStrips < 5 ||
                                double.IsNaN(lastQuorumLower) ||
                                double.IsNaN(lastQuorumUpper) ||
                                lastQuorumUpper < lastQuorumLower) continue;
                            ++bounded;
                            double truth = sceneDefinition.EdgeZMetres + offsets[k];
                            if (ContainsWithTolerance(lastQuorumLower, lastQuorumUpper, truth))
                                ++covered;
                            widthSum += (lastQuorumUpper - lastQuorumLower) * 1000.0;
                            double e = Math.Abs((lastQuorumUpper + lastQuorumLower) * 0.5 - truth) * 1000.0;
                            errorSum += e;
                            errorMax = Math.Max(errorMax, e);
                        }
                    Debug.Log("[WalkEdgeLight P0-F Concentrated] mode=" + mode +
                        ", detected=" + detected + "/30, bounded=" + bounded +
                        "/30, coverage=" + covered + "/30, validBelow5=" + insufficient +
                        "/30, spreadSamples=" + spreadCount +
                        ", meanStripMidpointSpread=" + (spreadCount == 0 ? "N/A" :
                            (spreadSum / spreadCount).ToString("F3")) +
                        " mm, maxStripMidpointSpread=" + spreadMax.ToString("F3") +
                        " mm, meanWidth=" + (bounded == 0 ? "N/A" :
                            (widthSum / bounded).ToString("F3")) +
                        " mm, midpointMAE=" + (bounded == 0 ? "N/A" :
                            (errorSum / bounded).ToString("F3")) +
                        " mm, midpointMaxAbsError=" + errorMax.ToString("F3") + " mm");
                }
            }
            finally { sceneDefinition.Build(); Physics.SyncTransforms(); }
        }

        [ContextMenu("Run P0-F Clustered Quorum Diagnostic")]
        public void RunClusteredQuorumDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            int[] sizes = { 2, 4, 8 };
            int[] counts = { 1, 4, 12 };
            int[] thresholds = { 8, 7, 6, 5 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                for (int variant = 0; variant < sizes.Length; ++variant)
                {
                    int[] detected = new int[4], bounded = new int[4], covered = new int[4];
                    double[] widths = new double[4], errors = new double[4], maxima = new double[4];
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double result = RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                0, 20261009, 0.05, false,
                                sizes[variant], sizes[variant], counts[variant],
                                seeds[si] + k, true);
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            for (int t = 0; t < thresholds.Length; ++t)
                            {
                                if (!double.IsNaN(result)) ++detected[t];
                                if (lastQuorumValidStrips < thresholds[t] ||
                                    double.IsNaN(lastQuorumLower) ||
                                    double.IsNaN(lastQuorumUpper) ||
                                    lastQuorumUpper < lastQuorumLower) continue;
                                ++bounded[t];
                                if (ContainsWithTolerance(lastQuorumLower, lastQuorumUpper, truthZ))
                                    ++covered[t];
                                widths[t] += (lastQuorumUpper - lastQuorumLower) * 1000.0;
                                double e = Math.Abs((lastQuorumLower + lastQuorumUpper) * 0.5 - truthZ) * 1000.0;
                                errors[t] += e;
                                maxima[t] = Math.Max(maxima[t], e);
                            }
                        }
                    for (int t = 0; t < thresholds.Length; ++t)
                        Debug.Log("[WalkEdgeLight P0-F Quorum] patch=" + sizes[variant] +
                            "x" + sizes[variant] + ", patches=" + counts[variant] +
                            ", required=" + thresholds[t] + "/8, detected=" + detected[t] +
                            "/30, bounded=" + bounded[t] + "/30, coverage=" + covered[t] +
                            "/30, meanWidth=" + (bounded[t] == 0 ? "N/A" :
                                (widths[t] / bounded[t]).ToString("F3")) +
                            " mm, midpointMAE=" + (bounded[t] == 0 ? "N/A" :
                                (errors[t] / bounded[t]).ToString("F3")) +
                            " mm, midpointMaxAbsError=" + (bounded[t] == 0 ? "N/A" :
                                maxima[t].ToString("F3")) + " mm");
                }
            }
            finally { sceneDefinition.Build(); Physics.SyncTransforms(); }
        }

        [ContextMenu("Run P0-F Clustered Failure Diagnostic")]
        public void RunClusteredFailureDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            int[] sizes = { 2, 4, 8 };
            int[] counts = { 1, 4, 12 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                for (int variant = 0; variant < sizes.Length; ++variant)
                {
                    int failures = 0;
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                0, 20261009, 0.05, false,
                                sizes[variant], sizes[variant], counts[variant],
                                seeds[si] + k);
                            if (!double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) &&
                                lastProjectedUpper >= lastLowerBound) continue;
                            ++failures;
                            Debug.Log("[WalkEdgeLight P0-F ClusterDiag] FAIL patch=" +
                                sizes[variant] + "x" + sizes[variant] +
                                ", patches=" + counts[variant] +
                                ", seed=" + (seeds[si] + k) +
                                ", offset=" + offsets[k].ToString("F3") +
                                ", truthZ=" + (sceneDefinition.EdgeZMetres + offsets[k]).ToString("F9") +
                                ", changed=" + lastOutlierCount);
                            RunInternal(false, offsets[k], false, 320, 240,
                                true, 0, 20261009, 0, 20261009,
                                0, 20261009, 0.05, false,
                                sizes[variant], sizes[variant], counts[variant],
                                seeds[si] + k);
                        }
                    Debug.Log("[WalkEdgeLight P0-F ClusterDiag] summary patch=" +
                        sizes[variant] + "x" + sizes[variant] +
                        ", failures=" + failures + "/30");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Clustered Outliers")]
        public void RunClusteredOutliers()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            int[] patchSizes = { 2, 4, 8 };
            int[] patchCounts = { 1, 4, 12 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                for (int variant = 0; variant < patchSizes.Length; ++variant)
                {
                    int detected = 0, bounded = 0, covered = 0;
                    long candidates = 0, changed = 0;
                    double sumWidth = 0, sumError = 0, maxError = 0;
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double error = RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                0, 20261009, 0.05, false,
                                patchSizes[variant], patchSizes[variant],
                                patchCounts[variant], seeds[si] + k);
                            candidates += lastOutlierCandidates;
                            changed += lastOutlierCount;
                            if (!double.IsNaN(error)) ++detected;
                            if (double.IsNaN(lastLowerBound) ||
                                double.IsNaN(lastProjectedUpper) ||
                                lastProjectedUpper < lastLowerBound) continue;
                            ++bounded;
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            if (ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ))
                                ++covered;
                            double absError = Math.Abs(lastProjectedMidpointError);
                            sumWidth += (lastProjectedUpper - lastLowerBound) * 1000;
                            sumError += absError;
                            maxError = Math.Max(maxError, absError);
                        }
                    Debug.Log("[WalkEdgeLight P0-F Clustered] patch=" +
                        patchSizes[variant] + "x" + patchSizes[variant] +
                        ", patches=" + patchCounts[variant] +
                        ", candidates=" + candidates + ", changed=" + changed +
                        ", actualRate=" + (candidates == 0 ? "N/A" :
                            (100.0 * changed / candidates).ToString("F3")) +
                        "%, detected=" + detected + "/30, bounded=" + bounded +
                        "/30, coverage=" + covered + "/30, meanWidth=" +
                        (bounded == 0 ? "N/A" : (sumWidth / bounded).ToString("F3")) +
                        " mm, midpointMAE=" +
                        (bounded == 0 ? "N/A" : (sumError / bounded).ToString("F3")) +
                        " mm, midpointMaxAbsError=" +
                        (bounded == 0 ? "N/A" : maxError.ToString("F3")) + " mm");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        // Diagnostic only: compare raw far-side minimum against low quantiles.
        // Does not alter production boundary selection.
        [ContextMenu("Run P0-F Projected Far Quantile")]
        public void RunProjectedFarQuantile()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] rates = { 0.0, 0.001, 0.005, 0.01 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (double rate in rates)
                {
                    int[] bounded = new int[2];
                    int[] coverage = new int[2];
                    double[] widthSum = new double[2];
                    double[] errorSum = new double[2];
                    double[] errorMax = new double[2];
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                rate, seeds[si] + k, 0.05, true);
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            for (int q = 0; q < 2; ++q)
                            {
                                int index = q + 1; // 2% and 5%
                                double lower = lastFarQuantileLowers[index];
                                double upper = lastProjectedFarQuantileUppers[index];
                                if (lastProjectedFarQuantileValidStrips[index] != Strips ||
                                    double.IsNaN(lower) || double.IsNaN(upper) ||
                                    upper < lower)
                                    continue;
                                ++bounded[q];
                                if (ContainsWithTolerance(lower, upper, truthZ))
                                    ++coverage[q];
                                double absErrorMm = Math.Abs((0.5 * (lower + upper) - truthZ) * 1000);
                                widthSum[q] += (upper - lower) * 1000;
                                errorSum[q] += absErrorMm;
                                errorMax[q] = Math.Max(errorMax[q], absErrorMm);
                            }
                        }
                    for (int q = 0; q < 2; ++q)
                        Debug.Log("[WalkEdgeLight P0-F ProjectedFar] rate=" +
                            (rate * 100).ToString("F1") + "%, farQuantile=" +
                            (q == 0 ? "2" : "5") +
                            "%, bounded=" + bounded[q] + "/30, coverage=" +
                            coverage[q] + "/30, meanWidth=" +
                            (bounded[q] == 0 ? "N/A" : (widthSum[q] / bounded[q]).ToString("F3")) +
                            " mm, midpointMAE=" +
                            (bounded[q] == 0 ? "N/A" : (errorSum[q] / bounded[q]).ToString("F3")) +
                            " mm, midpointMaxAbsError=" +
                            (bounded[q] == 0 ? "N/A" : errorMax[q].ToString("F3")) + " mm");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Far Quantile Accuracy")]
        public void RunFarQuantileAccuracy()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] rates = { 0.0, 0.001, 0.005, 0.01 };
            double[] fractions = { 0.02, 0.05, 0.10 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (double rate in rates)
                {
                    int[] bounded = new int[3];
                    int[] coverage = new int[3];
                    double[] sumWidth = new double[3];
                    double[] sumAbsError = new double[3];
                    double[] maxAbsError = new double[3];
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                rate, seeds[si] + k, 0.05, true);
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            for (int q = 0; q < fractions.Length; ++q)
                            {
                                int index = q + 1;
                                double lower = lastFarQuantileLowers[index];
                                double upper = lastFarQuantileUppers[index];
                                if (lastFarQuantileValidStrips[index] != Strips ||
                                    double.IsNaN(lower) || double.IsNaN(upper) ||
                                    upper < lower)
                                    continue;
                                ++bounded[q];
                                if (ContainsWithTolerance(lower, upper, truthZ))
                                    ++coverage[q];
                                double errorMm = Math.Abs((0.5 * (lower + upper) - truthZ) * 1000);
                                sumWidth[q] += (upper - lower) * 1000;
                                sumAbsError[q] += errorMm;
                                maxAbsError[q] = Math.Max(maxAbsError[q], errorMm);
                            }
                        }
                    for (int q = 0; q < fractions.Length; ++q)
                        Debug.Log("[WalkEdgeLight P0-F FarAccuracy] rate=" +
                            (rate * 100).ToString("F1") + "%, farQuantile=" +
                            (fractions[q] * 100).ToString("F0") +
                            "%, bounded=" + bounded[q] + "/30, coverage=" +
                            coverage[q] + "/30, meanWidth=" +
                            (bounded[q] == 0 ? "N/A" : (sumWidth[q] / bounded[q]).ToString("F3")) +
                            " mm, midpointMAE=" +
                            (bounded[q] == 0 ? "N/A" : (sumAbsError[q] / bounded[q]).ToString("F3")) +
                            " mm, midpointMaxAbsError=" +
                            (bounded[q] == 0 ? "N/A" : maxAbsError[q].ToString("F3")) + " mm");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Far Quantile Diagnostic")]
        public void RunFarQuantileDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] rates = { 0.0, 0.001, 0.005, 0.01 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (double rate in rates)
                {
                    int[] validStrips = new int[4];
                    int[] coverage = new int[4];
                    double[] fractions = { 0.00, 0.02, 0.05, 0.10 };
                    for (int si = 0; si < seeds.Length; ++si)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                rate, seeds[si] + k, 0.05, true);
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            for (int q = 0; q < fractions.Length; ++q)
                            {
                                if (lastFarQuantileValidStrips[q] == Strips)
                                    ++validStrips[q];
                                if (!double.IsNaN(lastFarQuantileLowers[q]) &&
                                    !double.IsNaN(lastFarQuantileUppers[q]) &&
                                    lastFarQuantileValidStrips[q] == Strips &&
                                    lastFarQuantileUppers[q] >= lastFarQuantileLowers[q] &&
                                    ContainsWithTolerance(lastFarQuantileLowers[q],
                                        lastFarQuantileUppers[q], truthZ))
                                    ++coverage[q];
                            }
                        }
                    for (int q = 0; q < fractions.Length; ++q)
                        Debug.Log("[WalkEdgeLight P0-F FarQuantile] rate=" +
                            (rate * 100).ToString("F1") + "%, farQuantile=" +
                            (fractions[q] * 100).ToString("F0") +
                            "%, allStripsValid=" + validStrips[q] +
                            "/30, coverage=" + coverage[q] +
                            "/30, cases=30");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Outlier Diagnostic")]
        public void RunOutlierDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] rates = { 0.001, 0.005, 0.01 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (double rate in rates)
                {
                    bool found = false;
                    foreach (int seed in seeds)
                    {
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            RunInternal(false, offsets[k], false, 320, 240, false,
                                0, 20261009, 0, 20261009, rate, seed + k, 0.05);
                            if (!double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) &&
                                lastProjectedUpper >= lastLowerBound)
                                continue;
                            Debug.Log("[WalkEdgeLight P0-F OutlierDiag] failingCase rate=" +
                                (rate * 100).ToString("F1") + "%, offsetMm=" +
                                (offsets[k] * 1000).ToString("F1") + ", seed=" + (seed + k));
                            RunInternal(false, offsets[k], false, 320, 240, true,
                                0, 20261009, 0, 20261009, rate, seed + k, 0.05);
                            found = true;
                            break;
                        }
                        if (found) break;
                    }
                    if (!found)
                        Debug.Log("[WalkEdgeLight P0-F OutlierDiag] noFailure rate=" +
                            (rate * 100).ToString("F1") + "%");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Outliers")]
        public void RunOutliers()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] rates = { 0.0, 0.001, 0.005, 0.01 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            const double magnitudeMetres = 0.05;
            try
            {
                foreach (double rate in rates)
                {
                    int detected = 0, bounded = 0, covered = 0;
                    long candidates = 0, changed = 0;
                    double sumAbs = 0, maxAbs = 0, sumWidth = 0;
                    foreach (int seed in seeds)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double error = RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, 0, 20261009,
                                rate, seed + k, magnitudeMetres);
                            candidates += lastOutlierCandidates;
                            changed += lastOutlierCount;
                            if (!double.IsNaN(error)) ++detected;
                            bool valid = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) &&
                                lastProjectedUpper >= lastLowerBound;
                            if (!valid) continue;
                            ++bounded;
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            if (ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ)) ++covered;
                            double absError = Math.Abs(lastProjectedMidpointError);
                            sumAbs += absError;
                            maxAbs = Math.Max(maxAbs, absError);
                            sumWidth += (lastProjectedUpper - lastLowerBound) * 1000.0;
                        }
                    Debug.Log("[WalkEdgeLight P0-F OutlierAudit] requestedRate=" +
                        (rate * 100).ToString("F1") + "%, candidatePixels=" + candidates +
                        ", changedPixels=" + changed + ", actualRate=" +
                        (candidates > 0 ? (changed * 100.0 / candidates).ToString("F3") : "N/A") +
                        "%, signedDepthErrorMm=+/-50, cases=30");
                    Debug.Log("[WalkEdgeLight P0-F Outliers] rate=" +
                        (rate * 100).ToString("F1") + "%, detected=" + detected +
                        "/30, bounded=" + bounded + "/30, coverage=" + covered +
                        "/30, meanWidth=" +
                        (bounded > 0 ? (sumWidth / bounded).ToString("F3") : "N/A") +
                        " mm, midpointMAE=" +
                        (bounded > 0 ? (sumAbs / bounded).ToString("F3") : "N/A") +
                        " mm, midpointMaxAbsError=" +
                        (bounded > 0 ? maxAbs.ToString("F3") : "N/A") +
                        " mm, seedCount=5, percentile=95");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Dropout")]
        public void RunDropout()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] dropoutRates = { 0.0, 0.05, 0.10, 0.20 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            try
            {
                foreach (double rate in dropoutRates)
                {
                    int detected = 0, bounded = 0, covered = 0;
                    double sumAbs = 0, maxAbs = 0, sumWidth = 0;
                    long beforePixels = 0, afterPixels = 0, reconstructed = 0;
                    long nearBoundary = 0, farBoundary = 0;
                    foreach (int seed in seeds)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double error = RunInternal(false, offsets[k], false, 320, 240,
                                false, 0, 20261009, rate, seed + k);
                            beforePixels += lastValidBeforeDropout;
                            afterPixels += lastValidAfterDropout;
                            reconstructed += lastReconstructedPoints;
                            nearBoundary += lastNearBoundaryPoints;
                            farBoundary += lastFarBoundaryPoints;
                            if (!double.IsNaN(error)) ++detected;
                            bool valid = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                            if (!valid) continue;
                            ++bounded;
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            if (ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ)) ++covered;
                            double absError = Math.Abs(lastProjectedMidpointError);
                            sumAbs += absError;
                            maxAbs = Math.Max(maxAbs, absError);
                            sumWidth += (lastProjectedUpper - lastLowerBound) * 1000.0;
                        }
                    Debug.Log("[WalkEdgeLight P0-F DropoutAudit] requestedRate=" +
                        (rate * 100).ToString("F0") + "%, validBefore=" + beforePixels +
                        ", validAfter=" + afterPixels + ", dropped=" + (beforePixels - afterPixels) +
                        ", actualDropRate=" +
                        (beforePixels > 0 ? ((beforePixels - afterPixels) * 100.0 / beforePixels).ToString("F3") : "N/A") +
                        "%, reconstructedPoints=" + reconstructed +
                        ", nearBoundaryPoints=" + nearBoundary +
                        ", farBoundaryPoints=" + farBoundary + ", cases=30");
                    Debug.Log("[WalkEdgeLight P0-F Dropout] rate=" + (rate * 100).ToString("F0") +
                        "%, detected=" + detected + "/30, bounded=" + bounded +
                        "/30, coverage=" + covered + "/30, meanWidth=" +
                        (bounded > 0 ? (sumWidth / bounded).ToString("F3") : "N/A") +
                        " mm, midpointMAE=" +
                        (bounded > 0 ? (sumAbs / bounded).ToString("F3") : "N/A") +
                        " mm, midpointMaxAbsError=" +
                        (bounded > 0 ? maxAbs.ToString("F3") : "N/A") +
                        " mm, seedCount=5, percentile=95");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Gaussian")]
        public void RunGaussianNoise()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] sigmaMm = { 0.0, 0.5, 1.0, 2.0 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            for (int level = 0; level < sigmaMm.Length; ++level)
            {
                int detected = 0, bounded = 0, covered = 0;
                double sumAbsError = 0, maxAbsError = 0;
                for (int k = 0; k < offsets.Length; ++k)
                {
                    double error = RunInternal(false, offsets[k], false, 320, 240,
                        false, sigmaMm[level] * 0.001, 20261009 + k);
                    bool hasBounds = !double.IsNaN(lastLowerBound) &&
                        !double.IsNaN(lastProjectedUpper) &&
                        lastProjectedUpper >= lastLowerBound;
                    double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                    if (!double.IsNaN(error)) ++detected;
                    if (hasBounds)
                    {
                        ++bounded;
                        if (ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ)) ++covered;
                        double absError = Math.Abs(lastProjectedMidpointError);
                        sumAbsError += absError;
                        maxAbsError = Math.Max(maxAbsError, absError);
                    }
                    Debug.Log("[WalkEdgeLight P0-F Gaussian] sigma=" + sigmaMm[level].ToString("F1") +
                        " mm, edgeTruth=" + truthZ.ToString("F4") +
                        " m, detected=" + !double.IsNaN(error) +
                        ", bounded=" + hasBounds +
                        ", midpointError=" + (hasBounds ? lastProjectedMidpointError.ToString("F3") : "N/A") + " mm");
                }
                Debug.Log("[WalkEdgeLight P0-F Gaussian] summary sigma=" +
                    sigmaMm[level].ToString("F1") + " mm, detected=" + detected +
                    "/6, bounded=" + bounded + "/6, toleranceCoverage=" + covered +
                    "/6, midpointMAE=" + (bounded > 0 ? (sumAbsError / bounded).ToString("F3") : "N/A") +
                    " mm, midpointMaxAbsError=" + (bounded > 0 ? maxAbsError.ToString("F3") : "N/A") +
                    " mm, seedBase=20261009");
            }
            sceneDefinition.Build();
            Physics.SyncTransforms();
        }

        [ContextMenu("Run P0-F Default Regression")]
        public void RunDefaultRegression()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            float[] heights = { 1.0f, 1.2f, 1.4f };
            float[] pitches = { 35f, 45f, 55f };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            double[] sigmas = { 0.0, 1.0, 2.0 };
            int[] seeds = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            Vector3 oldPosition = sensorCamera.transform.position;
            Quaternion oldRotation = sensorCamera.transform.rotation;
            try
            {
                int detected = 0, bounded = 0, covered = 0;
                for (int h = 0; h < heights.Length; ++h)
                    for (int p = 0; p < pitches.Length; ++p)
                    {
                        sensorCamera.transform.position = new Vector3(0f, heights[h], 0f);
                        sensorCamera.transform.rotation = Quaternion.Euler(pitches[p], 0f, 0f);
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double result = RunInternal(false, offsets[k], false, 320, 240);
                            if (!double.IsNaN(result)) ++detected;
                            bool valid = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                            if (valid) ++bounded;
                            if (valid && ContainsWithTolerance(lastLowerBound, lastProjectedUpper,
                                sceneDefinition.EdgeZMetres + offsets[k])) ++covered;
                        }
                    }
                Debug.Log("[WalkEdgeLight P0-F DefaultRegression] poseSummary detected=" +
                    detected + "/54, bounded=" + bounded + "/54, coverage=" + covered +
                    "/54, percentile=95, toleranceUm=10");
                sensorCamera.transform.position = new Vector3(0f, 1.2f, 0f);
                sensorCamera.transform.rotation = Quaternion.Euler(45f, 0f, 0f);
                foreach (double sigma in sigmas)
                {
                    detected = 0; bounded = 0; covered = 0;
                    foreach (int seed in seeds)
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double result = RunInternal(false, offsets[k], false, 320, 240,
                                false, sigma * 0.001, seed + k);
                            if (!double.IsNaN(result)) ++detected;
                            bool valid = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                            if (valid) ++bounded;
                            if (valid && ContainsWithTolerance(lastLowerBound, lastProjectedUpper,
                                sceneDefinition.EdgeZMetres + offsets[k])) ++covered;
                        }
                    Debug.Log("[WalkEdgeLight P0-F DefaultRegression] noiseSummary sigma=" +
                        sigma.ToString("F1") + " mm, detected=" + detected +
                        "/30, bounded=" + bounded + "/30, coverage=" + covered +
                        "/30, percentile=95");
                }
            }
            finally
            {
                sensorCamera.transform.position = oldPosition;
                sensorCamera.transform.rotation = oldRotation;
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Quantile Seed Sweep")]
        public void RunQuantileSeedSweep()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] sigmaMm = { 0.0, 1.0, 2.0 };
            double[] quantiles = { 90.0, 95.0, 98.0, 100.0 };
            int[] seedBases = { 20261009, 20261019, 20261029, 20261039, 20261049 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            try
            {
                for (int s = 0; s < sigmaMm.Length; ++s)
                {
                    int[] bounded = new int[quantiles.Length];
                    int[] covered = new int[quantiles.Length];
                    double[] widthSum = new double[quantiles.Length];
                    double[] absErrorSum = new double[quantiles.Length];
                    double[] maxAbsError = new double[quantiles.Length];
                    int detected = 0;
                    for (int seedIndex = 0; seedIndex < seedBases.Length; ++seedIndex)
                    {
                        for (int k = 0; k < offsets.Length; ++k)
                        {
                            double error = RunInternal(false, offsets[k], false, 320, 240,
                                false, sigmaMm[s] * 0.001, seedBases[seedIndex] + k);
                            if (!double.IsNaN(error)) ++detected;
                            double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                            for (int q = 0; q < quantiles.Length; ++q)
                            {
                                double lower = lastQuantileLowers[q];
                                double upper = lastProjectedUpper;
                                if (double.IsNaN(lower) || double.IsNaN(upper) || upper < lower)
                                    continue;
                                ++bounded[q];
                                if (ContainsWithTolerance(lower, upper, truthZ)) ++covered[q];
                                widthSum[q] += (upper - lower) * 1000.0;
                                double absError = Math.Abs(((lower + upper) * 0.5 - truthZ) * 1000.0);
                                absErrorSum[q] += absError;
                                maxAbsError[q] = Math.Max(maxAbsError[q], absError);
                            }
                        }
                    }
                    int total = seedBases.Length * offsets.Length;
                    for (int q = 0; q < quantiles.Length; ++q)
                    {
                        Debug.Log("[WalkEdgeLight P0-F QuantileSweep] sigma=" + sigmaMm[s].ToString("F1") +
                            " mm, percentile=" + quantiles[q].ToString("F0") +
                            ", detected=" + detected + "/" + total +
                            ", bounded=" + bounded[q] + "/" + total +
                            ", coverage=" + covered[q] + "/" + total +
                            ", meanWidth=" + (bounded[q] > 0 ? (widthSum[q] / bounded[q]).ToString("F3") : "N/A") +
                            " mm, midpointMAE=" + (bounded[q] > 0 ? (absErrorSum[q] / bounded[q]).ToString("F3") : "N/A") +
                            " mm, midpointMaxAbsError=" + (bounded[q] > 0 ? maxAbsError[q].ToString("F3") : "N/A") +
                            " mm, seedCount=" + seedBases.Length +
                            ", seedBases=20261009/20261019/20261029/20261039/20261049");
                    }
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Percentile Comparison")]
        public void RunPercentileComparison()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] sigmaMm = { 0.0, 1.0, 2.0 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            try
            {
                foreach (double sigma in sigmaMm)
                {
                    int detected = 0, rawBounded = 0, percentileBounded = 0;
                    int rawCovered = 0, percentileCovered = 0;
                    double rawWidthSum = 0, percentileWidthSum = 0;
                    double rawErrorSum = 0, percentileErrorSum = 0;
                    foreach (float offset in offsets)
                    {
                        double result = RunInternal(false, offset, false, 320, 240,
                            false, sigma * 0.001, 20261009 + Array.IndexOf(offsets, offset));
                        if (!double.IsNaN(result)) ++detected;
                        double truthZ = sceneDefinition.EdgeZMetres + offset;
                        bool rawValid = !double.IsNaN(lastRawLowerBound) &&
                            !double.IsNaN(lastProjectedUpper) &&
                            lastProjectedUpper >= lastRawLowerBound;
                        bool percentileValid = !double.IsNaN(lastPercentileLowerBound) &&
                            !double.IsNaN(lastProjectedUpper) &&
                            lastProjectedUpper >= lastPercentileLowerBound;
                        if (rawValid)
                        {
                            ++rawBounded;
                            rawWidthSum += (lastProjectedUpper - lastRawLowerBound) * 1000;
                            rawErrorSum += Math.Abs(((lastRawLowerBound + lastProjectedUpper) * 0.5 - truthZ) * 1000);
                            if (ContainsWithTolerance(lastRawLowerBound, lastProjectedUpper, truthZ)) ++rawCovered;
                        }
                        if (percentileValid)
                        {
                            ++percentileBounded;
                            percentileWidthSum += (lastProjectedUpper - lastPercentileLowerBound) * 1000;
                            percentileErrorSum += Math.Abs(((lastPercentileLowerBound + lastProjectedUpper) * 0.5 - truthZ) * 1000);
                            if (ContainsWithTolerance(lastPercentileLowerBound, lastProjectedUpper, truthZ)) ++percentileCovered;
                        }
                        Debug.Log("[WalkEdgeLight P0-F Percentile] sigma=" + sigma.ToString("F1") +
                            " mm, truth=" + truthZ.ToString("F4") +
                            " m, rawLower=" + lastRawLowerBound.ToString("F9") +
                            ", percentile95Lower=" + lastPercentileLowerBound.ToString("F9") +
                            ", projectedUpper=" + lastProjectedUpper.ToString("F9") +
                            ", rawValid=" + rawValid + ", percentileValid=" + percentileValid);
                    }
                    Debug.Log("[WalkEdgeLight P0-F Percentile] summary sigma=" + sigma.ToString("F1") +
                        " mm, detected=" + detected + "/6, rawBounded=" + rawBounded +
                        "/6, percentileBounded=" + percentileBounded +
                        "/6, rawCoverage=" + rawCovered + "/6, percentileCoverage=" +
                        percentileCovered + "/6, rawMeanWidth=" +
                        (rawBounded > 0 ? (rawWidthSum / rawBounded).ToString("F3") : "N/A") +
                        " mm, percentileMeanWidth=" +
                        (percentileBounded > 0 ? (percentileWidthSum / percentileBounded).ToString("F3") : "N/A") +
                        " mm, rawMAE=" +
                        (rawBounded > 0 ? (rawErrorSum / rawBounded).ToString("F3") : "N/A") +
                        " mm, percentileMAE=" +
                        (percentileBounded > 0 ? (percentileErrorSum / percentileBounded).ToString("F3") : "N/A") + " mm");
                }
            }
            finally
            {
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-F Noise Bounds Diagnostic")]
        public void RunNoiseBoundsDiagnostic()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-F references missing.");
            double[] sigmaMm = { 1.0, 2.0 };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            for (int level = 0; level < sigmaMm.Length; ++level)
            {
                int detected = 0, bounded = 0, covered = 0;
                int lowerMissCount = 0, upperMissCount = 0;
                double maximumLowerMissMm = 0, maximumUpperMissMm = 0;
                for (int k = 0; k < offsets.Length; ++k)
                {
                    double error = RunInternal(false, offsets[k], false, 320, 240,
                        false, sigmaMm[level] * 0.001, 20261009 + k);
                    if (!double.IsNaN(error)) ++detected;
                    bool hasBounds = !double.IsNaN(lastRawLowerBound) &&
                        !double.IsNaN(lastProjectedUpper) &&
                        lastProjectedUpper >= lastRawLowerBound;
                    if (!hasBounds)
                    {
                        Debug.Log("[WalkEdgeLight P0-F NoiseBounds] sigma=" +
                            sigmaMm[level].ToString("F1") + " mm, edgeOffset=" +
                            offsets[k].ToString("F3") + " m, bounded=False");
                        continue;
                    }
                    ++bounded;
                    double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                    double lowerMissMm = Math.Max(0, (lastRawLowerBound - truthZ - BoundToleranceMetres) * 1000);
                    double upperMissMm = Math.Max(0, (truthZ - lastProjectedUpper - BoundToleranceMetres) * 1000);
                    bool contains = lowerMissMm <= 0 && upperMissMm <= 0;
                    if (contains) ++covered;
                    if (lowerMissMm > 0) { ++lowerMissCount; maximumLowerMissMm = Math.Max(maximumLowerMissMm, lowerMissMm); }
                    if (upperMissMm > 0) { ++upperMissCount; maximumUpperMissMm = Math.Max(maximumUpperMissMm, upperMissMm); }
                    Debug.Log("[WalkEdgeLight P0-F NoiseBounds] sigma=" +
                        sigmaMm[level].ToString("F1") + " mm, edgeTruth=" +
                        truthZ.ToString("F9") + " m, rawLower=" +
                        lastRawLowerBound.ToString("F9") + " m, rawUpper=" +
                        lastProjectedUpper.ToString("F9") + " m, lowerMiss=" +
                        lowerMissMm.ToString("F6") + " mm, upperMiss=" +
                        upperMissMm.ToString("F6") + " mm, contains=" + contains);
                }
                Debug.Log("[WalkEdgeLight P0-F NoiseBounds] summary sigma=" +
                    sigmaMm[level].ToString("F1") + " mm, detected=" + detected +
                    "/6, bounded=" + bounded + "/6, coverage=" + covered +
                    "/6, lowerMisses=" + lowerMissCount + ", upperMisses=" + upperMissCount +
                    ", maxLowerMiss=" + maximumLowerMissMm.ToString("F6") +
                    " mm, maxUpperMiss=" + maximumUpperMissMm.ToString("F6") +
                    " mm, toleranceUm=" + (BoundToleranceMetres * 1e6).ToString("F1"));
            }
            sceneDefinition.Build();
            Physics.SyncTransforms();
        }

        [ContextMenu("Run P0-E Bounds Regression")]
        public void RunBoundsRegression()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-E references missing.");
            float[] heights = { 1.0f, 1.2f, 1.4f };
            float[] pitches = { 35f, 45f, 55f };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            int detected = 0, bounded = 0, rawCovered = 0, toleranceCovered = 0;
            int midpointValid = 0;
            Vector3 originalPosition = sensorCamera.transform.position;
            Quaternion originalRotation = sensorCamera.transform.rotation;
            try
            {
                foreach (float height in heights)
                    foreach (float pitch in pitches)
                    {
                        sensorCamera.transform.position = new Vector3(0f, height, 0f);
                        sensorCamera.transform.rotation = Quaternion.Euler(pitch, 0f, 0f);
                        foreach (float offset in offsets)
                        {
                            double error = RunInternal(false, offset, false, 320, 240);
                            if (!double.IsNaN(error)) ++detected;
                            bool valid = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) &&
                                lastProjectedUpper >= lastLowerBound;
                            if (!valid) continue;
                            ++bounded;
                            double truthZ = sceneDefinition.EdgeZMetres + offset;
                            double rawMidpoint = (lastLowerBound + lastProjectedUpper) * 0.5;
                            double tolerantLower = lastLowerBound - BoundToleranceMetres;
                            double tolerantUpper = lastProjectedUpper + BoundToleranceMetres;
                            double tolerantMidpoint = (tolerantLower + tolerantUpper) * 0.5;
                            bool rawContains = lastLowerBound <= truthZ && truthZ <= lastProjectedUpper;
                            bool tolerantContains = ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ);
                            if (rawContains) ++rawCovered;
                            if (tolerantContains) ++toleranceCovered;
                            if (Math.Abs(rawMidpoint - tolerantMidpoint) < 1e-12) ++midpointValid;
                            if (!rawContains || !tolerantContains)
                                Debug.Log("[WalkEdgeLight P0-E BoundsRegression] exception height=" +
                                    height.ToString("F2") + ", pitch=" + pitch.ToString("F0") +
                                    ", truth=" + truthZ.ToString("F9") +
                                    ", rawLower=" + lastLowerBound.ToString("F9") +
                                    ", rawUpper=" + lastProjectedUpper.ToString("F9") +
                                    ", tolerantLower=" + tolerantLower.ToString("F9") +
                                    ", tolerantUpper=" + tolerantUpper.ToString("F9") +
                                    ", rawContains=" + rawContains +
                                    ", tolerantContains=" + tolerantContains);
                        }
                    }
                Debug.Log("[WalkEdgeLight P0-E BoundsRegression] summary detected=" + detected +
                    "/54, bounded=" + bounded + "/54, rawCoverage=" + rawCovered +
                    "/54, toleranceCoverage=" + toleranceCovered +
                    "/54, midpointUnchanged=" + midpointValid +
                    "/54, toleranceUm=" + (BoundToleranceMetres * 1e6).ToString("F1"));
            }
            finally
            {
                sensorCamera.transform.position = originalPosition;
                sensorCamera.transform.rotation = originalRotation;
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-E Tolerance")]
        public void RunTolerance()
        {
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-E references missing.");
            float[] heights = { 1.0f, 1.2f, 1.4f };
            float[] pitches = { 35f, 45f, 55f };
            float[] offsets = { 0f, 0.005f, 0.010f, 0.013f, 0.020f, 0.025f };
            double[] toleranceMicrometres = { 0.0, 1.0, 5.0, 10.0 };
            int[] coverage = new int[toleranceMicrometres.Length];
            int detected = 0, bounded = 0;
            double maximumRequiredMicrometres = 0;
            Vector3 originalPosition = sensorCamera.transform.position;
            Quaternion originalRotation = sensorCamera.transform.rotation;
            try
            {
                foreach (float height in heights)
                    foreach (float pitch in pitches)
                    {
                        sensorCamera.transform.position = new Vector3(0f, height, 0f);
                        sensorCamera.transform.rotation = Quaternion.Euler(pitch, 0f, 0f);
                        foreach (float offset in offsets)
                        {
                            double originalError = RunInternal(false, offset, false, 320, 240);
                            if (!double.IsNaN(originalError)) ++detected;
                            bool hasBounds = !double.IsNaN(lastLowerBound) &&
                                !double.IsNaN(lastProjectedUpper) && lastProjectedUpper >= lastLowerBound;
                            if (!hasBounds) continue;
                            ++bounded;
                            double truthZ = sceneDefinition.EdgeZMetres + offset;
                            double lowerMissMicrometres = Math.Max(0, (lastLowerBound - truthZ) * 1e6);
                            double upperMissMicrometres = Math.Max(0, (truthZ - lastProjectedUpper) * 1e6);
                            double required = Math.Max(lowerMissMicrometres, upperMissMicrometres);
                            maximumRequiredMicrometres = Math.Max(maximumRequiredMicrometres, required);
                            for (int i = 0; i < toleranceMicrometres.Length; ++i)
                                if (required <= toleranceMicrometres[i]) ++coverage[i];
                            if (required > 0)
                                Debug.Log("[WalkEdgeLight P0-E Tolerance] miss height=" + height.ToString("F2") +
                                    " m, pitch=" + pitch.ToString("F0") + " deg, edgeTruth=" + truthZ.ToString("F9") +
                                    " m, lower=" + lastLowerBound.ToString("F9") +
                                    " m, upper=" + lastProjectedUpper.ToString("F9") +
                                    " m, lowerMiss=" + lowerMissMicrometres.ToString("F3") +
                                    " um, upperMiss=" + upperMissMicrometres.ToString("F3") +
                                    " um, required=" + required.ToString("F3") + " um");
                        }
                    }
                for (int i = 0; i < toleranceMicrometres.Length; ++i)
                    Debug.Log("[WalkEdgeLight P0-E Tolerance] summary tolerance=" +
                        toleranceMicrometres[i].ToString("F1") + " um, detected=" + detected +
                        "/54, bounded=" + bounded + "/54, coverage=" + coverage[i] +
                        "/54, maximumRequired=" + maximumRequiredMicrometres.ToString("F3") + " um");
            }
            finally
            {
                sensorCamera.transform.position = originalPosition;
                sensorCamera.transform.rotation = originalRotation;
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        [ContextMenu("Run P0-E Bound Diagnostic")]
        public void RunBoundDiagnostic()
        {
            if (sensorCamera == null || sceneDefinition == null)
                throw new InvalidOperationException("P0-E references missing.");
            Vector3 originalPosition = sensorCamera.transform.position;
            Quaternion originalRotation = sensorCamera.transform.rotation;
            try
            {
                sensorCamera.transform.position = new Vector3(0f, 1.2f, 0f);
                sensorCamera.transform.rotation = Quaternion.Euler(55f, 0f, 0f);
                RunInternal(false, 0.010f, false, 320, 240, true);
            }
            finally
            {
                sensorCamera.transform.position = originalPosition;
                sensorCamera.transform.rotation = originalRotation;
                sceneDefinition.Build();
                Physics.SyncTransforms();
            }
        }

        private double RunInternal(bool slanted, float edgeOffset = 0f, bool verbose = true, int imageWidth = 160, int imageHeight = 120, bool diagnoseBounds = false, double gaussianSigmaMetres = 0, int gaussianSeed = 20261009, double dropoutFraction = 0, int dropoutSeed = 20261009, double outlierFraction = 0, int outlierSeed = 20261009, double outlierMagnitudeMetres = 0.05, bool farQuantileDiagnostic = false, int clusterWidth = 0, int clusterHeight = 0, int clusterCount = 0, int clusterSeed = 20261009, bool quorumDiagnostic = false, int clusterPlacementMode = 0)
        {
            lastLowerBound = double.NaN;
            lastRawLowerBound = double.NaN;
            lastValidBeforeDropout = 0;
            lastValidAfterDropout = 0;
            lastReconstructedPoints = 0;
            lastNearBoundaryPoints = 0;
            lastFarBoundaryPoints = 0;
            lastOutlierCount = 0;
            lastOutlierCandidates = 0;
            for (int q = 0; q < 4; ++q)
            {
                lastFarQuantileLowers[q] = double.NaN;
                lastFarQuantileUppers[q] = double.NaN;
                lastFarQuantileValidStrips[q] = 0;
                lastProjectedFarQuantileUppers[q] = double.NaN;
                lastProjectedFarQuantileValidStrips[q] = 0;
            }
            lastPercentileLowerBound = double.NaN;
            for (int q = 0; q < lastQuantileLowers.Length; ++q) lastQuantileLowers[q] = double.NaN;
            lastUpperBound = double.NaN;
            lastProjectedUpper = double.NaN;
            lastProjectedMidpointError = double.NaN;
            lastQuorumValidStrips = 0;
            lastQuorumLower = double.NaN;
            lastQuorumUpper = double.NaN;
            lastQuorumMidpointSpread = double.NaN;
            if (sceneDefinition == null || sensorCamera == null)
                throw new InvalidOperationException("P0-D references missing.");
            if (slanted) sceneDefinition.BuildSlanted(0.2f);
            else if (edgeOffset != 0f) sceneDefinition.BuildAtEdge(sceneDefinition.EdgeZMetres + edgeOffset);
            else sceneDefinition.Build();
            Physics.SyncTransforms();
            var depth = new CpuRaycastPerfectDepthGenerator().Generate(sensorCamera, imageWidth, imageHeight, 10f);
            if (gaussianSigmaMetres > 0)
                depth = AddGaussianDepthNoise(depth, gaussianSigmaMetres, gaussianSeed);
            foreach (bool validPixel in depth.Valid)
                if (validPixel) ++lastValidBeforeDropout;
            if (dropoutFraction > 0)
                depth = DropDepthPixels(depth, dropoutFraction, dropoutSeed);
            foreach (bool validPixel in depth.Valid)
                if (validPixel) ++lastValidAfterDropout;
            if (outlierFraction > 0)
                depth = InjectDepthOutliers(depth, outlierFraction, outlierMagnitudeMetres,
                    outlierSeed, out lastOutlierCandidates, out lastOutlierCount);
            else
            {
                foreach (bool validPixel in depth.Valid)
                    if (validPixel) ++lastOutlierCandidates;
            }
            if (clusterCount > 0)
            {
                int clusterCandidates, clusterChanged;
                depth = InjectClusteredDepthOutliers(depth, clusterWidth, clusterHeight,
                    clusterCount, outlierMagnitudeMetres, clusterSeed,
                    out clusterCandidates, out clusterChanged, clusterPlacementMode);
                lastOutlierCandidates = clusterCandidates;
                lastOutlierCount = clusterChanged;
            }
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
                    ++lastReconstructedPoints;
                    var s = (int)Math.Floor((p.Value.X + 1f) / 0.25f);
                    var b = (int)Math.Floor(p.Value.Z / BinWidth);
                    // Use the visible near-side floor, not a fixed 0.4 m camera-relative window.
                    // The edge position is not read from ground truth by the detector.
                    if (p.Value.Z >= 0 && p.Value.Z < 0.9f &&
                        p.Value.Y > -0.005f && p.Value.Y < 0.005f)
                        groundPoints.Add(p.Value);
                    if (s >= 0 && s < Strips && b >= 0 && b < Bins)
                        cells[s, b].Add(p.Value);
                }
            if (groundPoints.Count < 3)
            {
                if (verbose) Debug.Log("[WalkEdgeLight P0-E] detection=NONE, reason=INSUFFICIENT_GROUND_POINTS, groundPoints=" + groundPoints.Count);
                return double.NaN;
            }
            var plane = GroundPlaneEstimator.FitLeastSquares(groundPoints);
            if (diagnoseBounds)
                Debug.Log("[WalkEdgeLight P0-E BoundDiag] groundPoints=" + groundPoints.Count +
                    ", cameraY=" + sensorCamera.transform.position.y.ToString("F6") +
                    ", cameraPitch=" + sensorCamera.transform.eulerAngles.x.ToString("F3"));
            var xs = new List<double>();
            var zs = new List<double>();
            double commonLower = double.NegativeInfinity;
            double commonPercentileLower = double.NegativeInfinity;
            double[] commonQuantileLowers = { double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity };
            double[] quantileFractions = { 0.90, 0.95, 0.98, 1.00 };
            double commonUpper = double.PositiveInfinity;
            double commonProjectedUpper = double.PositiveInfinity;
            double[] diagnosticProjectedUppers = {
                double.PositiveInfinity, double.PositiveInfinity,
                double.PositiveInfinity, double.PositiveInfinity
            };
            int[] diagnosticProjectedValidStrips = new int[4];
            double[] diagnosticFarLowers = {
                double.NegativeInfinity, double.NegativeInfinity,
                double.NegativeInfinity, double.NegativeInfinity
            };
            double[] diagnosticFarUppers = {
                double.PositiveInfinity, double.PositiveInfinity,
                double.PositiveInfinity, double.PositiveInfinity
            };
            int[] diagnosticFarValidStrips = new int[4];
            double[] diagnosticFarFractions = { 0.00, 0.02, 0.05, 0.10 };
            int projectedStrips = 0;
            int boundedStrips = 0;
            double quorumLower = double.NegativeInfinity;
            double quorumUpper = double.PositiveInfinity;
            int quorumStrips = 0;
            double quorumMidpointMin = double.PositiveInfinity;
            double quorumMidpointMax = double.NegativeInfinity;
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
                if (bestBin < 0)
                {
                    if (diagnoseBounds)
                        Debug.Log("[WalkEdgeLight P0-F ClusterDiag] strip=" + s +
                            ", status=NO_EDGE_BIN");
                    continue;
                }
                // Refine the coarse 25 mm bin using raw reconstructed points.
                // Only classify samples close to the candidate edge; do not use truth.
                var coarseZ = bestBin * BinWidth;
                double nearMax = double.NegativeInfinity;
                var nearZValues = new List<double>();
                double farMin = double.PositiveInfinity;
                var farZValues = new List<double>();
                double projectedUpper = double.PositiveInfinity;
                for (int b = Math.Max(0, bestBin - 4); b < Math.Min(Bins, bestBin + 5); ++b)
                    foreach (var p in cells[s, b])
                    {
                        var distance = plane.SignedDistance(p);
                        if (distance > -0.005 && distance < 0.005 && p.Z <= coarseZ + 0.10)
                        {
                            nearMax = Math.Max(nearMax, p.Z);
                            nearZValues.Add(p.Z);
                            ++lastNearBoundaryPoints;
                        }
                        else if (distance < -0.010 && p.Z >= coarseZ - 0.10)
                        {
                            farMin = Math.Min(farMin, p.Z);
                            farZValues.Add(p.Z);
                            ++lastFarBoundaryPoints;
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
                if (farZValues.Count > 0 && nearZValues.Count > 0)
                {
                    double nearLower95 = Quantile(nearZValues, 0.95);
                    for (int q = 0; q < diagnosticFarFractions.Length; ++q)
                    {
                        double farLower = Quantile(farZValues, diagnosticFarFractions[q]);
                        if (farLower < nearLower95) continue;
                        diagnosticFarLowers[q] = Math.Max(diagnosticFarLowers[q], nearLower95);
                        diagnosticFarUppers[q] = Math.Min(diagnosticFarUppers[q], farLower);
                        ++diagnosticFarValidStrips[q];
                        if (!double.IsPositiveInfinity(projectedUpper))
                        {
                            diagnosticProjectedUppers[q] = Math.Min(
                                diagnosticProjectedUppers[q],
                                Math.Min(farLower, projectedUpper));
                            ++diagnosticProjectedValidStrips[q];
                        }
                    }
                }
                if (diagnoseBounds)
                {
                    double near95 = nearZValues.Count == 0 ? double.NaN :
                        Quantile(nearZValues, 0.95);
                    double far2 = farZValues.Count == 0 ? double.NaN :
                        Quantile(farZValues, 0.02);
                    Debug.Log("[WalkEdgeLight P0-F ClusterDiag] strip=" + s +
                        ", bestBin=" + bestBin +
                        ", nearCount=" + nearZValues.Count +
                        ", farCount=" + farZValues.Count +
                        ", near95=" + near95.ToString("F9") +
                        ", far2=" + far2.ToString("F9") +
                        ", projectedUpper=" + (double.IsPositiveInfinity(projectedUpper) ?
                            "N/A" : projectedUpper.ToString("F9")) +
                        ", quantileValid=" + (far2 >= near95) +
                        ", projectedValid=" + (!double.IsPositiveInfinity(projectedUpper)) +
                        ", projectedAboveNear=" + (projectedUpper >= near95));
                }
                if (diagnoseBounds)
                    Debug.Log("[WalkEdgeLight P0-E BoundDiag] strip=" + s +
                        ", bestBin=" + bestBin +
                        ", nearMax=" + (double.IsNegativeInfinity(nearMax) ? "N/A" : nearMax.ToString("F9")) +
                        ", farMin=" + (double.IsPositiveInfinity(farMin) ? "N/A" : farMin.ToString("F9")) +
                        ", projectedUpper=" + (double.IsPositiveInfinity(projectedUpper) ? "N/A" : projectedUpper.ToString("F9")));
                var refinedZ = double.IsNegativeInfinity(nearMax) || double.IsPositiveInfinity(farMin)
                    || farMin < nearMax ? coarseZ : (nearMax + farMin) * 0.5;
                // P0-F-12: use robust far-side 2% quantile for interval bounds.
                // Keep the existing refinedZ estimate unchanged for detection regression.
                if (nearZValues.Count > 0 && farZValues.Count > 0)
                {
                    double nearLower95 = Quantile(nearZValues, 0.95);
                    double farLower2 = Quantile(farZValues, 0.02);
                    if (quorumDiagnostic && farLower2 >= nearLower95 &&
                        !double.IsPositiveInfinity(projectedUpper) &&
                        projectedUpper >= nearLower95)
                    {
                        quorumLower = Math.Max(quorumLower, nearLower95);
                        quorumUpper = Math.Min(quorumUpper, Math.Min(farLower2, projectedUpper));
                        ++quorumStrips;
                        double midpoint = (nearLower95 + Math.Min(farLower2, projectedUpper)) * 0.5;
                        quorumMidpointMin = Math.Min(quorumMidpointMin, midpoint);
                        quorumMidpointMax = Math.Max(quorumMidpointMax, midpoint);
                    }
                    if (farLower2 >= nearLower95)
                    {
                        commonLower = Math.Max(commonLower, nearMax);
                        commonPercentileLower = Math.Max(commonPercentileLower, nearLower95);
                        for (int q = 0; q < quantileFractions.Length; ++q)
                            commonQuantileLowers[q] = Math.Max(commonQuantileLowers[q],
                                Quantile(nearZValues, quantileFractions[q]));
                        commonUpper = Math.Min(commonUpper, farLower2);
                        ++boundedStrips;
                        if (!double.IsPositiveInfinity(projectedUpper))
                        {
                            commonProjectedUpper = Math.Min(commonProjectedUpper, projectedUpper);
                            ++projectedStrips;
                        }
                    }
                }
                xs.Add(-1.0 + (s + 0.5) * 0.25);
                zs.Add(refinedZ);
            }
            for (int q = 0; q < 4; ++q)
            {
                lastFarQuantileValidStrips[q] = diagnosticFarValidStrips[q];
                lastProjectedFarQuantileValidStrips[q] = diagnosticProjectedValidStrips[q];
                if (diagnosticProjectedValidStrips[q] > 0)
                    lastProjectedFarQuantileUppers[q] = diagnosticProjectedUppers[q];
                if (diagnosticFarValidStrips[q] > 0)
                {
                    lastFarQuantileLowers[q] = diagnosticFarLowers[q];
                    lastFarQuantileUppers[q] = diagnosticFarUppers[q];
                }
            }
            if (quorumDiagnostic && quorumStrips > 0)
            {
                lastQuorumValidStrips = quorumStrips;
                lastQuorumLower = quorumLower;
                lastQuorumUpper = quorumUpper;
                lastQuorumMidpointSpread = quorumMidpointMax - quorumMidpointMin;
            }
            if (xs.Count < 2)
            {
                if (verbose) Debug.Log((slanted ? "[WalkEdgeLight P0-D Slanted]" : edgeOffset != 0f ? "[WalkEdgeLight P0-E Offset]" : "[WalkEdgeLight P0-D]") + " detection=NONE, stripsDetected=" + xs.Count);
                return double.NaN;
            }
            if (!slanted && boundedStrips == xs.Count && commonPercentileLower <= commonUpper)
            {
                lastRawLowerBound = commonLower;
                lastLowerBound = commonPercentileLower;
                lastPercentileLowerBound = commonPercentileLower;
                for (int q = 0; q < lastQuantileLowers.Length; ++q)
                    lastQuantileLowers[q] = commonQuantileLowers[q];
                lastUpperBound = commonUpper;
                if (projectedStrips == xs.Count && commonProjectedUpper >= commonPercentileLower)
                    lastProjectedUpper = Math.Min(commonUpper, commonProjectedUpper);
            }
            if (diagnoseBounds)
                Debug.Log("[WalkEdgeLight P0-E BoundDiag] combinedLower=" +
                    (double.IsNegativeInfinity(commonLower) ? "N/A" : commonLower.ToString("F9")) +
                    ", combinedFarUpper=" + (double.IsPositiveInfinity(commonUpper) ? "N/A" : commonUpper.ToString("F9")) +
                    ", combinedProjectedUpper=" + (double.IsPositiveInfinity(commonProjectedUpper) ? "N/A" : commonProjectedUpper.ToString("F9")) +
                    ", strips=" + boundedStrips + "/" + projectedStrips + "/" + xs.Count +
                    ", truthZ=" + (sceneDefinition.EdgeZMetres + edgeOffset).ToString("F9"));
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
