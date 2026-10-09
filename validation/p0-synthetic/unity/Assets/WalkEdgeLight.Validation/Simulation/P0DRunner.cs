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
        private double lastPercentileLowerBound = double.NaN;
        private readonly double[] lastQuantileLowers = new double[4];
        private double lastUpperBound = double.NaN;
        private double lastProjectedUpper = double.NaN;
        private double lastProjectedMidpointError = double.NaN;
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
                        bool rawValid = !double.IsNaN(lastLowerBound) &&
                            !double.IsNaN(lastProjectedUpper) &&
                            lastProjectedUpper >= lastLowerBound;
                        bool percentileValid = !double.IsNaN(lastPercentileLowerBound) &&
                            !double.IsNaN(lastProjectedUpper) &&
                            lastProjectedUpper >= lastPercentileLowerBound;
                        if (rawValid)
                        {
                            ++rawBounded;
                            rawWidthSum += (lastProjectedUpper - lastLowerBound) * 1000;
                            rawErrorSum += Math.Abs(((lastLowerBound + lastProjectedUpper) * 0.5 - truthZ) * 1000);
                            if (ContainsWithTolerance(lastLowerBound, lastProjectedUpper, truthZ)) ++rawCovered;
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
                            " m, rawLower=" + lastLowerBound.ToString("F9") +
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
                    bool hasBounds = !double.IsNaN(lastLowerBound) &&
                        !double.IsNaN(lastProjectedUpper) &&
                        lastProjectedUpper >= lastLowerBound;
                    if (!hasBounds)
                    {
                        Debug.Log("[WalkEdgeLight P0-F NoiseBounds] sigma=" +
                            sigmaMm[level].ToString("F1") + " mm, edgeOffset=" +
                            offsets[k].ToString("F3") + " m, bounded=False");
                        continue;
                    }
                    ++bounded;
                    double truthZ = sceneDefinition.EdgeZMetres + offsets[k];
                    double lowerMissMm = Math.Max(0, (lastLowerBound - truthZ - BoundToleranceMetres) * 1000);
                    double upperMissMm = Math.Max(0, (truthZ - lastProjectedUpper - BoundToleranceMetres) * 1000);
                    bool contains = lowerMissMm <= 0 && upperMissMm <= 0;
                    if (contains) ++covered;
                    if (lowerMissMm > 0) { ++lowerMissCount; maximumLowerMissMm = Math.Max(maximumLowerMissMm, lowerMissMm); }
                    if (upperMissMm > 0) { ++upperMissCount; maximumUpperMissMm = Math.Max(maximumUpperMissMm, upperMissMm); }
                    Debug.Log("[WalkEdgeLight P0-F NoiseBounds] sigma=" +
                        sigmaMm[level].ToString("F1") + " mm, edgeTruth=" +
                        truthZ.ToString("F9") + " m, rawLower=" +
                        lastLowerBound.ToString("F9") + " m, rawUpper=" +
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

        private double RunInternal(bool slanted, float edgeOffset = 0f, bool verbose = true, int imageWidth = 160, int imageHeight = 120, bool diagnoseBounds = false, double gaussianSigmaMetres = 0, int gaussianSeed = 20261009)
        {
            lastLowerBound = double.NaN;
            lastPercentileLowerBound = double.NaN;
            for (int q = 0; q < lastQuantileLowers.Length; ++q) lastQuantileLowers[q] = double.NaN;
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
            if (gaussianSigmaMetres > 0)
                depth = AddGaussianDepthNoise(depth, gaussianSigmaMetres, gaussianSeed);
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
                var nearZValues = new List<double>();
                double farMin = double.PositiveInfinity;
                double projectedUpper = double.PositiveInfinity;
                for (int b = Math.Max(0, bestBin - 4); b < Math.Min(Bins, bestBin + 5); ++b)
                    foreach (var p in cells[s, b])
                    {
                        var distance = plane.SignedDistance(p);
                        if (distance > -0.005 && distance < 0.005 && p.Z <= coarseZ + 0.10)
                        {
                            nearMax = Math.Max(nearMax, p.Z);
                            nearZValues.Add(p.Z);
                        }
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
                if (diagnoseBounds)
                    Debug.Log("[WalkEdgeLight P0-E BoundDiag] strip=" + s +
                        ", bestBin=" + bestBin +
                        ", nearMax=" + (double.IsNegativeInfinity(nearMax) ? "N/A" : nearMax.ToString("F9")) +
                        ", farMin=" + (double.IsPositiveInfinity(farMin) ? "N/A" : farMin.ToString("F9")) +
                        ", projectedUpper=" + (double.IsPositiveInfinity(projectedUpper) ? "N/A" : projectedUpper.ToString("F9")));
                var refinedZ = double.IsNegativeInfinity(nearMax) || double.IsPositiveInfinity(farMin)
                    || farMin < nearMax ? coarseZ : (nearMax + farMin) * 0.5;
                if (!double.IsNegativeInfinity(nearMax) && !double.IsPositiveInfinity(farMin) && farMin >= nearMax)
                {
                    commonLower = Math.Max(commonLower, nearMax);
                    commonPercentileLower = Math.Max(commonPercentileLower, Quantile(nearZValues, 0.95));
                    for (int q = 0; q < quantileFractions.Length; ++q)
                        commonQuantileLowers[q] = Math.Max(commonQuantileLowers[q], Quantile(nearZValues, quantileFractions[q]));
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
                lastPercentileLowerBound = commonPercentileLower;
                for (int q = 0; q < lastQuantileLowers.Length; ++q)
                    lastQuantileLowers[q] = commonQuantileLowers[q];
                lastUpperBound = commonUpper;
                if (projectedStrips == xs.Count && commonProjectedUpper >= commonLower)
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
