using System;
using System.Collections.Generic;
using System.Numerics;

namespace WalkEdgeLight.Validation.Reconstruction
{
    public struct Plane3
    {
        public Plane3(Vector3 normal, float offset)
        {
            Normal = normal;
            Offset = offset;
        }

        public Vector3 Normal { get; private set; }
        public float Offset { get; private set; }

        public float SignedDistance(Vector3 point)
        {
            return Vector3.Dot(Normal, point) + Offset;
        }
    }

    public static class GroundPlaneEstimator
    {
        // P0-B starts with an ideal-data least-squares plane fit. Robust outlier
        // rejection is intentionally deferred until degraded synthetic inputs.
        public static Plane3 FitLeastSquares(IList<Vector3> points)
        {
            if (points == null) throw new ArgumentNullException("points");
            if (points.Count < 3) throw new ArgumentException("At least three points are required.", "points");

            var centroid = Vector3.Zero;
            for (var i = 0; i < points.Count; ++i) centroid += points[i];
            centroid /= points.Count;

            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            for (var i = 0; i < points.Count; ++i)
            {
                var r = points[i] - centroid;
                xx += r.X * r.X; xy += r.X * r.Y; xz += r.X * r.Z;
                yy += r.Y * r.Y; yz += r.Y * r.Z; zz += r.Z * r.Z;
            }

            // For a walking surface in P0-B, solve y = a*x + b*z + c.
            // This avoids relying on world-up for the fitted height itself while
            // keeping the ideal validation implementation compact and deterministic.
            var det = xx * zz - xz * xz;
            if (Math.Abs(det) < 1e-12) throw new InvalidOperationException("Ground samples are degenerate.");

            var a = (xy * zz - yz * xz) / det;
            var b = (yz * xx - xy * xz) / det;
            var normal = Vector3.Normalize(new Vector3((float)-a, 1f, (float)-b));
            var offset = -Vector3.Dot(normal, centroid);
            return new Plane3(normal, offset);
        }
    }
}
