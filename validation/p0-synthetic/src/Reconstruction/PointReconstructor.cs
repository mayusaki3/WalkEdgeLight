using System.Numerics;
using WalkEdgeLight.Validation.Sensors;

namespace WalkEdgeLight.Validation.Reconstruction;

public static class PointReconstructor
{
    public static Vector3? ReconstructCameraPoint(SensorFrame frame, int u, int v)
    {
        var intrinsics = frame.Intrinsics;
        if ((uint)u >= (uint)intrinsics.Width || (uint)v >= (uint)intrinsics.Height)
            throw new ArgumentOutOfRangeException();

        if (frame.Depth.Width != intrinsics.Width || frame.Depth.Height != intrinsics.Height)
            throw new InvalidOperationException("Depth dimensions must match camera intrinsics in P0-A.");

        var index = v * intrinsics.Width + u;
        if (!frame.Depth.Valid[index])
            return null;

        var z = frame.Depth.DepthMetres[index];
        if (!float.IsFinite(z) || z <= 0)
            return null;

        var x = ((u + 0.5) - intrinsics.Cx) / intrinsics.Fx * z;
        var y = -((v + 0.5) - intrinsics.Cy) / intrinsics.Fy * z;

        return new Vector3((float)x, (float)y, z);
    }

    public static Vector3? ReconstructWorldPoint(SensorFrame frame, int u, int v)
    {
        var point = ReconstructCameraPoint(frame, u, v);
        return point is null ? null : frame.CameraToWorld.TransformPoint(point.Value);
    }
}
