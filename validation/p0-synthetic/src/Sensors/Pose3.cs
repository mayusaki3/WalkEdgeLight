using System.Numerics;

namespace WalkEdgeLight.Validation.Sensors;

/// <summary>Camera-local to WalkEdgeLight world-space transform.</summary>
public readonly record struct Pose3(
    Vector3 Position,
    Quaternion Rotation)
{
    public Vector3 TransformPoint(Vector3 cameraPoint)
        => Position + Vector3.Transform(cameraPoint, Rotation);
}
