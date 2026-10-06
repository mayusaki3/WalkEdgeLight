using System.Numerics;
namespace WalkEdgeLight.Validation.Sensors
{
    public readonly struct Pose3
    {
        public Pose3(Vector3 position, Quaternion rotation) { Position=position; Rotation=rotation; }
        public Vector3 Position { get; } public Quaternion Rotation { get; }
        public Vector3 TransformPoint(Vector3 cameraPoint) { return Position + Vector3.Transform(cameraPoint, Rotation); }
    }
}
