namespace WalkEdgeLight.Validation.Sensors;

/// <summary>Platform-neutral input boundary for WalkEdgeLight validation.</summary>
public sealed record SensorFrame(
    long FrameId,
    double TimestampSeconds,
    CameraIntrinsics Intrinsics,
    Pose3 CameraToWorld,
    DepthFrame Depth);
