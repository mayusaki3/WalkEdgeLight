namespace WalkEdgeLight.Validation.Sensors
{
    public sealed class SensorFrame
    {
        public SensorFrame(long frameId,double timestampSeconds,CameraIntrinsics intrinsics,Pose3 cameraToWorld,DepthFrame depth)
        { FrameId=frameId; TimestampSeconds=timestampSeconds; Intrinsics=intrinsics; CameraToWorld=cameraToWorld; Depth=depth; }
        public long FrameId { get; } public double TimestampSeconds { get; } public CameraIntrinsics Intrinsics { get; }
        public Pose3 CameraToWorld { get; } public DepthFrame Depth { get; }
    }
}
