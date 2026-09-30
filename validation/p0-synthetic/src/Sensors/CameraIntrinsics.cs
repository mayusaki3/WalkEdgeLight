namespace WalkEdgeLight.Validation.Sensors;

public readonly record struct CameraIntrinsics(
    double Fx,
    double Fy,
    double Cx,
    double Cy,
    int Width,
    int Height);
