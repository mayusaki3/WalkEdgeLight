# Unity P0 Probe

The Unity validation project is disposable and is not the canonical source.

Create a new Unity 6 LTS project, then deploy the WalkEdgeLight P0 probe from the repository:

```powershell
.\validation\p0-synthetic\probe\BuildProbeAssets.ps1 -UnityProjectDirectory "D:\path\to\WalkEdgeLightProbe"
```

The script rebuilds `probe/build/` and replaces only:

```text
Assets/WalkEdgeLight/Probe/
```

in the target Unity project.

After Unity finishes compiling, run:

```text
WalkEdgeLight > Probe > Setup P0-A
WalkEdgeLight > Probe > Run P0-A
```

Setup is repeatable. It recreates the validation root, camera, single-step scene, and component references.

The initial P0-A setup uses:

- camera height: 1.2 m
- step height: 20 mm
- depth: 160 x 120
- CPU raycast perfect depth
- no sensor noise
