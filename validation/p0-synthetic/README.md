# P0 Synthetic Validation

P0 validates WalkEdgeLight geometry processing under ideal synthetic conditions before introducing real sensor characteristics.

## Scope

- P0-A: Sensor geometry validation
- P0-B: Ground-plane estimation
- P0-C: Height-discontinuity detection
- P0-D: Edge geometry
- P0-E: Parameter sweep

The first implementation target is P0-A.

## P0-A

Generate a flat floor and a parameterized single step in Unity, produce perfect synthetic Z-depth, reconstruct 3D points through the common SensorFrame boundary, and compare the reconstructed geometry with independently generated ground truth.

No sensor noise, temporal processing, ARKit, ARCore, product UI, or hazard classification is included at this stage.

## Directory policy

- `unity/`: Unity project used to generate the synthetic scene, camera, and perfect depth.
- `src/`: validation code that should remain independent from Unity where practical.
- `experiments/`: experiment configurations.
- `results/`: generated validation results. Large or transient generated data should not be committed.

The structure may change as the PoC develops.
