# WalkEdgeLight

WalkEdgeLight is a project for visualizing nearby walking-surface edges and height discontinuities using camera and depth information such as LiDAR.

The project aims to make difficult-to-see steps, stair edges, and similar walking-surface geometry easier to perceive by overlaying detected geometry on the camera view.

## Project status

WalkEdgeLight is currently in the validation / prototype (PoC) phase.

The initial validation focuses on whether nearby walking-surface geometry can be detected and visualized with sufficient stability using depth information, including PC simulation and real-device experiments.

The current development target includes detecting small height differences around 20 mm. This is a validation target, not a guaranteed detection or safety specification.

## Documentation policy

Project documentation follows **HLDocS 0.6.1-preview**.

During the validation / PoC phase, formal specifications and formal test specifications are intentionally not required. Findings from prototypes and experiments will be used to determine the formal requirements, specifications, and test specifications before production implementation.

HLDocS: https://github.com/mayusaki3/HLDocS

## Development branches

- `main`: stable project baseline
- `develop`: ongoing development and validation

## License

MIT License.
