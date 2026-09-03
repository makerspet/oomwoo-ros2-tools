`slam_toolbox` wishlist:

### Desirable
- compute, output scan match quality - e.g. to detect robot-lost downstream
- output absolute (not normalized) scan match (per-point) scores

### Importance TBD
- configurable beam noise model, including 1/d^2 noise for triangulation-based 2D LiDARs

### Low importance
- match scan, drop scan points that don't match map, rerun scan matching to avoid transient obstacles influencing pose
- a localization mode switch to turn off new static obstacle addition