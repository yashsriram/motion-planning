package physical;

import math.Vec3;

public class SphericalAgentDescription {
    final Vec3 startPosition;
    public final float radius;

    public SphericalAgentDescription(Vec3 startPosition, float radius) {
        this.startPosition = Vec3.of(startPosition);
        this.radius = radius;
    }
}
