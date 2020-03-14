package physical;

import math.Vec3;

public class LineSegment2DAgentDescription {
    final Vec3 startPose;
    final float length;

    public LineSegment2DAgentDescription(Vec3 startPosition, float length) {
        this.startPose = startPosition;
        this.length = length;
    }
}
