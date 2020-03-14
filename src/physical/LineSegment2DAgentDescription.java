package physical;

import math.Vec3;

public class LineSegment2DAgentDescription {
    final Vec3 startPose;
    final Vec3 finishPose;
    public final float length;

    public LineSegment2DAgentDescription(Vec3 startPose, Vec3 finishPose, float length) {
        this.startPose = startPose;
        this.finishPose = finishPose;
        this.length = length;
    }
}
