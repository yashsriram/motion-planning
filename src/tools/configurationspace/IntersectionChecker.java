package tools.configurationspace;

import math.Vec3;

public interface IntersectionChecker {
    public boolean doesVertexIntersectSomeObstacle(Vec3 p);

    public boolean doesEdgeIntersectSomeObstacle(Vec3 p1, Vec3 p2);
}
