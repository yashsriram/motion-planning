package physical.configurationspace;

import math.Vec3;

public interface IntersectionChecker {
    public boolean doesIntersectWithObstacle(Vec3 p);

    public boolean doesIntersectWithObstacle(Vec3 p1, Vec3 p2);
}
