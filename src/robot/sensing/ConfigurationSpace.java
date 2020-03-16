package robot.sensing;

import math.Vec3;

import java.util.List;

public abstract class ConfigurationSpace {
    public abstract boolean doesVertexIntersectSomeObstacle(Vec3 p);

    public abstract boolean doesEdgeIntersectSomeObstacle(Vec3 p1, Vec3 p2);

    public abstract List<Vec3> samplePoints(int numberOfPoints);

    public abstract Vec3 samplePoint();

    public abstract void draw();
}
