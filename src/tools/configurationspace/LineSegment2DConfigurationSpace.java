package tools.configurationspace;

import math.Vec3;
import physical.LineSegment2DAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class LineSegment2DConfigurationSpace extends ConfigurationSpace {
    final PApplet parent;
    final LineSegment2DAgentDescription lineSegment2DAgentDescription;
    final List<SphericalObstacle> sphericalObstacles;
    final Vec3 minCorner;
    final Vec3 maxCorner;

    public LineSegment2DConfigurationSpace(PApplet parent, LineSegment2DAgentDescription lineSegment2DAgentDescription, List<SphericalObstacle> sphericalObstacles, Vec3 minCorner, Vec3 maxCorner) {
        this.parent = parent;
        this.lineSegment2DAgentDescription = lineSegment2DAgentDescription;
        this.sphericalObstacles = sphericalObstacles;
        this.minCorner = minCorner;
        this.maxCorner = maxCorner;
    }

    public boolean doesVertexIntersectSomeObstacle(Vec3 p) {

        return false;
    }

    public boolean doesEdgeIntersectSomeObstacle(Vec3 p1, Vec3 p2) {

        return false;
    }

    public List<Vec3> samplePoints(int numberOfPoints) {
        List<Vec3> samples = new ArrayList<>();
        for (int i = 0; i < numberOfPoints; i++) {
            samples.add(Vec3.of(
                    parent.random(minCorner.x, maxCorner.x),
                    parent.random(minCorner.y, maxCorner.y),
                    parent.random(minCorner.z, maxCorner.z)
            ));
        }
        return samples;
    }

    public Vec3 samplePoint() {
        return Vec3.of(
                parent.random(minCorner.x, maxCorner.x),
                parent.random(minCorner.y, maxCorner.y),
                parent.random(minCorner.z, maxCorner.z)
        );
    }

    public void draw() {
    }
}
