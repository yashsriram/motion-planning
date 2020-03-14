package tools.configurationspace;

import math.Vec3;
import physical.LineSegment2DAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class LineSegment2DConfigurationSpace extends ConfigurationSpace {
    final PApplet parent;
    final LineSegment2DAgentDescription description;
    final List<SphericalObstacle> sphericalObstacles;
    final Vec3 minCorner;
    final Vec3 maxCorner;
    public final float orientationScale;

    public LineSegment2DConfigurationSpace(PApplet parent, LineSegment2DAgentDescription description, List<SphericalObstacle> sphericalObstacles, Vec3 minCorner, Vec3 maxCorner, float orientationScale) {
        this.parent = parent;
        this.description = description;
        this.sphericalObstacles = sphericalObstacles;
        this.minCorner = minCorner;
        this.maxCorner = maxCorner;
        this.orientationScale = orientationScale;
    }

    public boolean doesVertexIntersectSomeObstacle(final Vec3 pose) {
        Vec3 p1Position = Vec3.of(pose);
        p1Position.x = 0;
        Vec3 p2Position = Vec3.of(pose);
        p2Position.x = 0;
        Vec3 halfLength = Vec3.of(0, (float) (Math.sin(pose.x / orientationScale) * description.length / 2), (float) (Math.cos(pose.x / orientationScale) * description.length / 2));
        Vec3 p1 = p1Position.minusInPlace(halfLength);
        Vec3 p2 = p2Position.plusInPlace(halfLength);

        for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
            Vec3 pb_pa = p2.minus(p1);
            Vec3 pa_pc = p1.minus(sphericalObstacle.center);
            float r = sphericalObstacle.radius;
            float a = pb_pa.dot(pb_pa);
            float c = pa_pc.dot(pa_pc) - r * r;
            float b = 2 * pb_pa.dot(pa_pc);
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                float t1 = (float) ((-b + Math.sqrt(discriminant)) / (2 * a));
                float t2 = (float) ((-b - Math.sqrt(discriminant)) / (2 * a));
                // Intersection with line segment only possible iff at least one of the solutions lies in [0, 1]
                if ((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1)) {
                    return true;
                }
            }
        }

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
