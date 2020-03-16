package robot.sensing;

import math.Vec3;
import robot.input.SphericalAgentDescription;
import fixed.SphericalObstacle;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class PlainConfigurationSpace extends ConfigurationSpace {
    final PApplet parent;
    final SphericalAgentDescription sphericalAgentDescription;
    final List<SphericalObstacle> sphericalObstacles;
    final Vec3 minCorner;
    final Vec3 maxCorner;

    public PlainConfigurationSpace(PApplet parent, SphericalAgentDescription sphericalAgentDescription, List<SphericalObstacle> sphericalObstacles, Vec3 minCorner, Vec3 maxCorner) {
        this.parent = parent;
        this.sphericalAgentDescription = sphericalAgentDescription;
        this.sphericalObstacles = sphericalObstacles;
        this.minCorner = minCorner;
        this.maxCorner = maxCorner;
    }

    public boolean doesVertexIntersectSomeObstacle(Vec3 p) {
        for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
            if (p.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgentDescription.radius) {
                return true;
            }
        }
        return false;
    }

    public boolean doesEdgeIntersectSomeObstacle(Vec3 p1, Vec3 p2) {
        for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
            Vec3 pb_pa = p2.minus(p1);
            Vec3 pa_pc = p1.minus(sphericalObstacle.center);
            float r = sphericalObstacle.radius + sphericalAgentDescription.radius;
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
