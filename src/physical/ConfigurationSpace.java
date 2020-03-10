package physical;

import math.Vec3;

import java.util.List;

public class ConfigurationSpace {
    public SphericalAgent sphericalAgent;
    public List<SphericalObstacle> sphericalObstacles;

    public ConfigurationSpace(SphericalAgent sphericalAgent, List<SphericalObstacle> sphericalObstacles) {
        this.sphericalAgent = sphericalAgent;
        this.sphericalObstacles = sphericalObstacles;
    }

    public boolean doesIntersectWithObstacle(Vec3 p) {
        for (SphericalObstacle sphericalObstacle: sphericalObstacles) {
            if (p.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                return true;
            }
        }
        return false;
    }

    public boolean doesIntersectWithObstacle(Vec3 p1, Vec3 p2) {
        for (SphericalObstacle sphericalObstacle: sphericalObstacles) {
            Vec3 pb_pa = p2.minus(p1);
            Vec3 pa_pc = p1.minus(sphericalObstacle.center);
            float r = sphericalObstacle.radius + sphericalAgent.radius;
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

}
