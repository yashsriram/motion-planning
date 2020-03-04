package physical;

import math.Vec3;
import processing.core.PApplet;

public class CircularObstacle {
    PApplet parent;
    Vec3 center;
    float radius;
    Vec3 color;

    public CircularObstacle(PApplet parent, Vec3 center, float radius, Vec3 color) {
        this.parent = parent;
        this.center = center;
        this.radius = radius;
        this.color = color;
    }

    public void draw() {
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.translate(center.x, center.y, center.z);
        parent.sphere(radius);
        parent.popMatrix();
    }

}
