package physical;

import math.Vec3;
import processing.core.PApplet;
import processing.core.PShape;

public class SphericalObstacle {
    public final PApplet parent;
    public final Vec3 center;
    public final float radius;
    public final Vec3 color;

    public SphericalObstacle(PApplet parent, Vec3 center, float radius, Vec3 color) {
        this.parent = parent;
        this.center = center;
        this.radius = radius;
        this.color = color;
    }

    public void draw() {
        parent.pushMatrix();
        parent.noStroke();
        parent.fill(color.x, color.y, color.z);
        parent.translate(center.x, center.y, center.z);
        parent.sphere(radius);
        parent.popMatrix();
    }

    public void draw(PShape shape, float normalizedSize) {
        parent.pushMatrix();
        parent.translate(center.x, center.y, center.z);
//        parent.noFill();
//        parent.stroke(color.x, color.y, color.z);
//        parent.sphere(radius);
        parent.scale(2 * radius / normalizedSize);
        parent.shape(shape);
        parent.popMatrix();
    }

}
