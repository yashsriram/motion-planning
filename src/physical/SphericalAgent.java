package physical;

import math.Vec3;
import processing.core.PApplet;

public class SphericalAgent {
    public final PApplet parent;
    public Vec3 center;
    public float radius;
    public Vec3 color;

    public SphericalAgent(PApplet parent, Vec3 center, float radius, Vec3 color) {
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

}
