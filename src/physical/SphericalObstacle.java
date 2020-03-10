package physical;

import math.Vec3;
import processing.core.PApplet;

public class SphericalObstacle {
    public final PApplet parent;
    public final Vec3 center;
    public final float radius;
    public final Vec3 color;
    public boolean isDrawn;

    public SphericalObstacle(PApplet parent, Vec3 center, float radius, Vec3 color) {
        this.parent = parent;
        this.center = center;
        this.radius = radius;
        this.color = color;
        this.isDrawn = true;
    }

    public void draw() {
        if (!isDrawn) {
            return;
        }
        parent.pushMatrix();
        parent.noStroke();
        parent.fill(color.x, color.y, color.z);
        parent.translate(center.x, center.y, center.z);
        parent.sphere(radius);
        parent.popMatrix();
    }

}
