package physical;

import math.Vec3;
import processing.core.PApplet;

public class Milestone {
    public final PApplet parent;
    public Vec3 position;
    public Vec3 color;

    public Milestone(PApplet parent, Vec3 position, Vec3 color) {
        this.parent = parent;
        this.position = position;
        this.color = color;
    }

    public void draw() {
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.translate(position.x, position.y, position.z);
        parent.box(1);
        parent.popMatrix();
    }
}
