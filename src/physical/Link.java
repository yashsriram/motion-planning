package physical;

import math.Vec3;
import processing.core.PApplet;

public class Link {
    public final PApplet parent;
    public Vec3 p1;
    public Vec3 p2;
    public Vec3 color;

    public Link(PApplet parent, Vec3 p1, Vec3 p2, Vec3 color) {
        this.parent = parent;
        this.p1 = p1;
        this.p2 = p2;
        this.color = color;
    }

    public void draw() {
        parent.stroke(color.x, color.y, color.z);
        parent.line(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    }
}
