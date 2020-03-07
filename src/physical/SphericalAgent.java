package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class SphericalAgent {
    public final PApplet parent;
    public Vec3 center;
    public float radius;
    public Vec3 color;
    public List<Vertex> path = new ArrayList<>();
    private int pathIndex = 0;

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

    public void setPath(List<Vertex> path) {
        this.path = path;
        pathIndex = 0;
    }

    public void stepForward() {
        if (path.size() == 0) {
            return;
        }
        center = path.get(pathIndex).position;
        if (pathIndex < path.size() - 1) {
            pathIndex++;
        }
    }

    public void stepBackward() {
        if (path.size() == 0) {
            return;
        }
        center = path.get(pathIndex).position;
        if (pathIndex > 0) {
            pathIndex--;
        }
    }

}
