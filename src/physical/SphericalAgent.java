package physical;

import math.Vec3;
import processing.core.PApplet;
import tools.Vertex;

import java.util.ArrayList;
import java.util.List;

public class SphericalAgent {
    public final PApplet parent;
    public final Vec3 startPosition;
    public final float radius;
    public Vec3 center;
    public float speed;
    public Vec3 color;

    public List<Vertex> path = new ArrayList<>();
    public int currentMilestone = 0;

    public boolean isPaused = false;

    public SphericalAgent(PApplet parent, Vec3 startPosition, float speed, float radius, Vec3 color) {
        this.parent = parent;
        this.center = Vec3.of(startPosition);
        this.startPosition = Vec3.of(startPosition);
        this.speed = speed;
        this.radius = radius;
        this.color = color;
    }

    public void update(float dt) {
        if (isPaused) {
            return;
        }
        if (currentMilestone < path.size() - 1) {
            // reached next milestone
            if (path.get(currentMilestone + 1).position.minus(center).norm() < 2) {
                currentMilestone++;
                return;
            }
            // move towards next milestone
            Vec3 velocityDir =
                    path.get(currentMilestone + 1).position
                            .minus(center)
                            .normalizeInPlace();
            center.plusInPlace(velocityDir.scale(speed * dt));
        }
    }

    public void draw() {
        // path
        parent.stroke(color.x, color.y, color.z);
        for (int i = 0; i < path.size() - 1; i++) {
            Vec3 v1 = path.get(i).position;
            Vec3 v2 = path.get(i + 1).position;
            parent.line(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        }
        parent.noStroke();
        // agent
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.translate(center.x, center.y, center.z);
        parent.sphere(radius);
        parent.popMatrix();
    }

    public void setPath(List<Vertex> path) {
        this.path = path;
        currentMilestone = 0;
        center.set(startPosition);
    }

    public void stepForward() {
        if (path.size() == 0) {
            return;
        }
        center.set(path.get(currentMilestone).position);
        if (currentMilestone < path.size() - 1) {
            currentMilestone++;
        }
    }

    public void stepBackward() {
        if (path.size() == 0) {
            return;
        }
        center.set(path.get(currentMilestone).position);
        if (currentMilestone > 0) {
            currentMilestone--;
        }
    }

}
