package robot.acting;

import math.Vec3;
import processing.core.PApplet;
import processing.core.PShape;
import robot.input.SphericalAgentDescription;
import robot.sensing.ConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class SphericalAgent {
    public static float NEXT_MILESTONE_HINT_SIZE = 2f;
    public static float MILESTONE_REACHED_RADIUS = 2f;
    public static float SPRITE_CHANGE_DISTANCE_SCALE = 20f;

    final PApplet parent;
    final SphericalAgentDescription description;
    final ConfigurationSpace configurationSpace;
    final float speed;
    final Vec3 color;
    final Vec3 minCorner;
    final Vec3 maxCorner;

    Vec3 center;
    List<Vec3> path = new ArrayList<>();
    int currentMilestone = 0;
    float distanceCovered = 0;
    public boolean isPaused = false;

    public SphericalAgent(final PApplet parent, final SphericalAgentDescription description, final ConfigurationSpace configurationSpace, Vec3 minCorner, Vec3 maxCorner, float speed, Vec3 color) {
        this.parent = parent;
        this.description = description;
        this.configurationSpace = configurationSpace;
        this.speed = speed;
        this.color = color;

        this.center = Vec3.of(description.startPosition);
        this.minCorner = minCorner;
        this.maxCorner = maxCorner;
    }

    public void update(float dt) {
        if (isPaused) {
            return;
        }
        if (currentMilestone < path.size() - 1) {
            // reached next milestone
            if (path.get(currentMilestone + 1).minus(center).norm() < MILESTONE_REACHED_RADIUS) {
                currentMilestone++;
                return;
            }
            // move towards next milestone
            Vec3 velocityDir =
                    path.get(currentMilestone + 1)
                            .minus(center)
                            .normalizeInPlace();
            Vec3 displacement = velocityDir.scaleInPlace(speed * dt);
            center.plusInPlace(displacement);
            distanceCovered += displacement.norm();
        }
    }

    public void smoothUpdate(float dt) {
        if (isPaused) {
            return;
        }
        if (currentMilestone < path.size() - 1) {
            // reached next milestone
            if (path.get(currentMilestone + 1).minus(center).norm() < MILESTONE_REACHED_RADIUS) {
                currentMilestone++;
                return;
            }
            // next next milestone lookup
            if (currentMilestone < path.size() - 2) {
                boolean blocked = configurationSpace.doesEdgeIntersectSomeObstacle(path.get(currentMilestone + 2), center);
                if (!blocked) {
                    currentMilestone++;
                }
            }
            // move towards next milestone
            Vec3 velocityDir =
                    path.get(currentMilestone + 1)
                            .minus(center)
                            .normalizeInPlace();
            Vec3 displacement = velocityDir.scaleInPlace(speed * dt);
            center.plusInPlace(displacement);
            distanceCovered += displacement.norm();
        }
    }

    public void draw() {
        // path
        parent.stroke(color.x, color.y, color.z);
        for (int i = 0; i < path.size() - 1; i++) {
            Vec3 v1 = path.get(i);
            Vec3 v2 = path.get(i + 1);
            parent.line(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        }
        parent.noStroke();
        // agent
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.translate(center.x, center.y, center.z);
        parent.sphere(description.radius);
        parent.popMatrix();
        // next milestone
        if (currentMilestone < path.size() - 1) {
            Vec3 nextMilestonePosition = path.get(currentMilestone + 1);
            parent.pushMatrix();
            parent.fill(1, 0, 0);
            parent.translate(nextMilestonePosition.x, nextMilestonePosition.y, nextMilestonePosition.z);
            parent.sphere(description.radius);
            parent.popMatrix();
        }
    }

    public void draw(PShape shape, float normalizedSize) {
        // path
        parent.stroke(color.x, color.y, color.z);
        for (int i = 0; i < path.size() - 1; i++) {
            Vec3 v1 = path.get(i);
            Vec3 v2 = path.get(i + 1);
            parent.line(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        }
        parent.noStroke();

        // agent
        parent.pushMatrix();
        parent.translate(center.x, center.y, center.z);
//        parent.stroke(color.x, color.y, color.z);
//        parent.noFill();
//        parent.sphere(description.radius);
        if (currentMilestone < path.size() - 1) {
            Vec3 direction = path.get(currentMilestone + 1).minus(center);
            parent.rotateY((float) Math.atan2(direction.x, direction.z));
        }
        parent.scale(2 * description.radius / normalizedSize);
        parent.shape(shape);
        parent.popMatrix();

        // next milestone
        if (currentMilestone < path.size() - 1) {
            Vec3 nextMilestonePosition = path.get(currentMilestone + 1);
            parent.pushMatrix();
            parent.fill(1, 0, 0);
            parent.noStroke();
            parent.translate(nextMilestonePosition.x, nextMilestonePosition.y, nextMilestonePosition.z);
            parent.sphere(NEXT_MILESTONE_HINT_SIZE);
            parent.popMatrix();
        }
    }

    public void draw(List<PShape> shapes, float normalizedSize) {
        // path
        parent.stroke(color.x, color.y, color.z);
        for (int i = 0; i < path.size() - 1; i++) {
            Vec3 v1 = path.get(i);
            Vec3 v2 = path.get(i + 1);
            parent.line(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        }
        parent.noStroke();

        // agent
        parent.pushMatrix();
        parent.translate(center.x, center.y, center.z);
//        parent.stroke(color.x, color.y, color.z);
//        parent.noFill();
//        parent.sphere(description.radius);
        if (currentMilestone < path.size() - 1) {
            Vec3 direction = path.get(currentMilestone + 1).minus(center);
            parent.rotateY((float) Math.atan2(direction.x, direction.z));
        }
        parent.scale(2 * description.radius / normalizedSize);
        parent.shape(shapes.get((int) ((distanceCovered / SPRITE_CHANGE_DISTANCE_SCALE) % shapes.size())));
        parent.popMatrix();

        // next milestone
        if (currentMilestone < path.size() - 1) {
            Vec3 nextMilestonePosition = path.get(currentMilestone + 1);
            parent.pushMatrix();
            parent.fill(1, 0, 0);
            parent.noStroke();
            parent.translate(nextMilestonePosition.x, nextMilestonePosition.y, nextMilestonePosition.z);
            parent.sphere(NEXT_MILESTONE_HINT_SIZE);
            parent.popMatrix();
        }
    }

    public void setPath(List<Vec3> path) {
        this.path = path;
        currentMilestone = 0;
        center.set(description.startPosition);
    }

    public void stepForward() {
        if (path.size() == 0) {
            return;
        }
        center.set(path.get(currentMilestone));
        if (currentMilestone < path.size() - 1) {
            currentMilestone++;
        }
    }

    public void stepBackward() {
        if (path.size() == 0) {
            return;
        }
        center.set(path.get(currentMilestone));
        if (currentMilestone > 0) {
            currentMilestone--;
        }
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

}
