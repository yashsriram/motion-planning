package physical;

import math.Vec3;
import processing.core.PApplet;
import tools.configurationspace.ConfigurationSpace;
import tools.configurationspace.LineSegment2DConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class LineSegment2DAgent {
    final PApplet parent;
    final LineSegment2DAgentDescription description;
    final LineSegment2DConfigurationSpace configurationSpace;
    final float speed;
    final Vec3 color;

    Vec3 pose;
    List<Vec3> path = new ArrayList<>();
    int currentMilestone = 0;
    public boolean isPaused = false;

    public LineSegment2DAgent(final PApplet parent, final LineSegment2DAgentDescription description, final LineSegment2DConfigurationSpace configurationSpace, float speed, Vec3 color) {
        this.parent = parent;
        this.description = description;
        this.configurationSpace = configurationSpace;
        this.speed = speed;
        this.color = color;

        this.pose = Vec3.of(description.startPose);
    }

    public void update(float dt) {
        if (isPaused) {
            return;
        }
        if (currentMilestone < path.size() - 1) {
            // reached next milestone
            if (path.get(currentMilestone + 1).minus(pose).norm() < 2) {
                currentMilestone++;
                pose.set(path.get(currentMilestone));
                return;
            }
            // move towards next milestone
            Vec3 velocityDir =
                    path.get(currentMilestone + 1)
                            .minus(pose)
                            .normalizeInPlace();
            pose.plusInPlace(velocityDir.scale(speed * dt));
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
        parent.stroke(color.x, color.y, color.z);
        Vec3 halfLength = Vec3.of(0, (float) (Math.sin(pose.x) * description.length / 2), (float) (Math.cos(pose.x) * description.length / 2));
        Vec3 e1 = pose.minus(halfLength);
        Vec3 e2 = pose.plus(halfLength);
        parent.line(0, e1.y, e1.z, 0, e2.y, e2.z);
        parent.translate(0, pose.y, pose.z);
        parent.box(1f);
        parent.translate(0, halfLength.y, halfLength.z);
        parent.box(1f);
        parent.popMatrix();
        // next milestone
        if (currentMilestone < path.size() - 1) {
            Vec3 nextMilestonePosition = path.get(currentMilestone + 1);
            parent.pushMatrix();
            parent.fill(1, 0, 0);
            parent.noStroke();
            parent.translate(0, nextMilestonePosition.y, nextMilestonePosition.z);
            parent.sphere(1f);
            parent.popMatrix();
        }
    }

    public void setPath(List<Vec3> path) {
        this.path = path;
        currentMilestone = 0;
        pose.set(description.startPose);
    }

    public void stepForward() {
        if (path.size() == 0) {
            return;
        }
        pose.set(path.get(currentMilestone));
        if (currentMilestone < path.size() - 1) {
            currentMilestone++;
        }
    }

    public void stepBackward() {
        if (path.size() == 0) {
            return;
        }
        pose.set(path.get(currentMilestone));
        if (currentMilestone > 0) {
            currentMilestone--;
        }
    }

}
