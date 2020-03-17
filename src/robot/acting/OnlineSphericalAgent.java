package robot.acting;

import math.Vec3;
import processing.core.PApplet;
import robot.input.SphericalAgentDescription;
import robot.planning.dynamicgraph.DynamicGraph;
import robot.planning.dynamicgraph.Vertex;
import robot.sensing.ConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class OnlineSphericalAgent {
    public static float NEXT_MILESTONE_HINT_SIZE = 2f;
    public static float MILESTONE_REACHED_RADIUS = 2f;
    public static float SENSE_RADIUS = 50f;

    final PApplet parent;
    final SphericalAgentDescription description;
    final ConfigurationSpace configurationSpace;
    final float speed;
    final Vec3 color;
    final Vec3 minCorner;
    final Vec3 maxCorner;
    public final DynamicGraph dynamicGraph;

    Vec3 center;
    List<Vertex> path = new ArrayList<>();
    int currentMilestone = 0;
    float distanceCovered = 0;
    public boolean isPaused = false;

    public OnlineSphericalAgent(final PApplet parent,
                                final SphericalAgentDescription description,
                                final ConfigurationSpace configurationSpace,
                                Vec3 minCorner, Vec3 maxCorner,
                                float speed,
                                Vec3 color,
                                int numSamples,
                                float maxEdgeLen) {
        this.parent = parent;
        this.description = description;
        this.configurationSpace = configurationSpace;
        this.speed = speed;
        this.color = color;
        this.minCorner = minCorner;
        this.maxCorner = maxCorner;
        this.dynamicGraph = new DynamicGraph(parent, description.startPosition, description.finishPosition);
        dynamicGraph.generateGraph(samplePoints(numSamples), maxEdgeLen);

        this.center = Vec3.of(description.startPosition);
        this.path.add(dynamicGraph.start);
    }

    public void update(float dt) {
        if (isPaused) {
            return;
        }
        if (currentMilestone < path.size() - 1) {
            // proceed only if next milestone sensed
            if (!path.get(currentMilestone + 1).isSensed) {
                step();
                return;
            }
            // reached next milestone
            if (path.get(currentMilestone + 1).position.minus(center).norm() < MILESTONE_REACHED_RADIUS) {
                currentMilestone++;
                return;
            }
            // move towards next milestone
            Vec3 velocityDir =
                    path.get(currentMilestone + 1)
                            .position
                            .minus(center)
                            .normalizeInPlace();
            Vec3 displacement = velocityDir.scaleInPlace(speed * dt);
            center.plusInPlace(displacement);
            distanceCovered += displacement.norm();
        }
    }

    public void draw() {
        // graph
        dynamicGraph.draw();
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
        parent.sphere(description.radius);
        parent.popMatrix();
        // next milestone
        if (currentMilestone < path.size() - 1) {
            Vec3 nextMilestonePosition = path.get(currentMilestone + 1).position;
            parent.pushMatrix();
            parent.fill(1, 0, 0);
            parent.translate(nextMilestonePosition.x, nextMilestonePosition.y, nextMilestonePosition.z);
            parent.sphere(description.radius);
            parent.popMatrix();
        }
    }

    public void step() {
        dynamicGraph.senseAndUpdate(center, SENSE_RADIUS, configurationSpace);
        List<Vertex> path = dynamicGraph.aStar(this.path.get(currentMilestone));
        setPath(path);
    }

    private void setPath(List<Vertex> path) {
        this.path = path;
        currentMilestone = 0;
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

    private List<Vec3> samplePoints(int numberOfPoints) {
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
