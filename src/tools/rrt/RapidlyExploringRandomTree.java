package tools.rrt;

import math.Vec3;
import processing.core.PApplet;
import tools.configurationspace.ConfigurationSpace;

import java.util.Stack;

public class RapidlyExploringRandomTree {
    public static float GROWTH_LIMIT = 10f;
    final PApplet applet;
    final Vertex root;

    public RapidlyExploringRandomTree(PApplet applet, Vec3 rootPosition) {
        this.applet = applet;
        this.root = Vertex.of(applet, rootPosition);
    }

    private Vertex getNearestVertexUnderThis(final Vec3 position) {
        Stack<Vertex> fringe = new Stack<>();
        fringe.add(root);
        float minDistance = position.minus(root.position).norm();
        Vertex nearestVertex = root;

        while (fringe.size() > 0) {
            Vertex node = fringe.pop();
            float distance = position.minus(node.position).norm();
            if (distance < minDistance) {
                minDistance = distance;
                nearestVertex = node;
            }
            fringe.addAll(node.getChildren());
        }
        return nearestVertex;
    }

    public void generateNextNode(ConfigurationSpace configurationSpace) {
        Vec3 newPosition = Vec3.of(0, applet.random(-100, 100), applet.random(-100, 100));
        Vertex nearestVertex = getNearestVertexUnderThis(newPosition);
        Vec3 growth = newPosition.minus(nearestVertex.position);
        if (growth.norm() > GROWTH_LIMIT) {
            newPosition = nearestVertex.position.plus(growth.normalize().scale(GROWTH_LIMIT));
        }
        if (configurationSpace.doesEdgeIntersectSomeObstacle(nearestVertex.position, newPosition)) {
            return;
        }
        nearestVertex.addChild(Vertex.of(applet, newPosition));
    }

    public void generateTree(int numNodes, ConfigurationSpace configurationSpace) {
        for (int i = 0; i < numNodes; i++) {
            generateNextNode(configurationSpace);
        }
    }

    private void drawAllUnderThis(Vertex node) {
        node.draw();
        for (Vertex vertex : node.getChildren()) {
            drawAllUnderThis(vertex);
        }
    }

    public void draw() {
        drawAllUnderThis(root);
    }
}
