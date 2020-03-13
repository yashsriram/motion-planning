package tools.rrt;

import math.Vec3;
import processing.core.PApplet;
import tools.configurationspace.ConfigurationSpace;

import java.util.*;

public class RapidlyExploringRandomTree {
    public static float GROWTH_LIMIT = 10f;
    public static float END_POINT_SIZE = 2f;
    public static float FINISH_SLACK = 5f;
    public static boolean DRAW_TREE = true;

    final PApplet applet;
    final Vec3 startPosition;
    final Vec3 finishPosition;
    final Vertex root;

    public RapidlyExploringRandomTree(PApplet applet, Vec3 startPosition, Vec3 finishPosition) {
        this.applet = applet;
        this.startPosition = Vec3.of(startPosition);
        this.finishPosition = Vec3.of(finishPosition);
        this.root = Vertex.of(applet, startPosition);
    }

    private Vertex getNearestVertexFrom(final Vec3 position) {
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
        Vertex nearestVertex = getNearestVertexFrom(newPosition);
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

    public void draw() {
        // tree
        if (DRAW_TREE) {
            Stack<Vertex> fringe = new Stack<>();
            fringe.add(root);

            while (fringe.size() > 0) {
                Vertex node = fringe.pop();
                node.draw();
                fringe.addAll(node.getChildren());
            }
        }
        // start
        applet.pushMatrix();
        applet.fill(0, 0, 1);
        applet.noStroke();
        applet.translate(startPosition.x, startPosition.y, startPosition.z);
        applet.box(END_POINT_SIZE);
        applet.popMatrix();
        // finish
        applet.pushMatrix();
        applet.fill(0, 1, 0);
        applet.noStroke();
        applet.translate(finishPosition.x, finishPosition.y, finishPosition.z);
        applet.box(END_POINT_SIZE);
        applet.popMatrix();
        // finish slack sphere
        applet.pushMatrix();
        applet.stroke(0, 1, 0);
        applet.noFill();
        applet.translate(finishPosition.x, finishPosition.y, finishPosition.z);
        applet.box(FINISH_SLACK);
        applet.popMatrix();
    }

    public List<Vec3> search() {
        Vertex nearestVertex = getNearestVertexFrom(finishPosition);
        if (finishPosition.minus(nearestVertex.position).norm() < FINISH_SLACK) {
            List<Vec3> path = new ArrayList<>();
            path.add(0, finishPosition);
            Vertex node = nearestVertex.parent;
            while (node != null) {
                path.add(0, node.position);
                node = node.parent;
            }
            return path;
        }
        return Collections.singletonList(startPosition);
    }
}
