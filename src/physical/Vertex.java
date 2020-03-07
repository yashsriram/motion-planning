package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Vertex {
    public static final int START_ID = -1;
    public static final int FINISH_ID = 0;
    public static boolean DRAW_EDGES = true;

    private static int nextId = 1;

    private static int getNextId() {
        int currId = nextId;
        nextId++;
        return currId;
    }

    public final PApplet parent;
    public final int id;
    public final Vec3 position;

    // Depends on allowed configuration-space
    public boolean canBeReached = true;
    public final float heuristicDistanceToFinish;

    // Depends on max edge length
    public List<Vertex> neighbours = new ArrayList<>();
    // Depends on allowed configuration-space and cannotBeReached
    public List<Vec3> edgeColors = new ArrayList<>();

    // Can change with search calls
    public Vec3 color;
    public boolean isExplored = false;
    public float distanceFromStart = 0;

    public static Vertex start(PApplet parent, Vec3 position, float distanceToFinish, Vec3 color) {
        return new Vertex(parent, START_ID, position, distanceToFinish, color);
    }

    public static Vertex finish(PApplet parent, Vec3 position, float distanceToFinish, Vec3 color) {
        return new Vertex(parent, FINISH_ID, position, distanceToFinish, color);
    }

    public static Vertex of(PApplet parent, Vec3 position, float distanceToFinish, Vec3 color) {
        return new Vertex(parent, getNextId(), position, distanceToFinish, color);
    }

    private Vertex(PApplet parent, int id, Vec3 position, float heuristicDistanceToFinish, Vec3 color) {
        this.parent = parent;
        this.id = id;
        this.position = position;
        this.heuristicDistanceToFinish = heuristicDistanceToFinish;
        this.color = color;
    }

    public void draw() {
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.noStroke();
        // parent.text(id, position.x, position.y, position.z);
        parent.translate(position.x, position.y, position.z);
        parent.box(1f);
        parent.popMatrix();
        if (DRAW_EDGES) {
            for (int i = 0; i < neighbours.size(); ++i) {
                Vertex neighbour = neighbours.get(i);
                Vec3 color = edgeColors.get(i);
                parent.stroke(color.x, color.y, color.z);
                parent.line(this.position.x, this.position.y, this.position.z,
                        neighbour.position.x, neighbour.position.y, neighbour.position.z);
            }
        }
    }

    public void addNeighbour(Vertex other, Vec3 color) {
        neighbours.add(other);
        edgeColors.add(color);
    }

    @Override
    public String toString() {
        return "Vertex{" +
                "id=" + id +
                '}';
    }
}
