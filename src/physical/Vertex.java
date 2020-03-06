package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Vertex {
    private static final int START_ID = -1;
    private static final int FINISH_ID = 0;
    private static int nextId = 1;

    private static int getNextId() {
        int currId = nextId;
        nextId++;
        return currId;
    }

    public final PApplet parent;
    public final int id;
    public Vec3 position;
    public Vec3 color;
    public List<Vertex> neighbours = new ArrayList<>();
    public List<Vec3> edgeColors = new ArrayList<>();

    public static Vertex start(PApplet parent, Vec3 position, Vec3 color) {
        return new Vertex(parent, START_ID, position, color);
    }

    public static Vertex finish(PApplet parent, Vec3 position, Vec3 color) {
        return new Vertex(parent, FINISH_ID, position, color);
    }

    public static Vertex of(PApplet parent, Vec3 position, Vec3 color) {
        return new Vertex(parent, getNextId(), position, color);
    }

    private Vertex(PApplet parent, int id, Vec3 position, Vec3 color) {
        this.parent = parent;
        this.id = id;
        this.position = position;
        this.color = color;
    }

    public void draw() {
        parent.pushMatrix();
        parent.fill(color.x, color.y, color.z);
        parent.translate(position.x, position.y, position.z);
        parent.box(0.5f);
        parent.popMatrix();
        for (int i = 0;i < neighbours.size(); ++i) {
            Vertex neighbour = neighbours.get(i);
            Vec3 color = edgeColors.get(i);
            parent.stroke(color.x, color.y, color.z);
            parent.line(this.position.x, this.position.y, this.position.z,
                    neighbour.position.x, neighbour.position.y, neighbour.position.z);
        }
    }

    public void addNeighbour(Vertex other, Vec3 color) {
        neighbours.add(other);
        edgeColors.add(color);
    }
}
