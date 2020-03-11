package tools;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Vertex {
    public static final int START_ID = -1;
    public static final int FINISH_ID = 0;
    public static boolean DRAW_EDGES = false;

    private static int nextId = 1;

    private static int getNextId() {
        int currId = nextId;
        nextId++;
        return currId;
    }

    public final PApplet parent;
    public final int id;
    public final Vec3 position;
    public final float heuristicDistanceToFinish;

    public boolean isOutsideObstacle = true;
    public final List<Vertex> neighbours = new ArrayList<>();
    public final List<Vec3> edgeColors = new ArrayList<>();

    public class SearchState {
        public boolean isExplored = false;
        public float distanceFromStart = 0;
        public List<Vertex> pathFromStart = new ArrayList<>();
        public Vec3 color = Vec3.of(1);

        public void reset() {
            color.set(1, 1, 1);
            isExplored = false;
            distanceFromStart = 0;
            pathFromStart.clear();
        }

        public void addToFringeFrom(Vertex parent) {
            color.set(0, 1, 0);
            isExplored = true;
            pathFromStart.addAll(parent.searchState.pathFromStart);
            pathFromStart.add(Vertex.this);
        }

        public void setExplored() {
            color.set(1, 0, 0);
        }
    }

    public final SearchState searchState;

    public static Vertex start(PApplet parent, Vec3 position, float distanceToFinish) {
        return new Vertex(parent, START_ID, position, distanceToFinish);
    }

    public static Vertex finish(PApplet parent, Vec3 position, float distanceToFinish) {
        return new Vertex(parent, FINISH_ID, position, distanceToFinish);
    }

    public static Vertex of(PApplet parent, Vec3 position, float distanceToFinish) {
        return new Vertex(parent, getNextId(), position, distanceToFinish);
    }

    private Vertex(PApplet parent, int id, Vec3 position, float heuristicDistanceToFinish) {
        this.parent = parent;
        this.id = id;
        this.position = Vec3.of(position);
        this.heuristicDistanceToFinish = heuristicDistanceToFinish;
        this.searchState = new SearchState();
    }

    public void draw() {
        parent.pushMatrix();
        parent.fill(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.stroke(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.point(position.x, position.y, position.z);
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
