package robot.planning.graph;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

class Vertex {
    private static final int START_ID = -1;
    private static final int FINISH_ID = 0;

    private static int nextId = 1;

    private static int getNextId() {
        int currId = nextId;
        nextId++;
        return currId;
    }

    private final PApplet parent;
    private final int id;
    final Vec3 position;
    final float heuristicDistanceToFinish;

    final boolean isOutsideObstacle;
    final List<Vertex> neighbours = new ArrayList<>();
    final List<Vec3> edgeColors = new ArrayList<>();

    class SearchState {
        boolean isExplored = false;
        float distanceFromStart = 0;
        List<Vec3> pathFromStart = new ArrayList<>();
        Vec3 color;

        public SearchState() {
            this.color = Vertex.this.isOutsideObstacle ? Vec3.of(1) : Vec3.of(1, 0, 1);
        }

        void reset() {
            if (isOutsideObstacle) {
                color.set(1, 1, 1);
            } else {
                color.set(1, 0, 1);
            }
            isExplored = false;
            distanceFromStart = 0;
            pathFromStart.clear();
        }

        void addToFringeFrom(Vertex parent) {
            color.set(0, 1, 0);
            isExplored = true;
            pathFromStart.addAll(parent.searchState.pathFromStart);
            pathFromStart.add(Vertex.this.position);
        }

        void setExplored() {
            color.set(1, 0, 0);
        }
    }

    final SearchState searchState;

    static Vertex start(PApplet parent, Vec3 position, float distanceToFinish, boolean isOutsideObstacle) {
        return new Vertex(parent, START_ID, position, distanceToFinish, isOutsideObstacle);
    }

    static Vertex finish(PApplet parent, Vec3 position, float distanceToFinish, boolean isOutsideObstacle) {
        return new Vertex(parent, FINISH_ID, position, distanceToFinish, isOutsideObstacle);
    }

    static Vertex of(PApplet parent, Vec3 position, float distanceToFinish, boolean isOutsideObstacle) {
        return new Vertex(parent, getNextId(), position, distanceToFinish, isOutsideObstacle);
    }

    private Vertex(PApplet parent, int id, Vec3 position, float heuristicDistanceToFinish, boolean isOutsideObstacle) {
        this.parent = parent;
        this.id = id;
        this.position = Vec3.of(position);
        this.heuristicDistanceToFinish = heuristicDistanceToFinish;
        this.isOutsideObstacle = isOutsideObstacle;
        this.searchState = new SearchState();
    }

    void draw() {
        parent.pushMatrix();
        parent.fill(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.stroke(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.point(position.x, position.y, position.z);
        parent.popMatrix();
        if (Graph.DRAW_EDGES) {
            for (int i = 0; i < neighbours.size(); ++i) {
                Vertex neighbour = neighbours.get(i);
                Vec3 color = edgeColors.get(i);
                parent.stroke(color.x, color.y, color.z);
                parent.line(this.position.x, this.position.y, this.position.z,
                        neighbour.position.x, neighbour.position.y, neighbour.position.z);
            }
        }
    }

    void addNeighbour(Vertex other, Vec3 color) {
        neighbours.add(other);
        edgeColors.add(color);
    }

    boolean isFinishVertex() {
        return id == FINISH_ID;
    }

    @Override
    public String toString() {
        return "Vertex{" +
                "id=" + id +
                '}';
    }
}
