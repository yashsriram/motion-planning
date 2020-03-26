package robot.planning.multiagentgraph;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

class Vertex {
    private final PApplet parent;
    final Vec3 position;
    final boolean isOutsideObstacle;
    final List<Vertex> neighbours = new ArrayList<>();
    final List<Vec3> edgeColors = new ArrayList<>();

    class SearchState {
        float heuristicDistanceToFinish = 0;
        boolean isFinish = false;
        boolean isExplored = false;
        float distanceFromStart = 0;
        List<Vec3> pathFromStart = new ArrayList<>();
        Vec3 color;

        public SearchState() {
            this.color = Vertex.this.isOutsideObstacle ? Vec3.of(1) : Vec3.of(1, 0, 1);
        }

        void reset(Vec3 finishPosition) {
            isFinish = finishPosition.equals(Vertex.this.position);
            heuristicDistanceToFinish = Vertex.this.position.minus(finishPosition).norm();
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

    static Vertex of(PApplet parent, Vec3 position, boolean isOutsideObstacle) {
        return new Vertex(parent, position, isOutsideObstacle);
    }

    private Vertex(PApplet parent, Vec3 position, boolean isOutsideObstacle) {
        this.parent = parent;
        this.position = Vec3.of(position);
        this.isOutsideObstacle = isOutsideObstacle;
        this.searchState = new SearchState();
    }

    void draw() {
        parent.pushMatrix();
        parent.fill(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.stroke(searchState.color.x, searchState.color.y, searchState.color.z);
        parent.point(position.x, position.y, position.z);
        parent.popMatrix();
        if (MultiAgentGraph.DRAW_EDGES) {
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
        return searchState.isFinish;
    }

    @Override
    public String toString() {
        return "Vertex{" +
                "position=" + position +
                '}';
    }
}
