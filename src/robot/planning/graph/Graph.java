package robot.planning.graph;

import math.Vec3;
import processing.core.PApplet;
import robot.sensing.ConfigurationSpace;

import java.util.*;

public class Graph {
    public static boolean DRAW_VERTICES = true;
    public static boolean DRAW_EDGES = false;
    public static float END_POINT_SIZE = 2f;

    final PApplet parent;
    final Vertex start;
    final Vertex finish;
    final List<Vertex> vertices = new ArrayList<>();

    public Graph(PApplet parent, Vec3 startPosition, Vec3 finishPosition) {
        this.parent = parent;
        this.start = Vertex.start(parent, startPosition, finishPosition.minus(startPosition).norm());
        this.finish = Vertex.finish(parent, finishPosition, 0);
        this.vertices.add(start);
        this.vertices.add(finish);
    }

    public void generateVertices(List<Vec3> newVertexPositions, ConfigurationSpace configurationSpace) {
        for (Vec3 position : newVertexPositions) {
            float distanceToFinish = finish.position.minus(position).norm();
            vertices.add(Vertex.of(
                    parent,
                    position,
                    distanceToFinish));
        }
        int numVerticesCulled = 0;
        for (Vertex vertex : vertices) {
            if (configurationSpace.doesVertexIntersectSomeObstacle(vertex.position)) {
                vertex.searchState.color = Vec3.of(1, 0, 1);
                vertex.isOutsideObstacle = false;
                numVerticesCulled++;
            }
        }
        PApplet.println("# vertices before culling: " + vertices.size());
        PApplet.println("# vertices culled: " + numVerticesCulled);
        PApplet.println("# vertices after culling: " + (vertices.size() - numVerticesCulled));
    }

    public void generateAdjacencies(float maxEdgeLen, ConfigurationSpace configurationSpace) {
        int numEdges = 0;
        int numEdgesCulled = 0;
        for (int i = 0; i < vertices.size() - 1; ++i) {
            for (int j = i + 1; j < vertices.size(); j++) {
                Vertex v1 = vertices.get(i);
                Vertex v2 = vertices.get(j);
                if (v1.position.minus(v2.position).norm() <= maxEdgeLen) {
                    // Check for intersection with spherical obstacle
                    if (!v1.isOutsideObstacle || !v2.isOutsideObstacle || configurationSpace.doesEdgeIntersectSomeObstacle(v1.position, v2.position)) {
                        numEdgesCulled++;
                    } else {
                        v1.addNeighbour(v2, Vec3.of(1));
                        v2.addNeighbour(v1, Vec3.of(1));
                        numEdges++;
                    }
                }
            }
        }
        PApplet.println("# edges culled: " + numEdgesCulled);
        PApplet.println("# edges generated: " + numEdges);
    }

    public void clearAdjacencies() {
        for (Vertex vertex : vertices) {
            vertex.neighbours.clear();
            vertex.edgeColors.clear();
            vertex.searchState.reset();
        }
    }

    public void draw() {
        if (DRAW_VERTICES) {
            for (Vertex vertex : vertices) {
                vertex.draw();
            }
        }
        // Start
        parent.pushMatrix();
        parent.fill(0, 0, 1);
        parent.noStroke();
        parent.translate(start.position.x, start.position.y, start.position.z);
        parent.box(END_POINT_SIZE);
        parent.popMatrix();
        // Finish
        parent.pushMatrix();
        parent.fill(0, 1, 0);
        parent.noStroke();
        parent.translate(finish.position.x, finish.position.y, finish.position.z);
        parent.box(END_POINT_SIZE);
        parent.popMatrix();
    }

    private void resetSearchState() {
        PApplet.println("Resetting search states of vertices");
        for (Vertex v : vertices) {
            if (v.isOutsideObstacle) {
                v.searchState.reset();
            }
        }
    }

    private void addToFringe(final Stack<Vertex> fringe, final Vertex current, final Vertex next) {
        fringe.add(next);
        next.searchState.addToFringeFrom(current);
    }

    public List<Vec3> dfs() {
        PApplet.println("DFS");

        resetSearchState();
        final Stack<Vertex> fringe = new Stack<>();
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.pop();
            numVerticesExplored++;
            // Check if finish
            if (current.isFinishVertex()) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return finish.searchState.pathFromStart;
            }
            // Mark this vertex as explored
            current.searchState.setExplored();
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.isOutsideObstacle && !neighbour.searchState.isExplored) {
                    addToFringe(fringe, current, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
        return Collections.singletonList(start.position);
    }

    private void addToFringe(final Queue<Vertex> fringe, final Vertex current, final Vertex next) {
        next.searchState.distanceFromStart = current.searchState.distanceFromStart + next.position.minus(current.position).norm();
        fringe.add(next);
        next.searchState.addToFringeFrom(current);
    }

    private List<Vec3> search(final Queue<Vertex> fringe) {
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            numVerticesExplored++;
            // Check if finish
            if (current.isFinishVertex()) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return finish.searchState.pathFromStart;
            }
            // Mark this vertex as explored
            current.searchState.setExplored();
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.isOutsideObstacle && !neighbour.searchState.isExplored) {
                    addToFringe(fringe, current, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
        return Collections.singletonList(start.position);
    }

    public List<Vec3> bfs() {
        PApplet.println("BFS");
        resetSearchState();
        return search(new LinkedList<>());
    }

    public List<Vec3> ucs() {
        PApplet.println("UCS");
        resetSearchState();
        return search(new PriorityQueue<>((v1, v2) ->
                (int) (v1.searchState.distanceFromStart - v2.searchState.distanceFromStart)));
    }

    public List<Vec3> aStar() {
        PApplet.println("A*");
        resetSearchState();
        return search(new PriorityQueue<>((v1, v2) -> (int) (
                (v1.searchState.distanceFromStart + v1.heuristicDistanceToFinish)
                        - (v2.searchState.distanceFromStart + v2.heuristicDistanceToFinish)
        )));
    }

    public List<Vec3> weightedAStar(final float epislon) {
        PApplet.println("Weighted A* with epsilon = " + epislon);
        resetSearchState();
        return search(new PriorityQueue<>((v1, v2) -> (int) (
                (v1.searchState.distanceFromStart + epislon * v1.heuristicDistanceToFinish)
                        - (v2.searchState.distanceFromStart + epislon * v2.heuristicDistanceToFinish)
        )));
    }

}
