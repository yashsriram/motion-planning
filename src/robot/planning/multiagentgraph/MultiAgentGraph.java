package robot.planning.multiagentgraph;

import math.Vec3;
import processing.core.PApplet;
import robot.input.SphericalAgentDescription;
import robot.sensing.ConfigurationSpace;

import java.util.*;

public class MultiAgentGraph {
    public static boolean DRAW_VERTICES = true;
    public static boolean DRAW_EDGES = false;
    public static float END_POINT_SIZE = 2f;

    final PApplet parent;
    final List<Vertex> starts = new ArrayList<>();
    final List<Vertex> finishes = new ArrayList<>();
    final List<Vertex> vertices = new ArrayList<>();

    public MultiAgentGraph(PApplet parent, List<SphericalAgentDescription> sphericalAgentDescriptions) {
        this.parent = parent;
        for (SphericalAgentDescription description : sphericalAgentDescriptions) {
            Vertex start = Vertex.of(parent, description.startPosition, true);
            this.starts.add(start);
            this.vertices.add(start);

            Vertex finish = Vertex.of(parent, description.finishPosition, true);
            this.finishes.add(finish);
            this.vertices.add(finish);
        }
    }

    public void generateVertices(List<Vec3> newVertexPositions, ConfigurationSpace configurationSpace) {
        int numVerticesCulled = 0;
        for (Vec3 position : newVertexPositions) {
            if (configurationSpace.doesVertexIntersectSomeObstacle(position)) {
                numVerticesCulled++;
                vertices.add(Vertex.of(
                        parent,
                        position,
                        false));
            } else {
                vertices.add(Vertex.of(
                        parent,
                        position,
                        true));
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

    public void draw() {
        if (DRAW_VERTICES) {
            for (Vertex vertex : vertices) {
                vertex.draw();
            }
        }
        // Starts
        parent.fill(0, 0, 1);
        parent.noStroke();
        for (Vertex start : starts) {
            parent.pushMatrix();
            parent.translate(start.position.x, start.position.y, start.position.z);
            parent.box(END_POINT_SIZE);
            parent.popMatrix();
        }
        // Finishes
        parent.fill(0, 1, 0);
        parent.noStroke();
        for (Vertex finish : finishes) {
            parent.pushMatrix();
            parent.translate(finish.position.x, finish.position.y, finish.position.z);
            parent.box(END_POINT_SIZE);
            parent.popMatrix();
        }
    }

    private void resetSearchState(Vec3 finishPosition) {
        PApplet.println("Resetting search states of vertices");
        for (Vertex v : vertices) {
            if (v.isOutsideObstacle) {
                v.searchState.reset(finishPosition);
            }
        }
    }

    private void addToFringe(final Stack<Vertex> fringe, final Vertex current, final Vertex next) {
        fringe.add(next);
        next.searchState.addToFringeFrom(current);
    }

    public List<Vec3> dfs(int agentIndex) {
        PApplet.println("DFS");

        resetSearchState(finishes.get(agentIndex).position);
        final Stack<Vertex> fringe = new Stack<>();
        int numVerticesExplored = 0;

        // Add start to fringe

        addToFringe(fringe, starts.get(agentIndex), starts.get(agentIndex));
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.pop();
            numVerticesExplored++;
            // Check if finish
            if (current.isFinishVertex()) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return finishes.get(agentIndex).searchState.pathFromStart;
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
        return Collections.singletonList(starts.get(agentIndex).position);
    }

    private void addToFringe(final Queue<Vertex> fringe, final Vertex current, final Vertex next) {
        next.searchState.distanceFromStart = current.searchState.distanceFromStart + next.position.minus(current.position).norm();
        fringe.add(next);
        next.searchState.addToFringeFrom(current);
    }

    private List<Vec3> search(final Queue<Vertex> fringe, int agentIndex) {
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, starts.get(agentIndex), starts.get(agentIndex));
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            numVerticesExplored++;
            // Check if finish
            if (current.isFinishVertex()) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return finishes.get(agentIndex).searchState.pathFromStart;
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
        return Collections.singletonList(starts.get(agentIndex).position);
    }

    public List<Vec3> bfs(int agentIndex) {
        PApplet.println("BFS");
        resetSearchState(finishes.get(agentIndex).position);
        return search(new LinkedList<>(), agentIndex);
    }

    public List<Vec3> ucs(int agentIndex) {
        PApplet.println("UCS");
        resetSearchState(finishes.get(agentIndex).position);
        return search(new PriorityQueue<>((v1, v2) ->
                        (int) (v1.searchState.distanceFromStart - v2.searchState.distanceFromStart)),
                agentIndex);
    }

    public List<Vec3> aStar(int agentIndex) {
        PApplet.println("A*");
        resetSearchState(finishes.get(agentIndex).position);
        return search(new PriorityQueue<>((v1, v2) -> (int) (
                        (v1.searchState.distanceFromStart + v1.searchState.heuristicDistanceToFinish)
                                - (v2.searchState.distanceFromStart + v2.searchState.heuristicDistanceToFinish))),
                agentIndex);
    }

    public List<Vec3> weightedAStar(final float epislon, int agentIndex) {
        PApplet.println("Weighted A* with epsilon = " + epislon);
        resetSearchState(finishes.get(agentIndex).position);
        return search(new PriorityQueue<>((v1, v2) -> (int) (
                        (v1.searchState.distanceFromStart + epislon * v1.searchState.heuristicDistanceToFinish)
                                - (v2.searchState.distanceFromStart + epislon * v2.searchState.heuristicDistanceToFinish))),
                agentIndex);
    }

}
