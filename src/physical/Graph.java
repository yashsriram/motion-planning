package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.*;

public class Graph {
    final PApplet parent;
    final Vertex start;
    final Vertex finish;
    final List<Vertex> vertices = new ArrayList<>();

    public Graph(PApplet parent, Vec3 startPosition, Vec3 finishPosition) {
        this.parent = parent;
        this.start = Vertex.start(parent, startPosition, finishPosition.minus(startPosition).norm(), Vec3.of(0, 1, 0));
        this.finish = Vertex.finish(parent, finishPosition, 0, Vec3.of(0, 0, 1));
        this.vertices.add(start);
        this.vertices.add(finish);
    }

    public void generateVertices(List<Vec3> newVertexPositions, SphericalAgent sphericalAgent, SphericalObstacle sphericalObstacle) {
        for (Vec3 position : newVertexPositions) {
            float distanceToFinish = finish.position.minus(position).norm();
            vertices.add(Vertex.of(
                    parent,
                    position,
                    distanceToFinish,
                    Vec3.of(1)));
        }
        PApplet.println("# vertices before culling: " + vertices.size());
        int numVerticesCulled = 0;
        for (Vertex vertex : vertices) {
            if (vertex.position.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                vertex.color = Vec3.of(1, 0, 1);
                vertex.canBeReached = false;
                numVerticesCulled++;
            }
        }
        PApplet.println("# vertices culled: " + numVerticesCulled);
        PApplet.println("# vertices after culling: " + (vertices.size() - numVerticesCulled));
    }

    private boolean doesIntersect(Vertex v1, Vertex v2, SphericalAgent sphericalAgent, SphericalObstacle sphericalObstacle) {
        Vec3 pb_pa = v2.position.minus(v1.position);
        Vec3 pa_pc = v1.position.minus(sphericalObstacle.center);
        float r = sphericalObstacle.radius + sphericalAgent.radius;
        float a = pb_pa.dot(pb_pa);
        float c = pa_pc.dot(pa_pc) - r * r;
        float b = 2 * pb_pa.dot(pa_pc);
        float discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            float t1 = (float) ((-b + Math.sqrt(discriminant)) / (2 * a));
            float t2 = (float) ((-b - Math.sqrt(discriminant)) / (2 * a));
            // Intersection with line segment only possible iff at least one of the solutions lies in [0, 1]
            return (0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1);
        }
        return false;
    }

    public void generateAdjacencies(float maxEdgeLen, SphericalAgent sphericalAgent, SphericalObstacle sphericalObstacle) {
        int numEdges = 0;
        int numEdgesCulled = 0;
        for (int i = 0; i < vertices.size(); ++i) {
            for (int j = i + 1; j < vertices.size(); j++) {
                Vertex v1 = vertices.get(i);
                Vertex v2 = vertices.get(j);
                if (v1.position.minus(v2.position).norm() <= maxEdgeLen) {
                    // Check for intersection with spherical obstacle
                    if (!v1.canBeReached || !v2.canBeReached || doesIntersect(v1, v2, sphericalAgent, sphericalObstacle)) {
                        v1.addNeighbour(v2, Vec3.of(1, 0, 1));
                        v2.addNeighbour(v1, Vec3.of(1, 0, 1));
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
        for (Vertex vertex : vertices) {
            vertex.draw();
        }
    }

    public void reset() {
        PApplet.println("Reset");
        for (Vertex v : vertices) {
            if (v.canBeReached) {
                v.distanceFromStart = 0;
                v.isExplored = false;
                v.color = Vec3.of(1);
            }
        }
    }

    private void addToFringe(final Stack<Vertex> fringe, final Vertex v) {
        fringe.add(v);
        v.isExplored = true;
        v.color = Vec3.of(0, 1, 0);
    }

    public void dfs() {
        PApplet.println("-- DFS --");

        final Stack<Vertex> fringe = new Stack<>();
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.pop();
            current.color = Vec3.of(1, 0, 0);
            // PApplet.println(current.id);
            numVerticesExplored++;
            // Check if finish
            if (current.id == Vertex.FINISH_ID) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return;
            }
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.canBeReached && !neighbour.isExplored) {
                    addToFringe(fringe, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

    private void addToFringe(final Queue<Vertex> fringe, final Vertex v) {
        fringe.add(v);
        v.isExplored = true;
        v.color = Vec3.of(0, 1, 0);
    }

    public void bfs() {
        PApplet.println("-- BFS --");

        final Queue<Vertex> fringe = new LinkedList<>();
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            current.color = Vec3.of(1, 0, 0);
            // PApplet.println(current.id);
            numVerticesExplored++;
            // Check if finish
            if (current.id == Vertex.FINISH_ID) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return;
            }
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.canBeReached && !neighbour.isExplored) {
                    addToFringe(fringe, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

    private void addToFringe(final Queue<Vertex> fringe, final Vertex current, final Vertex next) {
        next.distanceFromStart = current.distanceFromStart + next.position.minus(current.position).norm();
        fringe.add(next);
        next.isExplored = true;
        next.color = Vec3.of(0, 1, 0);
    }

    public void ucs() {
        PApplet.println("-- UCS --");

        final Queue<Vertex> fringe = new PriorityQueue<>((v1, v2) -> (int) (v1.distanceFromStart - v2.distanceFromStart));
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            current.color = Vec3.of(1, 0, 0);
            // PApplet.println(current.id);
            numVerticesExplored++;
            // Check if finish
            if (current.id == Vertex.FINISH_ID) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return;
            }
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.canBeReached && !neighbour.isExplored) {
                    addToFringe(fringe, current, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

    public void aStar() {
        PApplet.println("-- A* --");

        final Queue<Vertex> fringe = new PriorityQueue<>((v1, v2) -> (int) (
                (v1.distanceFromStart + v1.heuristicDistanceToFinish)
                        - (v2.distanceFromStart + v2.heuristicDistanceToFinish)
        ));
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            current.color = Vec3.of(1, 0, 0);
            // PApplet.println(current.id);
            numVerticesExplored++;
            // Check if finish
            if (current.id == Vertex.FINISH_ID) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return;
            }
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.canBeReached && !neighbour.isExplored) {
                    addToFringe(fringe, current, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

    public void weightedAStar(final float epislon) {
        PApplet.println("-- Weighted A* with epsilon = " + epislon + " --");

        final Queue<Vertex> fringe = new PriorityQueue<>((v1, v2) -> (int) (
                (v1.distanceFromStart + epislon * v1.heuristicDistanceToFinish)
                        - (v2.distanceFromStart + epislon * v2.heuristicDistanceToFinish)
        ));
        int numVerticesExplored = 0;

        // Add start to fringe
        addToFringe(fringe, start, start);
        while (fringe.size() > 0) {
            // Pop one vertex
            Vertex current = fringe.remove();
            current.color = Vec3.of(1, 0, 0);
            // PApplet.println(current.id);
            numVerticesExplored++;
            // Check if finish
            if (current.id == Vertex.FINISH_ID) {
                PApplet.println("Reached finish, # vertices explored: " + numVerticesExplored);
                return;
            }
            // Update fringe
            for (Vertex neighbour : current.neighbours) {
                if (neighbour.canBeReached && !neighbour.isExplored) {
                    addToFringe(fringe, current, neighbour);
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

}
