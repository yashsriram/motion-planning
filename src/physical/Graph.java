package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class Graph {
    final PApplet parent;
    final Vertex start;
    final Vertex finish;
    final List<Vertex> vertices = new ArrayList<>();

    public Graph(PApplet parent, Vec3 startPosition, Vec3 finishPosition) {
        this.parent = parent;
        this.start = Vertex.start(parent, startPosition, Vec3.of(0, 1, 0));
        this.finish = Vertex.finish(parent, finishPosition, Vec3.of(0, 0, 1));
        this.vertices.add(start);
        this.vertices.add(finish);
    }

    public void addVertex(Vertex vertex) {
        this.vertices.add(vertex);
    }

    public void cullInObstacleVertices(SphericalAgent sphericalAgent, SphericalObstacle sphericalObstacle) {
        PApplet.println("# vertices before culling: " + vertices.size());
        List<Integer> badVertexIndices = new ArrayList<>();
        for (int i = 0; i < vertices.size(); ++i) {
            Vertex vertex = vertices.get(i);
            if (vertex.position.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                vertex.color = Vec3.of(1, 0, 1);
                vertex.isDead = true;
                badVertexIndices.add(i);
            }
        }
//        for (int i = badVertexIndices.size() - 1; i >= 0; --i) {
//            int indexToRemove = badVertexIndices.get(i);
//            vertices.remove(indexToRemove);
//        }
        PApplet.println("# vertices culled: " + badVertexIndices.size());
        PApplet.println("# vertices after culling: " + vertices.size());
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
            if ((0 <= t1 && t1 <= 1)
                    || (0 <= t2 && t2 <= 1)) {
                return true;
            }
        }
        return false;
    }

    public void generateEdges(float maxEdgeLen, SphericalAgent sphericalAgent, SphericalObstacle sphericalObstacle) {
        int numEdges = 0;
        int numEdgesCulled = 0;
        for (int i = 0; i < vertices.size(); ++i) {
            for (int j = i + 1; j < vertices.size(); j++) {
                Vertex v1 = vertices.get(i);
                Vertex v2 = vertices.get(j);
                if (v1.position.minus(v2.position).norm() < maxEdgeLen) {

                    // Check for intersection with spherical obstacle
                    if (v1.isDead || v2.isDead || doesIntersect(v1, v2, sphericalAgent, sphericalObstacle)) {
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

    public void bfs() {
        Queue<Vertex> fringe = new LinkedList<>();
        int numVerticesExplored = 0;

        // Add start to fringe
        fringe.add(start);
        start.isOnFringe = true;
        start.color = Vec3.of(0, 1, 0);

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
                if (!neighbour.isOnFringe && !neighbour.isDead) {
                    fringe.add(neighbour);
                    neighbour.color = Vec3.of(0, 1, 0);
                    neighbour.isOnFringe = true;
                }
            }
        }

        PApplet.println("Could not reach finish, # vertices explored: " + numVerticesExplored);
    }

}
