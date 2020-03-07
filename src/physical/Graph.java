package physical;

import math.Vec3;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Graph {
    final PApplet parent;
    final Vertex start;
    final Vertex finish;
    final List<Vertex> vertices = new ArrayList<>();

    public Graph(PApplet parent, Vertex start, Vertex finish) {
        this.parent = parent;
        this.start = start;
        this.finish = finish;
        this.vertices.add(start);
        this.vertices.add(finish);
    }

    public void addVertex(Vertex vertex) {
        this.vertices.add(vertex);
    }

    public void cullInObstacleVertices(SphericalObstacle sphericalObstacle, SphericalAgent sphericalAgent) {
        List<Integer> badVertexIndices = new ArrayList<>();
        for (int i = 0; i < vertices.size(); ++i) {
            Vertex vertex = vertices.get(i);
            if (vertex.position.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                vertex.color = Vec3.of(1, 0, 1);
                badVertexIndices.add(i);
            }
        }
        for (int i = badVertexIndices.size() - 1; i >= 0; --i) {
            int indexToRemove = badVertexIndices.get(i);
            vertices.remove(indexToRemove);
        }
    }

    public void generateEdges() {
        for (int i = 0; i < vertices.size(); ++i) {
            for (int j = 0; j < vertices.size(); j++) {
                Vertex v1 = vertices.get(i);
                Vertex v2 = vertices.get(j);
                if (v1.position.minus(v2.position).norm() < 30) {
                    v1.addNeighbour(v2, Vec3.of(1));
                    v2.addNeighbour(v1, Vec3.of(1));
                }
            }
        }
    }

    public void cullIntersectingEdges(SphericalObstacle sphericalObstacle, SphericalAgent sphericalAgent) {
        for (int i = 0; i < vertices.size(); ++i) {
            Vertex v = vertices.get(i);
            for (int j = 0; j < v.neighbours.size(); j++) {
                Vertex n = v.neighbours.get(j);
                Vec3 pb_pa = n.position.minus(v.position);
                Vec3 pa_pc = v.position.minus(sphericalObstacle.center);
                float r = sphericalObstacle.radius + sphericalAgent.radius;
                float a = pb_pa.dot(pb_pa);
                float c = pa_pc.dot(pa_pc) - r * r;
                float b = 2 * pb_pa.dot(pa_pc);
                float discriminant = b * b - 4 * a * c;
                if (discriminant >= 0) {
                    float t1 = (float) ((-b + Math.sqrt(discriminant)) / (2 * a));
                    float t2 = (float) ((-b - Math.sqrt(discriminant)) / (2 * a));
                    if ((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1)) {
                        v.edgeColors.set(j, Vec3.of(1, 0, 1));
                    }
                }
            }
        }
    }

    public void draw() {
        for (Vertex vertex : vertices) {
            vertex.draw();
        }
    }
}
