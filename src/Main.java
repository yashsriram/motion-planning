import camera.QueasyCam;
import math.Vec3;
import physical.SphericalAgent;
import physical.SphericalObstacle;
import physical.Vertex;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 start = Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finish = Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10));
    SphericalAgent sphericalAgent;
    SphericalObstacle sphericalObstacle;
    final List<Vertex> vertices = new ArrayList<>();

    QueasyCam cam;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
        sphericalObstacle = new SphericalObstacle(
                this,
                Vec3.of(0, 0, 0),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 0)
        );
        sphericalAgent = new SphericalAgent(
                this,
                start,
                SIDE * (0.5f / 20),
                Vec3.of(1)
        );

        // vertex sampling
        vertices.add(Vertex.start(
                this,
                start,
                Vec3.of(0, 1, 0)
        ));
        vertices.add(Vertex.finish(
                this,
                finish,
                Vec3.of(0, 0, 1)
        ));
        for (int i = 0; i < 1000; ++i) {
            vertices.add(Vertex.of(
                    this,
                    Vec3.of(0, random(-SIDE, SIDE), random(-SIDE, SIDE)),
                    Vec3.of(1, 1, 0)
            ));
        }

        // vertex culling
        List<Integer> badVertexIndices = new ArrayList<>();
        for (int i = 0; i < vertices.size(); ++i) {
            Vertex vertex = vertices.get(i);
            if (vertex.position.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                vertex.color = Vec3.of(1, 0, 1);
                badVertexIndices.add(i);
            }
        }
//        for (int i = badVertexIndices.size() - 1; i >= 0; --i) {
//            int indexToRemove = badVertexIndices.get(i);
//            vertices.remove(indexToRemove);
//        }

        // edge creation
        for (int i = 0; i < vertices.size(); ++i) {
            for (int j = 0; j < vertices.size(); j++) {
                Vertex v1 = vertices.get(i);
                Vertex v2 = vertices.get(j);
                if (v1.position.minus(v2.position).norm() < 20) {
                    v1.addNeighbour(v2, Vec3.of(1));
                    v2.addNeighbour(v1, Vec3.of(1));
                }
            }
        }

        // edge culling
        for (int i = 0; i < vertices.size(); ++i) {
            Vertex v = vertices.get(i);
            for (int j = 0; j < v.neighbours.size(); j++) {
                Vertex n = v.neighbours.get(j);
                Vec3 pb_pa = n.position.minus(v.position);
                Vec3 pa_pc = v.position.minus(sphericalObstacle.center);
                float r = sphericalObstacle.radius;
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
        long start = millis();
        // update
        long update = millis();
        // draw
        background(0);
        // agent
        sphericalAgent.draw();
        // obstacle
        sphericalObstacle.draw();
        // vertices
        for (Vertex vertex : vertices) {
            vertex.draw();
        }
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms");
    }

    public void keyPressed() {
        if (key == 'o') {
            sphericalObstacle.isDrawn = !sphericalObstacle.isDrawn;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"Main"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
