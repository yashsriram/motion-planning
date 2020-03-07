import camera.QueasyCam;
import math.Vec3;
import physical.Graph;
import physical.SphericalAgent;
import physical.SphericalObstacle;
import physical.Vertex;
import processing.core.PApplet;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 start = Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finish = Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10));
    SphericalAgent sphericalAgent;
    SphericalObstacle sphericalObstacle;
    Graph graph;

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
        graph = new Graph(this,
                Vertex.start(
                        this,
                        start,
                        Vec3.of(0, 1, 0)
                ),
                Vertex.finish(
                        this,
                        finish,
                        Vec3.of(0, 0, 1)
                )
        );
        // vertex sampling
        for (int i = 0; i < 1000; ++i) {
            graph.addVertex(Vertex.of(
                    this,
                    Vec3.of(0, random(-SIDE, SIDE), random(-SIDE, SIDE)),
                    Vec3.of(1, 1, 0)
            ));
        }
        graph.cullInObstacleVertices(sphericalObstacle, sphericalAgent);
        graph.generateEdges();
        graph.cullIntersectingEdges(sphericalObstacle, sphericalAgent);
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
        // graph
        graph.draw();
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms");
    }

    public void keyPressed() {
        if (key == 'h') {
            sphericalObstacle.isDrawn = !sphericalObstacle.isDrawn;
        }
        if (key == 'j') {
            Vertex.DRAW_EDGES = !Vertex.DRAW_EDGES;
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
