package demos;

import camera.QueasyCam;
import math.Vec3;
import physical.SphericalAgent;
import physical.SphericalAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;
import tools.configurationspace.BSHConfigurationSpace;
import tools.configurationspace.ConfigurationSpace;
import tools.configurationspace.PlainConfigurationSpace;
import tools.graph.Graph;

import java.util.ArrayList;
import java.util.List;

public class BSHSpeedUp extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 startPosition = Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finishPosition = Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10));
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);

    SphericalAgentDescription sphericalAgentDescription;
    SphericalAgent sphericalAgent;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    ConfigurationSpace configurationSpace;
    Graph graph;

    QueasyCam cam;

    static boolean DRAW_OBSTACLES = true;
    static String DATA_STRUCTURE = "";
    static long DATA_STRUCTURE_CREATION_TIME = 0;
    static long VERTEX_SAMPLING_TIME = 0;
    static long VERTEX_CULLING_TIME = 0;
    static long EDGE_CULLING_TIME = 0;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
        int numObstacles = 5000;
        for (int i = 0; i < numObstacles; i++) {
            sphericalObstacles.add(new SphericalObstacle(
                    this,
                    Vec3.of(0, -SIDE + i * SIDE * random(1.7f) / numObstacles, -SIDE + i * SIDE * random(1f) / numObstacles),
                    SIDE * (0.1f / 20),
                    Vec3.of(1, 0, 0)
            ));
        }

        sphericalAgentDescription = new SphericalAgentDescription(
                startPosition,
                SIDE * (0.5f / 20)
        );
        resetBSH();
    }

    private void resetPlain() {
        DATA_STRUCTURE = "Plain";
        long startConfig = millis();
        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles, minCorner, maxCorner);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, 20f, Vec3.of(1));
        long configSpace = millis();
        long start = millis();
        graph = new Graph(this, startPosition, finishPosition);
        long sampling = millis();
        graph.generateVertices(configurationSpace.samplePoints(10000), configurationSpace);
        long vertex = millis();
        graph.generateAdjacencies(20, configurationSpace);
        long edge = millis();
        DATA_STRUCTURE_CREATION_TIME = configSpace - startConfig;
        VERTEX_SAMPLING_TIME = sampling - start;
        VERTEX_CULLING_TIME = vertex - sampling;
        EDGE_CULLING_TIME = edge - vertex;
    }

    private void resetBSH() {
        DATA_STRUCTURE = "BSH";
        long startConfig = millis();
        configurationSpace = new BSHConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles, minCorner, maxCorner);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, 20f, Vec3.of(1));
        long configSpace = millis();
        long start = millis();
        graph = new Graph(this, startPosition, finishPosition);
        long sampling = millis();
        graph.generateVertices(configurationSpace.samplePoints(10000), configurationSpace);
        long vertex = millis();
        graph.generateAdjacencies(20, configurationSpace);
        long edge = millis();
        DATA_STRUCTURE_CREATION_TIME = configSpace - startConfig;
        VERTEX_SAMPLING_TIME = sampling - start;
        VERTEX_CULLING_TIME = vertex - sampling;
        EDGE_CULLING_TIME = edge - vertex;
    }

    public void draw() {
        if (keyPressed) {
            if (keyCode == RIGHT) {
                sphericalAgent.stepForward();
            }
            if (keyCode == LEFT) {
                sphericalAgent.stepBackward();
            }
        }
        long start = millis();
        // update
        sphericalAgent.update(0.1f);
        long update = millis();
        // draw
        background(0);
        // obstacles
        if (DRAW_OBSTACLES) {
            for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
                sphericalObstacle.draw();
            }
        }
        // agent
        sphericalAgent.draw();
        // configuration space
        configurationSpace.draw();
        // graph
        graph.draw();
        long draw = millis();

        surface.setTitle("DS: " + DATA_STRUCTURE
                + " DS creation : " + DATA_STRUCTURE_CREATION_TIME + "ms "
                + " V culling : " + VERTEX_CULLING_TIME + "ms "
                + " E culling: " + EDGE_CULLING_TIME + "ms"
        );
    }

    public void keyPressed() {
        if (key == 'b') {
            resetBSH();
        }
        if (key == 'v') {
            resetPlain();
        }
        if (key == 'g') {
            BSHConfigurationSpace.DRAW_BOUNDING_SPHERES = !BSHConfigurationSpace.DRAW_BOUNDING_SPHERES;
        }
        if (key == 'h') {
            DRAW_OBSTACLES = !DRAW_OBSTACLES;
        }
        if (key == 'k') {
            Graph.DRAW_VERTICES = !Graph.DRAW_VERTICES;
        }
        if (key == 'j') {
            Graph.DRAW_EDGES = !Graph.DRAW_EDGES;
        }
        if (key == 'p') {
            sphericalAgent.isPaused = !sphericalAgent.isPaused;
        }
        if (key == '1') {
            sphericalAgent.setPath(graph.dfs());
        }
        if (key == '2') {
            sphericalAgent.setPath(graph.bfs());
        }
        if (key == '3') {
            sphericalAgent.setPath(graph.ucs());
        }
        if (key == '4') {
            sphericalAgent.setPath(graph.aStar());
        }
        if (key == '5') {
            sphericalAgent.setPath(graph.weightedAStar(1.5f));
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.BSHSpeedUp"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
