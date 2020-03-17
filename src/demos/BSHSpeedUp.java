package demos;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.graph.Graph;
import robot.sensing.BSHConfigurationSpace;
import robot.sensing.ConfigurationSpace;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class BSHSpeedUp extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 startPosition = Vec3.of(0, SIDE * 0.9f, SIDE * -0.9f);
    Vec3 finishPosition = Vec3.of(0, SIDE * -0.9f, SIDE * 0.9f);
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
        int numObstacles = 2500;
        int numBlobs = 2;
        float obstacleRadius = SIDE * 0.01f;
        for (int j = 0; j < numBlobs; j++) {
            float offsetY = -SIDE + 2 * SIDE * (j + 0.5f) / numBlobs;
            float offsetZ = -offsetY;
            for (int i = 0; i < numObstacles; i++) {
                float r = (float) Math.sqrt(random(1)) * 10;
                float theta = random(2 * PI);
                sphericalObstacles.add(new SphericalObstacle(
                        this,
                        Vec3.of(0, offsetY + (float) (r * Math.sin(theta)), offsetZ + (float) (r * Math.cos(theta))),
                        obstacleRadius,
                        Vec3.of(1, 0, 1)
                ));
            }
        }
        sphericalAgentDescription = new SphericalAgentDescription(
                startPosition,
                finishPosition,
                SIDE * 0.025f
        );

        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles);
        graph = new Graph(this, startPosition, finishPosition);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, minCorner, maxCorner, 20f, Vec3.of(1));
        graph.generateVertices(sphericalAgent.samplePoints(7000), configurationSpace);

        resetBSH();
        sphericalAgent.setPath(graph.weightedAStar(1.5f));
    }

    private void resetPlain() {
        DATA_STRUCTURE = "Plain";
        long startConfig = millis();
        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, minCorner, maxCorner, 20f, Vec3.of(1));
        long configSpace = millis();
        graph.clearAdjacencies();
        graph.generateAdjacencies(10, configurationSpace);
        long edge = millis();
        DATA_STRUCTURE_CREATION_TIME = configSpace - startConfig;
        EDGE_CULLING_TIME = edge - configSpace;
    }

    private void resetBSH() {
        DATA_STRUCTURE = "BSH";
        long startConfig = millis();
        configurationSpace = new BSHConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, minCorner, maxCorner, 20f, Vec3.of(1));
        long configSpace = millis();
        graph.clearAdjacencies();
        graph.generateAdjacencies(10, configurationSpace);
        long edge = millis();
        DATA_STRUCTURE_CREATION_TIME = configSpace - startConfig;
        EDGE_CULLING_TIME = edge - configSpace;
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
                sphericalObstacle.drawBox();
            }
        }
        // agent
        sphericalAgent.draw();
        // configuration space
        configurationSpace.draw();
        // graph
        graph.draw();
        long draw = millis();

        surface.setTitle(
                "FPS: " + Math.round(frameRate)
                        + " DS: " + DATA_STRUCTURE
                        + " DS creation : " + DATA_STRUCTURE_CREATION_TIME + "ms "
                        + " E culling: " + EDGE_CULLING_TIME + "ms"
        );
    }

    public void keyPressed() {
        if (key == 'b') {
            resetBSH();
            sphericalAgent.setPath(graph.weightedAStar(1.5f));
        }
        if (key == 'v') {
            resetPlain();
            sphericalAgent.setPath(graph.weightedAStar(1.5f));
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
