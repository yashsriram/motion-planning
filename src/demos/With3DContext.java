package demos;

import camera.QueasyCam;
import math.Vec3;
import physical.Ground;
import physical.SphericalAgent;
import physical.SphericalAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;
import tools.Graph;
import tools.Vertex;
import tools.configurationspace.BSHConfigurationSpace;
import tools.configurationspace.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class With3DContext extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 OFFSET = Vec3.of(100, 100, 0);
    final Vec3 startPosition = Vec3.of(SIDE * (-9f / 10), 0, SIDE * (-9f / 10)).plusInPlace(OFFSET);
    final Vec3 finishPosition = Vec3.of(SIDE * (9f / 10), 0, SIDE * (9f / 10)).plusInPlace(OFFSET);
    SphericalAgentDescription sphericalAgentDescription;
    Ground ground;
    SphericalAgent sphericalAgent;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    PlainConfigurationSpace configurationSpace;
    Graph graph;

    QueasyCam cam;

    static boolean DRAW_OBSTACLES = true;
    static String SEARCH_ALGORITHM = "";
    static boolean SMOOTH_PATH = false;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
        sphericalObstacles.add(new SphericalObstacle(
                this,
                Vec3.of(OFFSET),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 0)
        ));
        sphericalAgentDescription = new SphericalAgentDescription(
                startPosition,
                SIDE * (5f / 20)
        );
        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles);
        sphericalAgent = new SphericalAgent(
                this,
                sphericalAgentDescription,
                configurationSpace,
                20f,
                Vec3.of(0.5f)
        );
        ground = new Ground(this,
                OFFSET.plus(Vec3.of(0, sphericalAgentDescription.radius, 0)),
                Vec3.of(0, 0, 1), Vec3.of(1, 0, 0),
                2 * SIDE, 2 * SIDE,
                loadImage("ground3.jpg"));

        // vertex sampling
        List<Vec3> vertexPositions = new ArrayList<>();
        for (int i = 0; i < 10000; ++i) {
            vertexPositions.add(Vec3.of(random(-SIDE, SIDE), 0, random(-SIDE, SIDE)).plusInPlace(OFFSET));
        }
        Graph.END_POINT_SIZE = 5f;
        graph = new Graph(this, startPosition, finishPosition);
        graph.generateVertices(vertexPositions, configurationSpace);
        graph.generateAdjacencies(10, configurationSpace);
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
        if (SMOOTH_PATH) {
            sphericalAgent.smoothUpdate(0.1f);
        } else {
            sphericalAgent.update(0.1f);
        }
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
        sphericalAgent.drawSprite();
        // ground
        ground.draw();
        // configuration space
        configurationSpace.draw();
        // graph
        graph.draw();
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " search: " + SEARCH_ALGORITHM + " smooth-path: " + SMOOTH_PATH);
    }

    public void keyPressed() {
        if (key == 'x') {
            SMOOTH_PATH = !SMOOTH_PATH;
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
            Vertex.DRAW_EDGES = !Vertex.DRAW_EDGES;
        }
        if (key == 'p') {
            sphericalAgent.isPaused = !sphericalAgent.isPaused;
        }
        if (key == '1') {
            sphericalAgent.setPath(graph.dfs());
            SEARCH_ALGORITHM = "DFS";
        }
        if (key == '2') {
            sphericalAgent.setPath(graph.bfs());
            SEARCH_ALGORITHM = "BFS";
        }
        if (key == '3') {
            sphericalAgent.setPath(graph.ucs());
            SEARCH_ALGORITHM = "UCS";
        }
        if (key == '4') {
            sphericalAgent.setPath(graph.aStar());
            SEARCH_ALGORITHM = "A*";
        }
        if (key == '5') {
            sphericalAgent.setPath(graph.weightedAStar(1.5f));
            SEARCH_ALGORITHM = "weighted A*";
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.With3DContext"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
