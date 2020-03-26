package demos;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.multiagentgraph.MultiAgentGraph;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class MultiAgentSmoothPath extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);

    SphericalAgent sphericalAgent;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    PlainConfigurationSpace configurationSpace;
    MultiAgentGraph multiAgentGraph;

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
                Vec3.of(0, 0, 0),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 1)
        ));
        List<SphericalAgentDescription> sphericalAgentDescriptions = new ArrayList<>();
        sphericalAgentDescriptions.add(new SphericalAgentDescription(
                Vec3.of(0, SIDE * 0.9f, SIDE * -0.9f),
                Vec3.of(0, SIDE * -0.9f, SIDE * 0.9f),
                SIDE * (0.5f / 20)
        ));
        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescriptions.get(0), sphericalObstacles);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescriptions.get(0), configurationSpace, minCorner, maxCorner, 20f, Vec3.of(1));
        multiAgentGraph = new MultiAgentGraph(this, sphericalAgentDescriptions);
        multiAgentGraph.generateVertices(sphericalAgent.samplePoints(10000), configurationSpace);
        multiAgentGraph.generateAdjacencies(10, configurationSpace);
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
        sphericalAgent.draw();
        // configuration space
        configurationSpace.draw();
        // graph
        multiAgentGraph.draw();
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " search: " + SEARCH_ALGORITHM + " smooth-path: " + SMOOTH_PATH);
    }

    public void keyPressed() {
        if (key == 'x') {
            SMOOTH_PATH = !SMOOTH_PATH;
        }
        if (key == 'h') {
            DRAW_OBSTACLES = !DRAW_OBSTACLES;
        }
        if (key == 'k') {
            MultiAgentGraph.DRAW_VERTICES = !MultiAgentGraph.DRAW_VERTICES;
        }
        if (key == 'j') {
            MultiAgentGraph.DRAW_EDGES = !MultiAgentGraph.DRAW_EDGES;
        }
        if (key == 'p') {
            sphericalAgent.isPaused = !sphericalAgent.isPaused;
        }
        if (key == '1') {
            sphericalAgent.setPath(multiAgentGraph.dfs(0));
            SEARCH_ALGORITHM = "DFS";
        }
        if (key == '2') {
            sphericalAgent.setPath(multiAgentGraph.bfs(0));
            SEARCH_ALGORITHM = "BFS";
        }
        if (key == '3') {
            sphericalAgent.setPath(multiAgentGraph.ucs(0));
            SEARCH_ALGORITHM = "UCS";
        }
        if (key == '4') {
            sphericalAgent.setPath(multiAgentGraph.aStar(0));
            SEARCH_ALGORITHM = "A*";
        }
        if (key == '5') {
            float weight = 1.5f;
            sphericalAgent.setPath(multiAgentGraph.weightedAStar(weight, 0));
            SEARCH_ALGORITHM = weight + "A*";
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.MultiAgentSmoothPath"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
