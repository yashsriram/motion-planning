package demos;

import camera.QueasyCam;
import math.Vec3;
import physical.LineSegment2DAgent;
import physical.LineSegment2DAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;
import tools.configurationspace.LineSegment2DConfigurationSpace;
import tools.graph.Graph;

import java.util.ArrayList;
import java.util.List;

public class With2DRotation extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(2 * PI, SIDE, SIDE);

    final Vec3 startPose = Vec3.of(PI / 4, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finishPose = Vec3.of(PI / 2, SIDE * (-9f / 10), SIDE * (9f / 10));
    LineSegment2DAgentDescription lineSegment2DAgentDescription;
    LineSegment2DAgent lineSegmentAgent;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    LineSegment2DConfigurationSpace configurationSpace;
    Graph graph;

    QueasyCam cam;

    static boolean DRAW_OBSTACLES = true;
    static String SEARCH_ALGORITHM = "";

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
                Vec3.of(1, 0, 0)
        ));
        lineSegment2DAgentDescription = new LineSegment2DAgentDescription(
                startPose,
                20
        );
        configurationSpace = new LineSegment2DConfigurationSpace(this, lineSegment2DAgentDescription, sphericalObstacles, minCorner, maxCorner);
        lineSegmentAgent = new LineSegment2DAgent(this, lineSegment2DAgentDescription, configurationSpace, 20f, Vec3.of(1));
        graph = new Graph(this, startPose, finishPose);
        graph.generateVertices(configurationSpace.samplePoints(10000), configurationSpace);
        graph.generateAdjacencies(10, configurationSpace);
    }

    public void draw() {
        if (keyPressed) {
            if (keyCode == RIGHT) {
                lineSegmentAgent.stepForward();
            }
            if (keyCode == LEFT) {
                lineSegmentAgent.stepBackward();
            }
        }
        long start = millis();
        // update
        lineSegmentAgent.update(0.1f);
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
        lineSegmentAgent.draw();
        // configuration space
        configurationSpace.draw();
        // graph
        graph.draw();
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " search: " + SEARCH_ALGORITHM);
    }

    public void keyPressed() {
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
            lineSegmentAgent.isPaused = !lineSegmentAgent.isPaused;
        }
        if (key == '1') {
            lineSegmentAgent.setPath(graph.dfs());
            SEARCH_ALGORITHM = "DFS";
        }
        if (key == '2') {
            lineSegmentAgent.setPath(graph.bfs());
            SEARCH_ALGORITHM = "BFS";
        }
        if (key == '3') {
            lineSegmentAgent.setPath(graph.ucs());
            SEARCH_ALGORITHM = "UCS";
        }
        if (key == '4') {
            lineSegmentAgent.setPath(graph.aStar());
            SEARCH_ALGORITHM = "A*";
        }
        if (key == '5') {
            lineSegmentAgent.setPath(graph.weightedAStar(1.5f));
            SEARCH_ALGORITHM = "weighted A*";
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.With2DRotation"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
