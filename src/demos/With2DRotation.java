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

    final float orientationScale = 25;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(2 * PI * orientationScale, SIDE, SIDE);
    final Vec3 startPose = Vec3.of(PI * 1f * orientationScale, SIDE * -0.9f, SIDE * -0.9f);
    final Vec3 finishPose = Vec3.of(PI * 0f * orientationScale, SIDE * 0.9f, SIDE * -0.9f);

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
        for (int i = 0; i < 10; i++) {
            sphericalObstacles.add(new SphericalObstacle(
                    this,
                    Vec3.of(0, SIDE * 0.9f - 10, SIDE * -0.9f + 10 *  i - 10),
                    SIDE * 0.05f,
                    Vec3.of(1, 0, 1)
            ));
        }
        for (int i = 0; i < 10; i++) {
            sphericalObstacles.add(new SphericalObstacle(
                    this,
                    Vec3.of(0, SIDE * -0.9f + 10, SIDE * -0.9f + 10 * i - 10),
                    SIDE * 0.05f,
                    Vec3.of(1, 0, 1)
            ));
        }
        sphericalObstacles.add(new SphericalObstacle(
                this,
                Vec3.of(0, 0, 0),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 1)
        ));
        lineSegment2DAgentDescription = new LineSegment2DAgentDescription(
                startPose,
                finishPose,
                20
        );
        configurationSpace = new LineSegment2DConfigurationSpace(this, lineSegment2DAgentDescription, sphericalObstacles, minCorner, maxCorner, orientationScale);
        lineSegmentAgent = new LineSegment2DAgent(this, lineSegment2DAgentDescription, configurationSpace, 20f, Vec3.of(1));
        Graph.END_POINT_SIZE = 3f;
        graph = new Graph(this, startPose, finishPose);
        graph.generateVertices(configurationSpace.samplePoints(20000), configurationSpace);
        graph.generateAdjacencies(10, configurationSpace);
    }

    public void draw() {
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
        if (keyCode == RIGHT) {
            lineSegmentAgent.stepForward();
        }
        if (keyCode == LEFT) {
            lineSegmentAgent.stepBackward();
        }
        if (key == 'g') {
            LineSegment2DAgent.DRAW_POSITION_ORIENTATION_SPACE_PATH = !LineSegment2DAgent.DRAW_POSITION_ORIENTATION_SPACE_PATH;
            if (LineSegment2DAgent.DRAW_POSITION_ORIENTATION_SPACE_PATH) {
                Graph.END_POINT_SIZE = 3f;
            } else {
                Graph.END_POINT_SIZE = 0f;
            }
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
