package demos.boids;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import processing.core.PShape;
import robot.acting.MultiSphericalAgentSystem;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.multiagentgraph.MultiAgentGraph;
import robot.sensing.ConfigurationSpace;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class Birds extends PApplet {
    public static final int WIDTH = 1200;
    public static final int HEIGHT = 1200;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);

    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    MultiSphericalAgentSystem multiSphericalAgentSystem;
    QueasyCam cam;
    List<PShape> wings;

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
        float radiusFactor = 0.04f;
        float obstacleRadius = SIDE * radiusFactor;
        Vec3 center = Vec3.zero();
        float extent = 0.9f;
        float radii = SIDE * extent;
        float theta = 2 * PApplet.PI;
        int counter = 1;
        float divider = 50f;
        int openings = 3;
        int sideToggler = 4;
        float depth = 0;
        while (radii > SIDE * 0.2f) {
            sphericalObstacles.add(new SphericalObstacle(
                    this,
                    Vec3.of(0, center.y + radii * sin(theta), center.z + radii * cos(theta)),
                    obstacleRadius,
                    Vec3.of(1, 0, 1)
            ));

            theta += (2 * PApplet.PI) / divider;
            counter += 1;
            if (divider - counter == openings) {
                if (extent > 0.6) {
                    extent -= 0.3;
                } else {
                    extent -= 0.2;
                }
                radii = SIDE * extent;
                counter = 0;
                divider -= 4;
                depth += 0.3f;
                if (sideToggler % 2 == 0) {
                    theta = PApplet.PI;
                    sideToggler -= 1;
//                    openings -= 1 ;
                } else {
                    theta = 0f;
                    sideToggler -= 1;
                }
            }
        }
        List<SphericalAgentDescription> sphericalAgentDescriptions = new ArrayList<>();
        center = Vec3.of(0, SIDE * 0.9f, SIDE * -0.9f);
        for (int k = 0; k < 2; k++) {
            float agentRadius = SIDE * 0.020f;
            float slack = 8;
            int numAgentsRadially = 2;
            int numCircleDivisions = 8;
            Vec3 finishPosition = Vec3.of(0.0f * SIDE, 0, 0);
            for (int i = 0; i < numCircleDivisions; i++) {
                theta = 2 * PI / numCircleDivisions * i;
                for (int j = 0; j < numAgentsRadially; j++) {
                    float radialDistance = j * 2 * agentRadius + slack;
                    sphericalAgentDescriptions.add(new SphericalAgentDescription(
                            center.plus(Vec3.of(0, (float) Math.sin(theta), (float) Math.cos(theta)).scaleInPlace(radialDistance)),
                            finishPosition,
                            agentRadius
                    ));
                }
            }
            center.scaleInPlace(-1);

        }


        wings = new ArrayList<>();
        PShape model = loadShape("data/3d-model.obj");
        model.rotateX(PApplet.PI);
        model.rotateY(PApplet.PI);
        model.scale(0.01f);
        wings.add(model);

        ConfigurationSpace configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescriptions.get(0), sphericalObstacles);
        multiSphericalAgentSystem = new MultiSphericalAgentSystem(this, sphericalAgentDescriptions, configurationSpace, minCorner, maxCorner, 2);
        MultiSphericalAgentSystem.INITIAL_AGENT_SPEED = 10f;

        MultiAgentGraph.DRAW_ENDS = false;
        MultiAgentGraph.DRAW_VERTICES = false;

        // tuning parameters
        SphericalAgent.IMPACT_RADIUS = 10f;
        SphericalAgent.SEPERATION_FORCE_BOID = 3f;
        SphericalAgent.SEPERATION_FORCE_OBSTACLE = 5f;
        SphericalAgent.ALIGNMENT_FORCE = 0.02f;
        SphericalAgent.CENTROID_FORCE = 0.05f;
        SphericalAgent.DRAW_FUTURE_STATE = false;
        SphericalAgent.DRAW_PATH = false;
    }

    public void draw() {
        long start = millis();
        // update
        multiSphericalAgentSystem.updateBoid(sphericalObstacles, 0.1f);
        // multiagent system
        for (int i = 0; i < multiSphericalAgentSystem.sphericalAgents.size(); i++) {
            SphericalAgent agent = multiSphericalAgentSystem.sphericalAgents.get(i);
            agent.draw();
            if (agent.hasReachedEnd()) {
                agent.reset();
//                agent.setPath(multiAgentGraph.aStar(i));
            }
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

        multiSphericalAgentSystem.drawBox();
//        multiSphericalAgentSystem.draw(wings);
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " search: " + SEARCH_ALGORITHM);
    }

    public void keyPressed() {
        if (keyCode == RIGHT) {
            multiSphericalAgentSystem.stepForward();
        }
        if (keyCode == LEFT) {
            multiSphericalAgentSystem.stepBackward();
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
            multiSphericalAgentSystem.togglePause();
        }
        if (key == 'b') {
            SphericalAgent.DRAW_FUTURE_STATE = !SphericalAgent.DRAW_FUTURE_STATE;
        }
        if (key == 'v') {
            SphericalAgent.DRAW_PATH = !SphericalAgent.DRAW_PATH;
        }

        if (key == '4') {
            multiSphericalAgentSystem.aStar();
            SEARCH_ALGORITHM = "A*";
        }

    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.boids.Birds"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }

}
