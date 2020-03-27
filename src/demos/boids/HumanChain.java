package demos.boids;

import camera.QueasyCam;
import fixed.Ground;
import fixed.SphericalObstacle;
import fixed.SpriteSphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PShape;
import robot.acting.MultiSphericalAgentSystem;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.multiagentgraph.MultiAgentGraph;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class HumanChain extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 1000;

    final Vec3 OFFSET = Vec3.of(100, 100, 0);
    final Vec3 finishPosition = Vec3.of(SIDE * 0.9f, 0, SIDE * 0.9f).plusInPlace(OFFSET);
    final Vec3 minCorner = Vec3.of(-SIDE, 0, -SIDE).plusInPlace(OFFSET);
    final Vec3 maxCorner = Vec3.of(SIDE, 0, SIDE).plusInPlace(OFFSET);

    MultiSphericalAgentSystem multiSphericalAgentSystem;
    List<SphericalAgentDescription> sphericalAgentDescriptions;
    Ground ground;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    List<PShape> agentWalkCycleShapes = new ArrayList<>();
    List<PShape> obstacleShapes = new ArrayList<>();
    QueasyCam cam;
    List<Vec3> finishPositions = new ArrayList<>();

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
        cam.speed = 10f;
        sphericalAgentDescriptions = new ArrayList<>();
        //5
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * -0.6f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * -0.8f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.8f, 0, SIDE * -0.8f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.7f, 0, SIDE * -0.6f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.6f, 0, SIDE * -0.8f).plusInPlace(OFFSET));
        //6
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * -0.0f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * -0.2f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.7f, 0, SIDE * -0.2f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.7f, 0, SIDE * -0.0f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.6f, 0, SIDE * -0.0f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.6f, 0, SIDE * -0.2f).plusInPlace(OFFSET));
        //1
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * 0.4f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.8f, 0, SIDE * 0.4f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.7f, 0, SIDE * 0.4f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.6f, 0, SIDE * 0.4f).plusInPlace(OFFSET));
        //
        finishPositions.add(Vec3.of(SIDE * 0.9f, 0, SIDE * 0.8f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.8f, 0, SIDE * 0.8f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.7f, 0, SIDE * 0.8f).plusInPlace(OFFSET));
        finishPositions.add(Vec3.of(SIDE * 0.6f, 0, SIDE * 0.8f).plusInPlace(OFFSET));



        for(int i = 0 ; i < finishPositions.size(); i++){
            sphericalAgentDescriptions.add(new SphericalAgentDescription(
                    Vec3.of(SIDE * -0.9f, 0, SIDE * -0.9f).plusInPlace(OFFSET),
                    finishPositions.get(i),
                    SIDE * 0.08f
            ));
        }

        PlainConfigurationSpace configurationSpace = new PlainConfigurationSpace(
                this,
                sphericalAgentDescriptions.get(0),
                sphericalObstacles
        );
        SphericalAgent.MILESTONE_REACHED_RADIUS = 20f;
        SphericalAgent.NEXT_MILESTONE_HINT_SIZE = 18f;
        ground = new Ground(this,
                OFFSET.plus(Vec3.of(0, SIDE * 0.08f, 0)),
                Vec3.of(0, 0, 1), Vec3.of(1, 0, 0),
                2 * SIDE, 2 * SIDE,
                loadImage("ground6.png"));
        MultiAgentGraph.END_POINT_SIZE = 20f;
        MultiAgentGraph.DRAW_VERTICES = false ;
        MultiSphericalAgentSystem.MAX_EDGE_LEN = 50 ;
        multiSphericalAgentSystem = new MultiSphericalAgentSystem(this,sphericalAgentDescriptions,configurationSpace, minCorner, maxCorner);
        // tuning parameters
        SphericalAgent.IMPACT_RADIUS = 100f;
        SphericalAgent.SEPERATION_FORCE_BOID = 1f;
        SphericalAgent.SEPERATION_FORCE_OBSTACLE = 1.5f;
        SphericalAgent.ALIGNMENT_FORCE = 0.02f;
        SphericalAgent.CENTROID_FORCE = 0.02f;
        SphericalAgent.DRAW_PATH = false;

        for (int i = 0; i < 8; i++) {
            PShape agentShape = loadShape("data/robot/" + (i + 1) + ".obj");
            agentShape.rotateX(PApplet.PI);
            agentShape.rotateY(PApplet.PI);
            agentWalkCycleShapes.add(agentShape);
        }
    }

    public void draw() {
        long start = millis();
        // update
        if (SMOOTH_PATH) {
            multiSphericalAgentSystem.smoothUpdate(0.3f);
        } else {
//            multiSphericalAgentSystem.update(0.3f);
            multiSphericalAgentSystem.updateBoid(sphericalObstacles, 0.3f);
//            sphericalAgent1.update(0.1f);
        }
        long update = millis();
        // draw
        background(0);
        directionalLight(0.6f, 0.6f, 0.6f, 0, 1, -1);
        directionalLight(0.6f, 0.6f, 0.6f, 0, 1, 1);
        pointLight(1, 1, 1, 0, -10, 0);
        // obstacles
        if (DRAW_OBSTACLES) {
            for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
                sphericalObstacle.draw();
            }
        }
        // agent
        multiSphericalAgentSystem.draw(agentWalkCycleShapes, 5f);
//        checkFinish(multiSphericalAgentSystem);
        // ground
        ground.draw();
        // graph
//        graph.draw();
        long draw = millis();
        connectFinish(finishPositions);

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " search: " + SEARCH_ALGORITHM + " smooth-path: " + SMOOTH_PATH);
    }

    private void connectFinish(List<Vec3> finishPositions) {
        for(int i = 0 ; i < finishPositions.size()-1; i++){
            Vec3 v1 = finishPositions.get(i);
            Vec3 v2 = finishPositions.get(i+1);
            if(i != 4 && i!=14){
                if(i == 10){
                    v2 = v2 = finishPositions.get(7);
                }
                pushMatrix();
                strokeWeight(2);
                stroke(0,125,0);
                line(v1.x,v1.y,v1.z,v2.x,v2.y,v2.z);
                popMatrix();
            }
        }
    }

    private void checkFinish(MultiSphericalAgentSystem multiSphericalAgentSystem) {
        int i = 0 ;
        while (i < multiSphericalAgentSystem.sphericalAgents.size()){
            SphericalAgent agent =  multiSphericalAgentSystem.sphericalAgents.get(i) ;
            if(agent.hasReachedEnd()){
                multiSphericalAgentSystem.sphericalAgents.remove(i);
            }else {
                i++ ;
            }
        }
    }

    public void keyPressed() {
        if (keyCode == RIGHT) {
            multiSphericalAgentSystem.stepForward();
        }
        if (keyCode == LEFT) {
            multiSphericalAgentSystem.stepBackward();
        }
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
            multiSphericalAgentSystem.togglePause();
        }
        if (key == '1') {
            multiSphericalAgentSystem.dfs();
            SEARCH_ALGORITHM = "DFS";
        }
        if (key == '2') {
            multiSphericalAgentSystem.bfs();
            SEARCH_ALGORITHM = "BFS";
        }
        if (key == '3') {
            multiSphericalAgentSystem.ucs();
            SEARCH_ALGORITHM = "UCS";
        }
        if (key == '4') {
            multiSphericalAgentSystem.aStar();
            SEARCH_ALGORITHM = "A*";
        }
        if (key == '5') {
            float weight = 1.5f;
            multiSphericalAgentSystem.weightedAStar(weight);
            SEARCH_ALGORITHM = weight + "A*";
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.boids.HumanChain"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
