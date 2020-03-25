package demos;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import robot.acting.Boid;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.rrt.RapidlyExploringRandomTree;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class Crowd extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);
    final Vec3 center = Vec3.of(0, 0,0 );

    final Vec3 startPosition = Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finishPosition = Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10));
    SphericalAgentDescription sphericalAgentDescription;
    SphericalAgent sphericalAgent;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    PlainConfigurationSpace configurationSpace;
    RapidlyExploringRandomTree rrt;

    List<Boid> flock = new ArrayList<>();

    QueasyCam cam;

    static boolean DRAW_OBSTACLES = true;
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
        sphericalAgentDescription = new SphericalAgentDescription(
                startPosition,
                finishPosition,
                SIDE * (0.5f / 20)
        );

        configurationSpace = new PlainConfigurationSpace(this, sphericalAgentDescription, sphericalObstacles);
        sphericalAgent = new SphericalAgent(this, sphericalAgentDescription, configurationSpace, minCorner, maxCorner, 20f, Vec3.of(1));
        rrt = new RapidlyExploringRandomTree(this, startPosition, finishPosition);
        rrt.growTree(sphericalAgent.samplePoints(100), configurationSpace);

        for(int i = 0 ; i < 50; i++){
            flock.add(new Boid(this, 5,minCorner, maxCorner, Vec3.of(0, SIDE * (random(-10,10) / 10), SIDE * (random(-10,10) / 10)), 20, sphericalObstacles));
        }

    }

    public void draw(){
        if (keyPressed) {
            if (key == 'n') {
                rrt.growTree(sphericalAgent.samplePoints(10), configurationSpace);
            }
        }
        // update
        if (SMOOTH_PATH) {
            sphericalAgent.smoothUpdate(0.1f);
        } else {
            sphericalAgent.update(0.1f);
        }
        // draw
        background(0);
        // obstacles
        if (DRAW_OBSTACLES) {
            for (SphericalObstacle sphericalObstacle : sphericalObstacles) {
                sphericalObstacle.draw();
            }
        }
        // configuration space
        configurationSpace.draw();
        // rrt
        rrt.draw();
        // agent
        sphericalAgent.draw();
        for(Boid boid : flock){
            boid.draw();
            boid.update(flock, 0.01f, sphericalAgent.getCenter());
        }

        surface.setTitle("Processing - FPS: " + Math.round(frameRate));
    }

    public void keyPressed(){
        if (key == 'h') {
            DRAW_OBSTACLES = !DRAW_OBSTACLES;
        }
        if (key == '1') {
            sphericalAgent.setPath(rrt.search());
        }
        if (key == 'p') {
            sphericalAgent.isPaused = !sphericalAgent.isPaused;
        }
        if (key == 'x') {
            SMOOTH_PATH = !SMOOTH_PATH;
        }
        if (keyCode == RIGHT) {
            sphericalAgent.stepForward();
        }
        if (keyCode == LEFT) {
            sphericalAgent.stepBackward();
        }
        if (key == 'g') {
            RapidlyExploringRandomTree.DRAW_TREE = !RapidlyExploringRandomTree.DRAW_TREE;
        }
        if(key=='i'){
            center.y -= 20;
        }
        if(key=='k'){
            center.y += 20;
        }
        if(key=='l'){
            center.z += 20;
        }
        if(key=='j'){
            center.z -= 20;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.Crowd"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
