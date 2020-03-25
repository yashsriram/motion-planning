package demos.rrt;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.rrt.RapidlyExploringRandomTree;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class MultiRRT extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);

    List<Vec3> startPositions = new ArrayList<>() ;
    final Vec3 finishPosition = Vec3.of(0, SIDE * (-1f / 10), SIDE * (1f / 10));
    List<SphericalAgentDescription> descriptions = new ArrayList<>() ;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    List<SphericalAgent> agents = new ArrayList<>() ;
    List<PlainConfigurationSpace> configurationSpaces = new ArrayList<>() ;
    List<RapidlyExploringRandomTree> rrts = new ArrayList<>() ;


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
        startPositions.add(Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10))) ;
        startPositions.add(Vec3.of(0, SIDE * (9f / 10), SIDE * (-9.2f / 10))) ;
        startPositions.add(Vec3.of(0, SIDE * (8.7f / 10), SIDE * (-9f / 10))) ;
        startPositions.add(Vec3.of(0, SIDE * (9.5f / 10), SIDE * (-9f / 10))) ;
        startPositions.add(Vec3.of(0, SIDE * (8.5f / 10), SIDE * (-9.4f / 10))) ;

        sphericalObstacles.add(new SphericalObstacle(
                this,
                Vec3.of(0, 0, 0),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 1)
        ));
        descriptions.add(new SphericalAgentDescription(startPositions.get(0),finishPosition,SIDE * (0.5f / 20))) ;
        descriptions.add(new SphericalAgentDescription(startPositions.get(1),finishPosition,SIDE * (0.5f / 20))) ;
        descriptions.add(new SphericalAgentDescription(startPositions.get(2),finishPosition,SIDE * (0.5f / 20))) ;
        descriptions.add(new SphericalAgentDescription(startPositions.get(3),finishPosition,SIDE * (0.5f / 20))) ;
        descriptions.add(new SphericalAgentDescription(startPositions.get(4),finishPosition,SIDE * (0.5f / 20))) ;

        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(0), sphericalObstacles)) ;
        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(1), sphericalObstacles)) ;
        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(2), sphericalObstacles)) ;
        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(3), sphericalObstacles)) ;
        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(4), sphericalObstacles)) ;

        agents.add(new SphericalAgent(this, descriptions.get(0), configurationSpaces.get(0), minCorner, maxCorner, 20f, Vec3.of(1))) ;
        agents.add(new SphericalAgent(this, descriptions.get(1), configurationSpaces.get(1), minCorner, maxCorner, 20f, Vec3.of(1))) ;
        agents.add(new SphericalAgent(this, descriptions.get(2), configurationSpaces.get(2), minCorner, maxCorner, 20f, Vec3.of(1))) ;
        agents.add(new SphericalAgent(this, descriptions.get(3), configurationSpaces.get(3), minCorner, maxCorner, 20f, Vec3.of(1))) ;
        agents.add(new SphericalAgent(this, descriptions.get(4), configurationSpaces.get(4), minCorner, maxCorner, 20f, Vec3.of(1))) ;

        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(0), finishPosition)) ;
        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(1), finishPosition)) ;
        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(2), finishPosition)) ;
        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(3), finishPosition)) ;
        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(4), finishPosition)) ;

        rrts.get(0).growTree(agents.get(0).samplePoints(100), configurationSpaces.get(0));
        rrts.get(0).growTree(agents.get(1).samplePoints(100), configurationSpaces.get(1));
        rrts.get(0).growTree(agents.get(2).samplePoints(100), configurationSpaces.get(2));
        rrts.get(0).growTree(agents.get(3).samplePoints(100), configurationSpaces.get(3));
        rrts.get(0).growTree(agents.get(4).samplePoints(100), configurationSpaces.get(4));
    }

    public void draw() {
        if (keyPressed) {
            if (key == 'n') {
                for(int i = 0 ; i < rrts.size(); i++){
                    RapidlyExploringRandomTree rrt = rrts.get(i) ;
                    rrt.growTree(agents.get(i).samplePoints(10), configurationSpaces.get(i));
                }

            }
        }
        long start = millis();
        // update
        for(SphericalAgent agent : agents){
            agent.update(0.1f);
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
        // configuration space
        for(PlainConfigurationSpace space : configurationSpaces){
            space.draw();
        }

        // rrt
        for(RapidlyExploringRandomTree rrt : rrts){
            rrt.draw();
        }

        // agent
        for(SphericalAgent agent : agents){
            agent.draw();
        }

        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " smooth-path: " + SMOOTH_PATH);
    }

    public void keyPressed() {
        if (key == 'h') {
            DRAW_OBSTACLES = !DRAW_OBSTACLES;
        }
        if (key == '1') {
            for(int i = 0 ; i < agents.size(); i++){
                agents.get(i).setPath(rrts.get(i).search());
            }
        }
        if (key == 'j') {
            RapidlyExploringRandomTree.DRAW_TREE = !RapidlyExploringRandomTree.DRAW_TREE;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.rrt.MultiRRT"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
