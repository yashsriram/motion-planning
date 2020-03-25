package demos;

import camera.QueasyCam;
import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import processing.core.PImage;
import robot.acting.BoidAgent;
import robot.acting.ClanAgent;
import robot.acting.SphericalAgent;
import robot.input.SphericalAgentDescription;
import robot.planning.rrt.RapidlyExploringRandomTree;
import robot.sensing.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class Clash extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;
    final Vec3 minCorner = Vec3.of(0, -SIDE, -SIDE);
    final Vec3 maxCorner = Vec3.of(0, SIDE, SIDE);

    final Vec3 finishPosition = Vec3.of(0,0, 0);
    List<Vec3> startPositions = new ArrayList<>() ;
    List<SphericalAgentDescription> descriptions = new ArrayList<>() ;
    List<SphericalObstacle> sphericalObstacles = new ArrayList<>();
    List<SphericalAgent> agents = new ArrayList<>() ;
    List<PlainConfigurationSpace> configurationSpaces = new ArrayList<>() ;
    List<RapidlyExploringRandomTree> rrts = new ArrayList<>() ;
    List<BoidAgent> flock = new ArrayList<>();
    PImage bg ;

    QueasyCam cam;

    static boolean DRAW_OBSTACLES = true;
    static boolean SMOOTH_PATH = false;
    static boolean WAR = false ;
    static final float impactRadius = 20 ;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
        bg = loadImage("data/Houses.jpg") ;
        startPositions.add(Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10))) ;
        startPositions.add(Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10))) ;

//        sphericalObstacles.add(new SphericalObstacle(
//                this,
//                Vec3.of(0, SIDE * (4f / 10), SIDE * (-1f / 10)),
//                SIDE * (2f / 20),
//                Vec3.of(1, 0, 1)
//        ));
        sphericalObstacles.add(new SphericalObstacle(
                this,
                Vec3.of(0, SIDE * (-1f / 10), SIDE * (6f / 10)),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 1)
        ));
//        sphericalObstacles.add(new SphericalObstacle(
//                this,
//                Vec3.of(0, SIDE * (-3f / 10), SIDE * (-2f / 10)),
//                SIDE * (2f / 20),
//                Vec3.of(1, 0, 1)
//        ));
        sphericalObstacles.add(new SphericalObstacle(
                this,
                Vec3.of(0, SIDE * (-1f / 10), SIDE * (-7f / 10)),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 1)
        ));
        descriptions.add(new SphericalAgentDescription(startPositions.get(0),finishPosition,SIDE * (0.5f / 20))) ;
        descriptions.add(new SphericalAgentDescription(startPositions.get(1),finishPosition,SIDE * (0.5f / 20))) ;

        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(0), sphericalObstacles)) ;
        configurationSpaces.add(new PlainConfigurationSpace(this, descriptions.get(1), sphericalObstacles)) ;

        agents.add(new SphericalAgent(this, descriptions.get(0), configurationSpaces.get(0), minCorner, maxCorner, 20f, Vec3.of(1))) ;
        agents.add(new SphericalAgent(this, descriptions.get(1), configurationSpaces.get(1), minCorner, maxCorner, 20f, Vec3.of(1))) ;

        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(0), finishPosition)) ;
        rrts.add(new RapidlyExploringRandomTree(this, startPositions.get(1), finishPosition)) ;

        rrts.get(0).growTree(agents.get(0).samplePoints(100), configurationSpaces.get(0));
        rrts.get(0).growTree(agents.get(1).samplePoints(100), configurationSpaces.get(1));

        for(int i = 0 ; i < 100; i++){
            flock.add(new ClanAgent(this, 5,minCorner, maxCorner, Vec3.of(0, SIDE * (random(-10,10) / 10), SIDE * (random(-10,10) / 10)), impactRadius, sphericalObstacles));
        }
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
        background(bg);
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
//        for(RapidlyExploringRandomTree rrt : rrts){
//            rrt.draw();
//        }

        // agent
//        for(SphericalAgent agent : agents){
//            agent.draw();
//        }

        long draw = millis();

        if(WAR){
            fight(flock);
        }

        for(BoidAgent boidAgent : flock){
            ClanAgent cl = (ClanAgent) boidAgent;
            if(cl.isBlue){
                boidAgent.update(flock, 0.01f, agents.get(1).getCenter());
            }
            else{
                boidAgent.update(flock, 0.01f, agents.get(0).getCenter());
            }
            boidAgent.draw();
        }

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms" + " smooth-path: " + SMOOTH_PATH);
    }

    public void fight(List<BoidAgent> flock) {
        int size = flock.size() ;
        boolean []change = new boolean[size] ;
        for(BoidAgent boidAgent : flock){
            ClanAgent cl = (ClanAgent) boidAgent;
            float enemies = 0 ;
            float neighbors = 0 ;
            for(int i = 0 ; i < size; i++){
                Vec3 force =  cl.center.minus(flock.get(i).center);
                float distance = force.norm() ;
                if( distance < cl.impactRadius && distance > 0 && !cl.isDead ){
                    ClanAgent n = (ClanAgent) flock.get(i);
                    if(!n.isDead && n.isBlue!=cl.isBlue){
                        neighbors += 1 ;
                        enemies += 1 ;
                    }
                    else{
                        neighbors += 1 ;
                    }
                }
            }
            if((enemies/neighbors) > 0.8){
                cl.isDead = true ;
                change[flock.indexOf(boidAgent)] = true ;
            }
            if(neighbors < 8){
                if(enemies > 2){
                    cl.isDead = true ;
                    change[flock.indexOf(boidAgent)] = true ;
                }
            }
        }
//        int i = 0 ;
//        while (i != flock.size()){
//            if(change[i]){
//                flock.remove(i);
//            }
//            else{
//                i += 1 ;
//            }
//        }
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
        if(key == 'y'){
            WAR = !WAR ;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"demos.Clash"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }

}
