import camera.QueasyCam;
import javafx.util.Pair;
import math.Vec3;
import physical.Milestone;
import physical.SphericalAgent;
import physical.SphericalObstacle;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    QueasyCam cam;
    SphericalObstacle sphericalObstacle;
    SphericalAgent sphericalAgent;
    List<Milestone> milestones = new ArrayList<>();
    List<Pair<Vec3, Vec3>> links = new ArrayList<>();
    boolean drawLinks = true;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
        sphericalObstacle = new SphericalObstacle(
                this,
                Vec3.of(0, 0, 0),
                SIDE * (2f / 20),
                Vec3.of(1, 0, 0)
        );
        sphericalAgent = new SphericalAgent(
                this,
                Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10)),
                SIDE * (0.5f / 20),
                Vec3.of(1)
        );

        // milestone sampling
        milestones.add(new Milestone(
                this,
                Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10)),
                Vec3.of(0, 1, 0)
        ));
        milestones.add(new Milestone(
                this,
                Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10)),
                Vec3.of(0, 0, 1)
        ));
        for (int i = 0; i < 1000; ++i) {
            milestones.add(new Milestone(
                    this,
                    Vec3.of(0, random(-SIDE, SIDE), random(-SIDE, SIDE)),
                    Vec3.of(1, 1, 0)
            ));
        }

        // milestone culling
        List<Integer> badMilestones = new ArrayList<>();
        for (int i = 0; i < milestones.size(); ++i) {
            Milestone milestone = milestones.get(i);
            if (milestone.position.minus(sphericalObstacle.center).norm() <= sphericalObstacle.radius + sphericalAgent.radius) {
                milestone.color = Vec3.of(1, 0, 1);
                badMilestones.add(i);
            }
        }
        for (int i = badMilestones.size() - 1; i >= 0; --i) {
            int indexToRemove = badMilestones.get(i);
            milestones.remove(indexToRemove);
        }

        // linking milestones
        for (int i = 0; i < milestones.size(); ++i) {
            for (int j = 0; j < milestones.size(); j++) {
                Vec3 p1 = milestones.get(i).position;
                Vec3 p2 = milestones.get(j).position;
                if (p1.minus(p2).norm() < 20) {
                    links.add(new Pair<>(p1, p2));
                }
            }
        }
    }

    public void draw() {
        long start = millis();
        // update
        long update = millis();
        // draw
        background(0);
        // agent
        sphericalAgent.draw();
        // obstacle
        sphericalObstacle.draw();
        // milestones
        for (Milestone milestone : milestones) {
            milestone.draw();
        }
        // links
        if (drawLinks) {
            for (Pair<Vec3, Vec3> link : links) {
                Vec3 p1 = link.getKey();
                Vec3 p2 = link.getValue();
                stroke(1, 1, 0);
                line(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
            }
        }
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms");
    }

    public void keyPressed() {
        if (key == 'o') {
            sphericalObstacle.isDrawn = !sphericalObstacle.isDrawn;
        }
        if (key == 'p') {
            drawLinks = !drawLinks;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"Main"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
