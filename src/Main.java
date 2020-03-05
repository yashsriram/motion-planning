import camera.QueasyCam;
import math.Vec3;
import physical.Link;
import physical.Milestone;
import physical.SphericalAgent;
import physical.SphericalObstacle;
import processing.core.PApplet;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static final int SIDE = 100;

    final Vec3 start = Vec3.of(0, SIDE * (9f / 10), SIDE * (-9f / 10));
    final Vec3 finish = Vec3.of(0, SIDE * (-9f / 10), SIDE * (9f / 10));
    SphericalAgent sphericalAgent;
    SphericalObstacle sphericalObstacle;
    final List<Milestone> milestones = new ArrayList<>();
    final List<Link> links = new ArrayList<>();

    QueasyCam cam;
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
                start,
                SIDE * (0.5f / 20),
                Vec3.of(1)
        );

        // milestone sampling: plain
        milestones.add(new Milestone(
                this,
                start,
                Vec3.of(0, 1, 0)
        ));
        milestones.add(new Milestone(
                this,
                finish,
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
//            milestones.remove(indexToRemove);
        }

        // linking milestones
        for (int i = 0; i < milestones.size(); ++i) {
            for (int j = 0; j < milestones.size(); j++) {
                Vec3 p1 = milestones.get(i).position;
                Vec3 p2 = milestones.get(j).position;
                if (p1.minus(p2).norm() < 20) {
                    links.add(new Link(this, p1, p2, Vec3.of(1, 1, 0)));
                }
            }
        }

        // link culling
        for (Link link : links) {
            Vec3 pb_pa = link.p2.minus(link.p1);
            Vec3 pa_pc = link.p1.minus(sphericalObstacle.center);
            float r = sphericalObstacle.radius;
            float a = pb_pa.dot(pb_pa);
            float c = pa_pc.dot(pa_pc) - r * r;
            float b = 2 * pb_pa.dot(pa_pc);
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                float t1 = (float) ((-b + Math.sqrt(discriminant)) / (2 * a));
                float t2 = (float) ((-b - Math.sqrt(discriminant)) / (2 * a));
                if ((0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1)) {
                    link.color = Vec3.of(1, 0, 1);
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
            for (Link link : links) {
                link.draw();
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
