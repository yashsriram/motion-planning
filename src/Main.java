import camera.QueasyCam;
import math.Vec3;
import physical.CircularObstacle;
import processing.core.PApplet;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;

    QueasyCam queasyCam;
    CircularObstacle circularObstacle;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");

        queasyCam = new QueasyCam(this);
        noStroke();
        circularObstacle = new CircularObstacle(this, Vec3.zero(), 10, Vec3.of(255, 255, 0));
    }

    public void draw() {
        long start = millis();
        // update
        long update = millis();
        // draw
        background(0);
        circularObstacle.draw();
        long draw = millis();

        surface.setTitle("Processing - FPS: " + Math.round(frameRate) + " Update: " + (update - start) + "ms Draw " + (draw - update) + "ms");
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
