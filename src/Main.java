import camera.QueasyCam;
import math.Vec3;
import physical.SphericalAgent;
import physical.SphericalAgentDescription;
import physical.SphericalObstacle;
import processing.core.PApplet;
import tools.Graph;
import tools.Vertex;
import tools.configurationspace.BSHConfigurationSpace;
import tools.configurationspace.PlainConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;

    QueasyCam cam;

    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        cam = new QueasyCam(this);
    }

    public void draw() {
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
