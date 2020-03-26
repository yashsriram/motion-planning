package robot.acting;

import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PImage;
import processing.core.PShape;

import java.util.List;

public class Clan extends Boid {
    public boolean isBlue;
    public boolean isDead;
    PShape shape;
    PImage img ;

    public Clan(PApplet parent, float radius, Vec3 minCorner, Vec3 maxCorner, Vec3 center, float impactRadius, List<SphericalObstacle> obstacles) {
        super(parent, radius, minCorner, maxCorner, center, impactRadius, obstacles);
        float coinFLip = parent.random(0,1);
        if(coinFLip >= 0.5){
            this.color = Vec3.of(parent.random(100, 150),parent.random(100, 150),0);
            img = parent.loadImage("data/lannister.png");
            isBlue = false;
        }
        else{
            this.color = Vec3.of(parent.random(100, 150),0,0);
            img = parent.loadImage("data/targaryen.png");
            isBlue = true;
        }
        isDead = false;
        shape = parent.createShape(PConstants.SPHERE, radius);
        shape.setTexture(img);

    }

    public void update(List<Boid> flock, float dt, Vec3 lead){
        if(!this.isDead){
            super.update(flock, dt, lead);
        }else{
            this.color = Vec3.of(163,82,45);
        }

    }

    @Override
    public void draw() {
//        super.draw();
        parent.pushMatrix();
        parent.translate(center.x, center.y, center.z);
        parent.rotateY(PConstants.PI/2);
        parent.noStroke();
        parent.shape(this.shape);
        parent.popMatrix();
    }
}
