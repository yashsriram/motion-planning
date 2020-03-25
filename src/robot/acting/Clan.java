package robot.acting;

import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;

import java.util.List;

public class Clan extends Boid {
    public boolean isBlue;
    public boolean isDead ;
    public Clan(PApplet parent, float radius, Vec3 minCorner, Vec3 maxCorner, Vec3 center, float impactRadius, List<SphericalObstacle> obstacles) {
        super(parent, radius, minCorner, maxCorner, center, impactRadius, obstacles);
        float coinFLip = parent.random(0,1);
        if(coinFLip >= 0.5){
            this.color = Vec3.of(parent.random(100, 150),parent.random(100, 150),0);
            isBlue = false;
        }
        else{
            this.color = Vec3.of(parent.random(100, 150),0,0);
            isBlue = true;
        }
        isDead = false;
    }

    public void update(List<Boid> flock, float dt, Vec3 lead){
        if(!this.isDead){
            super.update(flock, dt, lead);
        }else{
            this.color = Vec3.of(163,82,45);
        }

    }

}
