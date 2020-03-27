package robot.acting;

import fixed.SphericalObstacle;
import math.Vec3;
import processing.core.PApplet;
import processing.core.PShape;
import robot.input.SphericalAgentDescription;
import robot.planning.multiagentgraph.MultiAgentGraph;
import robot.sensing.ConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class MultiSphericalAgentSystem {
    public static float AGENT_SPEED = 20f;
    public static float MAX_EDGE_LEN = 10f;
    public static int NUM_VERTEX_SAMPLES = 10000;
    public static float TTC_K = 10;
    public static float TTC_T0 = 1;

    final PApplet parent;
    final ConfigurationSpace configurationSpace;
    final MultiAgentGraph multiAgentGraph;
    public List<SphericalAgent> sphericalAgents = new ArrayList<>();

    public MultiSphericalAgentSystem(PApplet parent, List<SphericalAgentDescription> sphericalAgentDescriptions, ConfigurationSpace configurationSpace, Vec3 minCorner, Vec3 maxCorner) {
        this.parent = parent;
        // At least one spherical agent is required
        assert (sphericalAgentDescriptions.size() > 0);
        for (SphericalAgentDescription sphericalAgentDescription : sphericalAgentDescriptions) {
            sphericalAgents.add(
                    new SphericalAgent(parent,
                            sphericalAgentDescription,
                            configurationSpace,
                            minCorner, maxCorner,
                            AGENT_SPEED,
                            Vec3.of(parent.random(1), parent.random(1), parent.random(1))
                    )
            );
        }
        this.configurationSpace = configurationSpace;
        this.multiAgentGraph = new MultiAgentGraph(parent, sphericalAgentDescriptions);
        this.multiAgentGraph.generateVertices(sphericalAgents.get(0).samplePoints(NUM_VERTEX_SAMPLES), configurationSpace);
        this.multiAgentGraph.generateAdjacencies(MAX_EDGE_LEN, configurationSpace);
    }

    public void update(float dt) {
        for (SphericalAgent agent : sphericalAgents) {
            agent.update(dt);
        }
    }

    public void smoothUpdate(float dt) {
        for (SphericalAgent agent : sphericalAgents) {
            agent.smoothUpdate(dt);
        }
    }

    public void updateBoid(List<SphericalObstacle> obstacles, float dt) {
        for (SphericalAgent agent : sphericalAgents) {
            agent.boidUpdate(sphericalAgents, obstacles, dt);
        }
    }

    public void updateTTC(List<SphericalObstacle> sphericalObstacles, float dt) {
        // Get isolated velocities
        List<Vec3> isolatedVelocities = new ArrayList<>();
        for (SphericalAgent agent : sphericalAgents) {
            isolatedVelocities.add(agent.getIsolatedVelocity());
        }
        // Compute ttc forces
        List<Vec3> totalTTCForces = new ArrayList<>();
        for (int i = 0; i < sphericalAgents.size(); i++) {
            totalTTCForces.add(Vec3.of(0));
        }
        // Agent-agent interaction
        for (int i = 0; i < sphericalAgents.size() - 1; i++) {
            SphericalAgent agentI = sphericalAgents.get(i);
            Vec3 agentIVel = isolatedVelocities.get(i);
            for (int j = i + 1; j < sphericalAgents.size(); j++) {
                // for all i != j
                SphericalAgent agentJ = sphericalAgents.get(j);
                Vec3 agentJVel = isolatedVelocities.get(j);
                Vec3 xji = agentJ.center.minus(agentI.center);
                float distance = xji.norm();
                float impactRadius = agentI.description.radius + agentJ.description.radius + 5;
                if (distance < impactRadius) {
                    // Avoid collision (which could be happening probably due to slow incoming agents)
                    Vec3 separationForce = xji.normalize().scaleInPlace(35 * (impactRadius - distance));
                    Vec3 agentIForce = separationForce.scale(-1);
                    Vec3 agentJForce = separationForce;
                    totalTTCForces.set(i, totalTTCForces.get(i).plusInPlace(agentIForce));
                    totalTTCForces.set(j, totalTTCForces.get(j).plusInPlace(agentJForce));
                    continue;
                }
                Vec3 vji = agentJVel.minus(agentIVel);
                // Collision detection
                final float a = vji.dot(vji);
                if (a < 1e-6) {
                    // Almost relatively stationary
                    continue;
                }
                final float b = xji.dot(vji);
                final float c = xji.dot(xji) - (agentI.description.radius + agentJ.description.radius) * (agentI.description.radius + agentJ.description.radius);
                float desc = b * b - a * c;
                if (desc < 0) {
                    // No collision
                    continue;
                }
                double t1 = (-b - Math.sqrt(desc)) / a;
                double t2 = (-b + Math.sqrt(desc)) / a;
                if (t1 < 0 && t2 < 0) {
                    // (-, -)
                    // No collision
                    continue;
                }
                double timeToCollision = -1;
                if ((t1 > 0 && t2 < 0) || (t2 > 0 && t1 < 0)) {
                    // (+, -) (-, +) case
                    // collision occurs
                    timeToCollision = Math.max(t1, t2);
                } else {
                    // (+, +) case
                    // collision occurs
                    timeToCollision = Math.min(t1, t2);
                }
                // Calculate ttcForce
                double firstTerm = (TTC_K * Math.exp(-timeToCollision / TTC_T0)) / (timeToCollision * timeToCollision * a);
                double secondTerm = (2 / timeToCollision + 1 / TTC_T0);
                Vec3 thirdTerm = xji.scale(a).minus(vji.scale(b)).scaleInPlace(1 / (float) Math.sqrt(desc));
                Vec3 fourthTerm = vji;
                Vec3 ttcForce = thirdTerm.minus(fourthTerm).scaleInPlace((float) (-1 * firstTerm * secondTerm));
                // Newtons 3rd law
                Vec3 agentI_Force = ttcForce;
                Vec3 agentJ_Force = ttcForce.scale(-1);
                // Adding to existing ttc forces
                totalTTCForces.set(i, totalTTCForces.get(i).plusInPlace(agentI_Force));
                totalTTCForces.set(j, totalTTCForces.get(j).plusInPlace(agentJ_Force));
            }
        }

        // Agent obstacle interaction
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agentI = sphericalAgents.get(i);
            Vec3 agentIVel = isolatedVelocities.get(i);
            for (SphericalObstacle obstacleJ : sphericalObstacles) {
                Vec3 xji = obstacleJ.center.minus(agentI.center);
                float distance = xji.norm();
                float impactRadius = agentI.description.radius + obstacleJ.radius + 5;
                if (distance < impactRadius) {
                    // Avoid collision (which could be happening probably due to slow incoming agents)
                    Vec3 separationForce = xji.normalize().scaleInPlace(30 * (impactRadius - distance));
                    Vec3 agentIForce = separationForce.scale(-1);
                    totalTTCForces.set(i, totalTTCForces.get(i).plusInPlace(agentIForce));
                    continue;
                }
                Vec3 vji = agentIVel.scale(-1);
                // Collision detection
                final float a = vji.dot(vji);
                if (a < 1e-6) {
                    // Almost relatively stationary
                    continue;
                }
                final float b = xji.dot(vji);
                final float c = xji.dot(xji) - (agentI.description.radius + obstacleJ.radius) * (agentI.description.radius + obstacleJ.radius);
                float desc = b * b - a * c;
                if (desc < 0) {
                    // No collision
                    continue;
                }
                double t1 = (-b - Math.sqrt(desc)) / a;
                double t2 = (-b + Math.sqrt(desc)) / a;
                if (t1 < 0 && t2 < 0) {
                    // (-, -)
                    // No collision
                    continue;
                }
                double timeToCollision = -1;
                if ((t1 > 0 && t2 < 0) || (t2 > 0 && t1 < 0)) {
                    // (+, -) (-, +) case
                    // collision occurs
                    timeToCollision = Math.max(t1, t2);
                } else {
                    // (+, +) case
                    // collision occurs
                    timeToCollision = Math.min(t1, t2);
                }
                // Calculate ttcForce
                double firstTerm = (TTC_K * Math.exp(-timeToCollision / TTC_T0)) / (timeToCollision * timeToCollision * a);
                double secondTerm = (2 / timeToCollision + 1 / TTC_T0);
                Vec3 thirdTerm = xji.scale(a).minus(vji.scale(b)).scaleInPlace(1 / (float) Math.sqrt(desc));
                Vec3 fourthTerm = vji;
                Vec3 ttcForce = thirdTerm.minus(fourthTerm).scaleInPlace((float) (-1 * firstTerm * secondTerm));
                // Adding to existing ttc forces
                totalTTCForces.set(i, totalTTCForces.get(i).plusInPlace(ttcForce));
            }
        }

        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            Vec3 isolatedVelocity = isolatedVelocities.get(i);
            Vec3 ttcForce = totalTTCForces.get(i);
            Vec3 ttcVelocity = ttcForce.scale(dt);
            Vec3 displacement = isolatedVelocity.plusInPlace(ttcVelocity).scale(dt);
            agent.ttcUpdate(displacement);
        }
    }

    public void draw() {
        // agents
        for (SphericalAgent agent : sphericalAgents) {
            agent.draw();
        }
        // graph
        multiAgentGraph.draw();
    }

    public void draw(List<PShape> agentWalkCycleShapes, float size) {
        // agents
        for (SphericalAgent agent : sphericalAgents) {
            agent.draw(agentWalkCycleShapes, size);
        }
        // graph
        multiAgentGraph.draw();
    }

    public void stepForward() {
        for (SphericalAgent agent : sphericalAgents) {
            agent.stepForward();
        }
    }

    public void stepBackward() {
        for (SphericalAgent agent : sphericalAgents) {
            agent.stepBackward();
        }
    }

    public void togglePause() {
        for (SphericalAgent agent : sphericalAgents) {
            agent.isPaused = !agent.isPaused;
        }
    }

    public void dfs() {
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            agent.setPath(multiAgentGraph.dfs(i));
        }
    }

    public void bfs() {
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            agent.setPath(multiAgentGraph.bfs(i));
        }
    }

    public void ucs() {
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            agent.setPath(multiAgentGraph.ucs(i));
        }
    }

    public void aStar() {
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            agent.setPath(multiAgentGraph.aStar(i));
        }
    }

    public void weightedAStar(float epsilon) {
        for (int i = 0; i < sphericalAgents.size(); i++) {
            SphericalAgent agent = sphericalAgents.get(i);
            agent.setPath(multiAgentGraph.weightedAStar(epsilon, i));
        }
    }

}
