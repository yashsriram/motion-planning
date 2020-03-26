package robot.acting;

import math.Vec3;
import processing.core.PApplet;
import robot.input.SphericalAgentDescription;
import robot.planning.multiagentgraph.MultiAgentGraph;
import robot.sensing.ConfigurationSpace;

import java.util.ArrayList;
import java.util.List;

public class MultiSphericalAgentSystem {
    final PApplet parent;
    final ConfigurationSpace configurationSpace;
    final MultiAgentGraph multiAgentGraph;
    final List<SphericalAgent> sphericalAgents = new ArrayList<>();

    public MultiSphericalAgentSystem(PApplet parent, List<SphericalAgentDescription> sphericalAgentDescriptions, ConfigurationSpace configurationSpace, Vec3 minCorner, Vec3 maxCorner) {
        this.parent = parent;
        for (SphericalAgentDescription sphericalAgentDescription : sphericalAgentDescriptions) {
            sphericalAgents.add(
                    new SphericalAgent(parent,
                            sphericalAgentDescription,
                            configurationSpace,
                            minCorner, maxCorner,
                            20f,
                            Vec3.of(parent.random(1), parent.random(1), parent.random(1))
                    )
            );
        }
        this.configurationSpace = configurationSpace;
        this.multiAgentGraph = new MultiAgentGraph(parent, sphericalAgentDescriptions);
        this.multiAgentGraph.generateVertices(sphericalAgents.get(0).samplePoints(10000), configurationSpace);
        this.multiAgentGraph.generateAdjacencies(10, configurationSpace);
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

    public void draw() {
        // agents
        for (SphericalAgent agent : sphericalAgents) {
            agent.draw();
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
