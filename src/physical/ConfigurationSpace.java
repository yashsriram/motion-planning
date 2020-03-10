package physical;

import java.util.List;

public class ConfigurationSpace {
    public SphericalAgent sphericalAgent;
    public List<SphericalObstacle> sphericalObstacles;

    public ConfigurationSpace(SphericalAgent sphericalAgent, List<SphericalObstacle> sphericalObstacles) {
        this.sphericalAgent = sphericalAgent;
        this.sphericalObstacles = sphericalObstacles;
    }
}
