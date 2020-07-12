# motion-planning-algorithms

## description
- Simple motion planning algorithms for single agent, crowd and flocks in known/unknown environments.
- The task for an agent is to reach its goal without colliding with obstacles or other agents.
## code
- Code is written in Java, should work with JRE 8+.
    - `src/` contains all source code.
    - `jars/` contain all libraries bundled as jars.
        - `processing` is used as a rendering library.
        - `queasy cam` is used as a camera library.
    - `data/` contains resources such as images, obj, mtl files.
## documentation
- For most of the code, the documentation is itself.
## usage
- Open a terminal at project root (the directory containing this file).
- Use `javac -cp "jars/*" -d build/ $(find -name "*.java")` to compile and put all output class files under `build/`.
- Use `java -cp "build/:jars/*" <package>.<to>.<path>.<class>` to run any simulation.
    - For example `java -cp "build/:jars/*" demos.WalkCycle`.
- Common controls
    - `w a s d` for basic camera movements.
    - `q e` for camera up and down movements.
    - `p` for pause/play.
- Tested on Ubuntu 18.04
    - If you use a distrubution that uses rolling release cycle (like Arch) you might have to install some older version of JRE and mesa (opensource intel openGL driver) that work with processing library.
## demonstration
The following color scheme is used in all videos.
| Syntax      | Description |
| ----------- | ----------- |
| white sphere/line | agent |
| red sphere/line | agent in the future (useful to visualize how the path is smoothed) |
| green cube | goal |
| pink circle | obstacle |
| white point | sampled milestone |
| pink point  | milestone inside obstacle |
| green point | milestone on the fringe during search |
| red point | explored milestone during search |

The title bar shows the state of the system at any instant (current search algorithm, path smoothing on/off ...).
Most of the demos use path smoothing.

### Search methods on PRM and path smoothing
#### videos
[![](http://img.youtube.com/vi/_ZvYOEbU1mI/0.jpg)](https://www.youtube.com/watch?v=_ZvYOEbU1mI)

A circular agent is in a square room.
Its goal is to reach the other diagonal.
But there is an obstacle in the middle of the room.

#### images
| Algorithm | Image
| --- | ---
| DFS | ![](github/dfs.png)
| BFS | ![](github/bfs.png)
| UCS | ![](github/ucs.png)
| A\* | ![](github/astar.png)
| weighted A\* (1.5) | ![](github/weighted_astart.png) |

### `PRM/A*` vs `RRT` vs `RRT*`
#### videos
| PRM/A\* | RRT | RRT\* |
| ----------- | ----------- | ----------- |
| [![](http://img.youtube.com/vi/4pUF_Zvpd20/0.jpg)](https://www.youtube.com/watch?v=4pUF_Zvpd20) | [![](http://img.youtube.com/vi/duChnMpVGxI/0.jpg)](https://www.youtube.com/watch?v=duChnMpVGxI) | [![](http://img.youtube.com/vi/RMHfadOEfeg/0.jpg)](https://www.youtube.com/watch?v=RMHfadOEfeg)
| [![](http://img.youtube.com/vi/CfLtVadzC4s/0.jpg)](https://www.youtube.com/watch?v=CfLtVadzC4s) | [![](http://img.youtube.com/vi/iS9O4PCP0tU/0.jpg)](https://www.youtube.com/watch?v=iS9O4PCP0tU) | [![](http://img.youtube.com/vi/1VMYDos3AgA/0.jpg)](https://www.youtube.com/watch?v=1VMYDos3AgA)
| [![](http://img.youtube.com/vi/JDKTm5UFPIE/0.jpg)](https://www.youtube.com/watch?v=JDKTm5UFPIE) | [![](http://img.youtube.com/vi/RyKUdOMsHhc/0.jpg)](https://www.youtube.com/watch?v=RyKUdOMsHhc) | [![](http://img.youtube.com/vi/OOG0S0QjUSs/0.jpg)](https://www.youtube.com/watch?v=OOG0S0QjUSs)
| Graph | Growing tree | Growing and mutating tree
| Takes more memory | Takes less memory | Takes less memory
| Build time depends on environment | Build time depends on environment | Build time depends on environment
| Probabilistically complete (Finds a path at limit) | Probabilistically complete (Finds a path at limit) | Probabilistically complete (Finds a path at limit)
| Asymptotically optimal (Finds optimal path at limit) | Asymptotically NON optimal (Maynot optimal path at limit) | Asymptotically optimal (Finds optimal path at limit)

- In `RRT` and `RRT*` the tree is grown interactively.
- In `RRT` note that even when tree grows considerably the path to finish does not change and more importantly is not a shortest one.
- In `RRT\*` note that as tree grows the path to finish decreases until it reaches a shortest path at limit.

### Crowds using `time-to-collision` method
#### videos
- Simple case.

[![](http://img.youtube.com/vi/37fwqM4V46M/0.jpg)](https://www.youtube.com/watch?v=37fwqM4V46M)

- Interactive placement.

[![](http://img.youtube.com/vi/VVYf8ZLg4Y0/0.jpg)](https://www.youtube.com/watch?v=VVYf8ZLg4Y0)

- Antipodes.

[![](http://img.youtube.com/vi/dxd10-PlHgU/0.jpg)](https://www.youtube.com/watch?v=dxd10-PlHgU)

- Traffic circle.

[![](http://img.youtube.com/vi/ItN2N-VQiYg/0.jpg)](https://www.youtube.com/watch?v=ItN2N-VQiYg)

- Bottleneck.

[![](http://img.youtube.com/vi/fLOeLoSkX-g/0.jpg)](https://www.youtube.com/watch?v=fLOeLoSkX-g)

- Crosspaths.

[![](http://img.youtube.com/vi/2uLhvvCwcFk/0.jpg)](https://www.youtube.com/watch?v=2uLhvvCwcFk)

- Hallway.

[![](http://img.youtube.com/vi/nFPgeXd-7jI/0.jpg)](https://www.youtube.com/watch?v=nFPgeXd-7jI)

- 2D swarms.

[![](http://img.youtube.com/vi/YzPFQAHuiNA/0.jpg)](https://www.youtube.com/watch?v=YzPFQAHuiNA)

- 3D swarms.

[![](http://img.youtube.com/vi/qP5CNbdWYgo/0.jpg)](https://www.youtube.com/watch?v=qP5CNbdWYgo)

### Crowds using `boids` method
#### videos
- Two flocks.

[![](http://img.youtube.com/vi/mECsomLgSSY/0.jpg)](https://www.youtube.com/watch?v=mECsomLgSSY)

### `boids` vs `time-to-collision`
#### videos
| time-to-collision | boids |
| ----------- | ----------- |
| [![](http://img.youtube.com/vi/6dMgVDwFzH8/0.jpg)](https://www.youtube.com/watch?v=6dMgVDwFzH8) | [![](http://img.youtube.com/vi/xDQGSYznX1g/0.jpg)](https://www.youtube.com/watch?v=xDQGSYznX1g) |
| [![](http://img.youtube.com/vi/p0aMsLU1KDo/0.jpg)](https://www.youtube.com/watch?v=p0aMsLU1KDo) | [![](http://img.youtube.com/vi/6swq7i15GmA/0.jpg)](https://www.youtube.com/watch?v=6swq7i15GmA) |

### Failing/Odd cases
#### videos

- Too narrow passages.

[![](http://img.youtube.com/vi/dzMZYY5M3_k/0.jpg)](https://www.youtube.com/watch?v=dzMZYY5M3_k)

- Takes a lot of time at the junction for the traffic to clear due to high symmetry.

[![](http://img.youtube.com/vi/V7wa66PK3dg/0.jpg)](https://www.youtube.com/watch?v=V7wa66PK3dg)

- Some agents are beings captured by other flock.

[![](http://img.youtube.com/vi/jXSMevpM_mc/0.jpg)](https://www.youtube.com/watch?v=jXSMevpM_mc)

### Misc
#### videos
- 3D context.

[![](http://img.youtube.com/vi/xLF955Qc3QM/0.jpg)](https://www.youtube.com/watch?v=xLF955Qc3QM)

- Walkcycle.

[![](http://img.youtube.com/vi/FwlAQXOEGBI/0.jpg)](https://www.youtube.com/watch?v=FwlAQXOEGBI)

- Bounding sphere heirarchy.

[![](http://img.youtube.com/vi/1uuFNwSa1_U/0.jpg)](https://www.youtube.com/watch?v=1uuFNwSa1_U)

- Bounding sphere heirarchy is used as a spatial data structure to cull the edges of PRM that intersect obstacles.
- The title bar shows the current state (data structure used, time taken to cull edges, #obstacles).
- `b` - reset using bsh data structure.
- `v` - reset using vanilla data structure.
- ~10k obstacles almost uniformly spaced.
- Obstacles visualized @ `00:23`.
- Free edges visualizes @ `00:30`.
- BSH edge culling @ `00:46 - 00:56`, done several times.
- Vanilla edge culling @ `00:58 - 01:10`, done barely once, screen is unresponsive in that time.
- Bounding spheres visualized @ `01:35`.
- BSH edge culling time ~ 1000ms.
- Vanilla edge culling time ~ 12900ms.
- 12.9x faster.
- Speedup increases with setup size.

### Graph search time comparision
![](github/search_time_comparision.png)
- The graph shows the average run time of the algorithms for searching the path in milliseconds.
- Each algorithm was run in simulations with 196 agents and the average time of searching the path for all the agents is calculated.
- There is a sharp decrease in search time with A\* and weighted A\*.
- Most of the demos use A\* search.

### 2D Rotation
- The agent is a directed linesegment with direction visualized by two boxes at center and one end.
- The agent makes 180 degree turn in its journey from start to finish.
- The paths in 3D configuration space and its mapping in 2D as position and orientation is visualized.
- The milestones inside obstacles in 3D configutation space are colored pink.
- Path smoothing is done using a bounding sphere for the agent and is demonstrated.
- The title bar shows whether or not smooth pathing is done and which search is used.
#### videos
[![](http://img.youtube.com/vi/YxobpU-Am8k/0.jpg)](https://www.youtube.com/watch?v=YxobpU-Am8k)

[![](http://img.youtube.com/vi/i_CAbNsm0f4/0.jpg)](https://www.youtube.com/watch?v=i_CAbNsm0f4)

### Exploring (a static) world on the way
- The robot does not know where the obstacles are.
- It has a sensor which tells the positions of obstacles in a fixed sphere around the it.
- Therfore it doesnot know what exists beyond that sensing sphere.
- It need to reach the goal on top right.
- The yellow points represent sensed region.
- The agent always stays inside sensed region.
- The milestones found to be inside obstacles are colored pink.
- Green and red points are related to search as before.
- Note that in videos the obstacles are sometimes hidden to visualize what the world looks like for the agent.
- Path smoothing is done only inside the sensed region is demonstrated.
- The title bar shows whether or not smooth pathing is done and which search is used.
#### the essensce of D\* lite
- Sense
- Plan to go to finish using milestones as if there are no other obstacles other than what is found
- Move along that path (until the edge of sensed area)
- If finish reached stop, else go to step 1
#### videos
- Wall
- Path smoothing @ `00:23`.

[![](http://img.youtube.com/vi/1fdoryW57Wk/0.jpg)](https://www.youtube.com/watch?v=1fdoryW57Wk)

- Big ball

[![](http://img.youtube.com/vi/JL-EimCRz0o/0.jpg)](https://www.youtube.com/watch?v=JL-EimCRz0o)

- Unreachable

[![](http://img.youtube.com/vi/g0x1sf4M2bU/0.jpg)](https://www.youtube.com/watch?v=g0x1sf4M2bU)

- Zig zag

[![](http://img.youtube.com/vi/gPv0E11VV2Y/0.jpg)](https://www.youtube.com/watch?v=gPv0E11VV2Y)
