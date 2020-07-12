# motion planning

## description
Simple motion planning algorithms for single agent, crowd and flocks in known/unknown environments.
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
In each illustration, agent(s) navigate through an environment using some sort of global path planning algorithm.
### Search methods on PRM and path smoothing
#### videos
[![](http://img.youtube.com/vi/_ZvYOEbU1mI/0.jpg)](https://www.youtube.com/watch?v=_ZvYOEbU1mI)

The title bar shows the currently used search algorithm and state of path smoothening.
A circular agent is in a square room.
Its goal is to reach the other diagonal.
But there is an obstacle in the middle of the room.



#### images
- DFS

![](github/dfs.png =100x20)

- BFS

![](github/bfs.png)

- UCS

![](github/ucs.png)

- A\*

![](github/astar.png)

- weighted A\*

![](github/weighted_astart.png)

---
