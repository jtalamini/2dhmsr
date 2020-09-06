# Soft Robots

> Soft robots may be able to perform tasks which are hard for rigid robots, thanks to their soft nature which gives them “infinite degrees of freedom”. 
> This potential comes at the cost of a larger design effort required by this robot, both in its controller (brain) and its shape (body). 
> For this reason, optimization, possibly by means of meta-heuristics, is an effective way to address the design of soft robots.
>
<cite>[Design, Validation, and Case Studies of 2D-VSR-Sim, an Optimization-friendly
Simulator of 2-D Voxel-based Soft Robots][1]</cite>

[1]: https://arxiv.org/pdf/2001.08617.pdf

# Installation

1. This project requires Java Development Kit (JDK) version 14.
Download the latest JDK from [here](https://www.oracle.com/java/technologies/javase-downloads.html).

Check the installed version of Java:

```
java --version
```

2. Download and install [IntelliJ IDEA IDE](https://www.jetbrains.com/idea/download).

3. From IntelliJ main menu navigate to: *File > New Project from Version Control > Git* and enter URL: https://github.com/jtalamini/2dhmsr

4. Make sure the latest JDK version is used in the project: *File > Project Structure > Project > Project SDK*

Note: The following code can be found in *src/main/java/it/units/erallab/hmsrobots/LabTeaching.java*.

# Voxel

<img src="/assets/voxel.png" alt="voxel" width="300"/>

Physical voxel: 
* 4 rigid bodies
* (up to) 18 Spring-Damping Systems (SDS) (= configurable scaffolding)
* (up to) 4 ropes (= upper bounds to the distance between rigid bodies)

Actuation mode:
- Force applied on the mass center along the direction connecting it to the voxel center. 
- Area actuation (default mode) = instantaneously change the voxel area:
  - Allows compression/expansion
  - Takes into account other forces acting on the voxel

Materials with different softness can be created by changing:
* Scaffolding (adding/removing SDSs)
* SDSs frequency (the higher is the spring oscillation frequncy, the softer is the voxel material)

Default voxel material:

```java
final ControllableVoxel defaultMaterial = new ControllableVoxel();
```

This defaultMaterial has low SDSs frequency `springF = 8d`, and `springScaffoldings = Voxel.SPRING_SCAFFOLDINGS`, that is all the scaffoldings are enabled.
In order to create a custom material, the desired values have to be provided in their respective constructor arguments.

Hard material example:
* High SDSs frequency
* All the springs scaffoldings are enabled

```java
final ControllableVoxel hardMaterialVoxel = new ControllableVoxel(
       Voxel.SIDE_LENGTH,
       Voxel.MASS_SIDE_LENGTH_RATIO,
       50d,  // high frequency
       Voxel.SPRING_D,
       Voxel.MASS_LINEAR_DAMPING,
       Voxel.MASS_ANGULAR_DAMPING,
       Voxel.FRICTION,
       Voxel.RESTITUTION,
       Voxel.MASS,
       Voxel.LIMIT_CONTRACTION_FLAG, // the ropes are enabled
       Voxel.MASS_COLLISION_FLAG,
       Voxel.AREA_RATIO_MAX_DELTA,
       Voxel.SPRING_SCAFFOLDINGS, // all the scaffolding enabled
       ControllableVoxel.MAX_FORCE,
       ControllableVoxel.ForceMethod.DISTANCE
);
```

Soft material example:
- Lower SDSs frequency
- Only some of the scaffoldings are enabled:
  - `SIDE_EXTERNAL` (<span style="color:blue">blue</span> SDSs in figure)
  - `CENTRAL_CROSS` (<span style="color:orange">orange</span> SDSs in figure)

```java
final ControllableVoxel softMaterialVoxel = new ControllableVoxel(
       Voxel.SIDE_LENGTH,
       Voxel.MASS_SIDE_LENGTH_RATIO,
       5d, // low frequency
       Voxel.SPRING_D,
       Voxel.MASS_LINEAR_DAMPING,
       Voxel.MASS_ANGULAR_DAMPING,
       Voxel.FRICTION,
       Voxel.RESTITUTION,
       Voxel.MASS,
       Voxel.LIMIT_CONTRACTION_FLAG, // the ropes are enabled
       Voxel.MASS_COLLISION_FLAG,
       Voxel.AREA_RATIO_MAX_DELTA,
       EnumSet.of(Voxel.SpringScaffolding.SIDE_EXTERNAL,
       Voxel.SpringScaffolding.CENTRAL_CROSS), // scaffolding partially enabled
       ControllableVoxel.MAX_FORCE,
       ControllableVoxel.ForceMethod.DISTANCE
);
```

# Robot

Robots = Body + Mind
- Body = some voxels assembly that can be actuated:
  - Non-sensing body
  - Sensing body
- Mind = some controller responsible for actuating its body:
  - Centralized mind
  - Distributed mind

## Body

This is a **7x4** grid of voxel, used to create the following "biped" robot:

<img src="/assets/robot.png" alt="robot" width="300"/>

The `Grid.create()` method allows to create a 2D grid by using some filler function:

```java
public static <K> Grid<K> create(int w, int h, BiFunction<Integer, Integer, K> fillerFunction)
```

This allows to create a grid of boolean values called *structure*.
The structure is then used to create the robot body, by instantiating voxels only in the desired positions:

```java
int w = 7;
int h = 4;

final Grid<Boolean> structure = Grid.create(w, h, (x, y) -> (x < 2) || (x > 5) || (y > 0));
```
### Non-sensing Body

The following body is created according to the structure, using 2 different voxel materials:

```java
Grid<ControllableVoxel> body = Grid.create(structure.getW(), structure.getH(), (x, y) -> {
   if (structure.get(x, y)) {
       if ((y == 3) && (x < 6) && (x > 0)) {
           return SerializationUtils.clone(hardMaterialVoxel);
       } else {
           return SerializationUtils.clone(softMaterialVoxel);
       }
   } else {
       return null; // no voxel is placed here
   }
});
```

### Sensing Body

It is also possible to create a robot with the ability to sense the environment.
The sensing equipment for each voxel can be selected among a wide range of sensors.

In this example all the voxels have an `AreaRatio` sensor, and the voxels in the bottom layer of the body grid have also a `Touch` sensor:

```java
Grid<SensingVoxel> sensingBody = Grid.create(w, h, (x, y) -> {
   if (structure.get(x, y)) {
       if (y == 0) {
           return new SensingVoxel(List.of(new AreaRatio(), new Touch()));
       } else {
           return new SensingVoxel(List.of(new AreaRatio()));
       }
   } else {
       return null; // no voxel is placed here
   }
});
```

Sensors:
* `AreaRatio`: returns the ratio between the current area and rest area of its belonging voxel.
* `Touch`: return true if its belonging voxel is touching another object (which is not part of the robot), otherwise return false.
* ...

## Mind

### Simple Mind

The mind is an implementation of the `Controller` interface.
A simple mind, based on a non-sensng body, can be defined using the `TimeFunction` constructor:

```java
public TimeFunctions(Grid<SerializableFunction<Double, Double>> functions)
```

In this simple mind a different function of time is applied to each voxel.
Specifically the mind is a sine function with a different phase for each voxel:

```java
Controller<ControllableVoxel> mind = new TimeFunctions(
       Grid.create(w, h, (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * ((double) x / (double) w)))
);
```

`TimeFunction` has a public `control()` method, which is called by the simulator at each time step, and that applies to each voxel the corresponding signal:

```java
public void control(double t, Grid<? extends ControllableVoxel> voxels) {
   Iterator var4 = voxels.iterator();
   while(var4.hasNext()) {
       Entry<? extends ControllableVoxel> entry = (Entry)var4.next();
       SerializableFunction<Double, Double> function = (SerializableFunction)this.functions.get(entry.getX(), entry.getY());
       if (entry.getValue() != null && function != null) {
           ((ControllableVoxel)entry.getValue()).applyForce((Double)function.apply(t));
       }
   }
}
```

Building the robot:

```java
Robot<ControllableVoxel> robot = new Robot<>(mind, body);
```

### Centralized Mind

A centralized mind is a function that accepts inputs from all the voxels, and actuates them all in a centralized fashion.
The `CentralizedSensing` object stores inputs, outputs, and the controller function to invoke.
Specifically, the function to invoke is a `MultiLayerPerceptron` (MLP) with ReLU activation function, and no hidden layers:

```java
CentralizedSensing<SensingVoxel> centralizedMind = new CentralizedSensing<>(SerializationUtils.clone(sensingBody));

MultiLayerPerceptron mlp = new MultiLayerPerceptron(
       MultiLayerPerceptron.ActivationFunction.RELU,
       centralizedMind.nOfInputs(),
       new int[0], // hidden layers
       centralizedMind.nOfOutputs()
);
```

The MLP parameters are randomly sampled from a gaussian distribution, and finally we set the MLP as the controller function:

```java
double[] ws = mlp.getParams();
Random random = new Random();
IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextGaussian());
mlp.setParams(ws);
centralizedMind.setFunction(mlp);
```

Building the robot:

```java
Robot<SensingVoxel> centralizedRobot = new Robot<>(centralizedMind, SerializationUtils.clone(sensingBody));
```

### Distributed Mind

A distributed mind is a grid of functions, each of them accepting inputs from part of the body, and actuating it in a distributed fashion.
The `DistributedSensing` object that stores inputs, outputs, and the grid of functions to invoke.

```java
DistributedSensing distributedMind = new DistributedSensing(SerializationUtils.clone(sensingBody), 1);
```

In this example each function is associated to a voxel of the body.
A MLP is defined for each voxel, with parameters randomly sampled from a gaussian distribution:

```java
for (Grid.Entry<SensingVoxel> entry : sensingBody) {
   MultiLayerPerceptron localMlp = new MultiLayerPerceptron(
           MultiLayerPerceptron.ActivationFunction.RELU,
           distributedMind.nOfInputs(entry.getX(), entry.getY()),
           new int[0], // hidden layers
           distributedMind.nOfOutputs(entry.getX(), entry.getY())
   );
   double[] localWs = mlp.getParams();
   IntStream.range(0, localWs.length).forEach(i -> localWs[i] = random.nextGaussian());
   localMlp.setParams(localWs);
   distributedMind.getFunctions().set(entry.getX(), entry.getY(), localMlp);
}
```

Building the robot:

```java
Robot<SensingVoxel> distributedRobot = new Robot<>(distributedMind, SerializationUtils.clone(sensingBody));
```

## Task

A locomotion task is defined by the following constructor:

```java
public Locomotion(double finalT, double[][] groundProfile, List<Locomotion.Metric> metrics, Settings settings) {
    this(finalT, groundProfile, groundProfile[0][1] + 1.0D, metrics, settings);
}
```

This constructor requires the user to specify:
* `double finalT`: simulation length (i.e. 20 simulated seconds)
* `double[][] groundProfile`: defined by 2D coordinates
* `List<Locomotion.Metric> metrics`: which metrics to collect (i.e. TRAVELED_X_DISTANCE)
* `Settings settings`: the physical settings for the world initialization

```java
final Locomotion locomotion = new Locomotion(
    20,
    Locomotion.createTerrain("flat"),
    Lists.newArrayList(
        Locomotion.Metric.TRAVELED_X_DISTANCE
    ),
    new Settings() // default settings
);
```

Running a simulation with the given task, and printing the collected metrics:

```java
locomotion.apply(robot).stream().forEach(System.out::println);
```

## Visualization

It is possible to show the simulation as it is executed,through a `GridOnlineViewer`:

```java
ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(2);
ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
GridOnlineViewer gridOnlineViewer = new GridOnlineViewer(
       Grid.create(1, 1, "Simulation"),
       uiExecutor
);
gridOnlineViewer.start(5); // delay from beginning of the simulation
```

And then an `GridEpisodeRunner` is used to run the simulations:

```
GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
       Grid.create(1, 1, Pair.of("Robot", robot)),
       locomotion,
       gridOnlineViewer,
       executor
);
runner.run();
```
