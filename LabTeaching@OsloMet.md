# Soft Robots

> Soft robots may be able to perform tasks which are hard for rigid robots, thanks to their soft nature which gives them “infinite degrees of freedom”. 
> This potential comes at the cost of a larger design effort required by this robot, both in its controller (brain) and its shape (body). 
> For this reason, optimization, possibly by means of meta-heuristics, is an effective way to address the design of soft robots.
>
<cite>[Design, Validation, and Case Studies of 2D-VSR-Sim, an Optimization-friendly
Simulator of 2-D Voxel-based Soft Robots][1]</cite>

[1]: https://arxiv.org/pdf/2001.08617.pdf

# Installation
1. Clone this repository:

```
git clone https://github.com/jtalamini/2dhmsr
```

2. Install JRE:

```
sudo apt install openjdk-9-jre
```

(Optional)
Download and install [IntelliJ IDEA IDE](https://www.jetbrains.com/idea/download).

# Voxel

![2D voxel](/assets/images/tux.png)

Scaffoldings: 
* 4 rigid bodies
* (up to) 18 spring-damping systems (SDS)
* 2 ropes (= upper bounds to the distance between 2 rigid bodies)

Actuation modes:
- Force applied on the mass center along the direction connecting it to the voxel center. 
- Area actuation (default mode) = instantaneously change the voxel area:
  - Allows compression/expansion
  - Takes into account other forces acting on the voxel

Materials:

Material with different softness can be created by changing:
* Scaffolding
* SDS frequency

Default voxel material:

```java
final ControllableVoxel defaultMaterial = new ControllableVoxel();
```

This defaultMaterial has low SDS frequency (8.0), and all the scaffoldings enabled.
If we want to create custom materials we have to provide the desired parameters in the constructor.

Hard material example:
* High SDS frequency
* All the springs scaffoldings are enabled

```java
final ControllableVoxel hardMaterialVoxel = new ControllableVoxel(
       Voxel.SIDE_LENGTH,
       Voxel.MASS_SIDE_LENGTH_RATIO,
       50d,                                // high frequency
       Voxel.SPRING_D,
       Voxel.MASS_LINEAR_DAMPING,
       Voxel.MASS_ANGULAR_DAMPING,
       Voxel.FRICTION,
       Voxel.RESTITUTION,
       Voxel.MASS,
       Voxel.LIMIT_CONTRACTION_FLAG,
       Voxel.MASS_COLLISION_FLAG,
       Voxel.AREA_RATIO_MAX_DELTA,
       Voxel.SPRING_SCAFFOLDINGS,           // all the scaffoldings enabled
       ControllableVoxel.MAX_FORCE,
       ControllableVoxel.ForceMethod.DISTANCE
);
```

Soft material example:
- Lower SDS frequency
- Only some of the scaffoldings are enabled:
  - SIDE_EXTERNAL= <span style="color:blue">blue</span> SDS in figure
  - CENTRAL_CROSS = <span style="color:orange">orange</span> SDS in figure

```java
final ControllableVoxel softMaterialVoxel = new ControllableVoxel(
       Voxel.SIDE_LENGTH,
       Voxel.MASS_SIDE_LENGTH_RATIO,
       5d,                                       // low frequency
       Voxel.SPRING_D,
       Voxel.MASS_LINEAR_DAMPING,
       Voxel.MASS_ANGULAR_DAMPING,
       Voxel.FRICTION,
       Voxel.RESTITUTION,
       Voxel.MASS,
       Voxel.LIMIT_CONTRACTION_FLAG,
       Voxel.MASS_COLLISION_FLAG,
       Voxel.AREA_RATIO_MAX_DELTA,
       EnumSet.of(Voxel.SpringScaffolding.SIDE_EXTERNAL,
       Voxel.SpringScaffolding.CENTRAL_CROSS),    // some scaffoldings enabled
       ControllableVoxel.MAX_FORCE,
       ControllableVoxel.ForceMethod.DISTANCE
);
```

# Robot

Robots:
* Body = voxels assembly; it can be actuated, and possibly sense the environment
* Mind = controller that is responsible for actuating the voxels

## Body

Here we define a 2D **7x4** grid of voxels, and we create a biped robot like this:

![Robot body](/assets/images/robot.png)

The `Grid.create()` method allows to create a 2D grid with a custom filling function:

```
public static <K> Grid<K> create(int w, int h, BiFunction<Integer, Integer, K> fillerFunction)
```

We use this method to create a grid of booleans called structure:

```java
int w = 7;
int h = 4;

final Grid<Boolean> structure = Grid.create(w, h, (x, y) -> (x < 2) || (x > 5) || (y > 0));
```
### Non-sensing Body

We create the robot body according to the structure, using different materials for the voxels:

```java
Grid<ControllableVoxel> body = Grid.create(structure.getW(), structure.getH(), (x, y) -> {
   if (structure.get(x, y)) {
       if ((y == 3) && (x < 6) && (x >0)) {
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

It is possible to create a robot with the ability to sense the environment.
The sensing equipment for each voxel can be selected among a wide range of sensors.

In this example all the voxels have an `AreaRatio` sensor, and the voxels of the bottom layer have also a `Touch` sensor:

```java
Grid<SensingVoxel> sensingBody = Grid.create(w, h, (x, y) -> {
   if (structure.get(x, y)) {
       if (y == 0) {
           return new SensingVoxel(List.of(new AreaRatio(), new Touch()));
       } else {
           return new SensingVoxel(List.of(new AreaRatio()));
       }
   } else {
       return null;         // no voxel is placed here
   }
});
```

Sensors:
* `AreaRatio`: returns the ratio between the belonging voxel current area and rest area.
* `Touch`: indicates if the voxel is touching another object or not
* ...

## Mind

### Non-sensing Mind

We define a robot mind as an implementation of the `Controller` interface.
To do this we use the `TimeFunction` class, with its constructor:

```java
public TimeFunctions(Grid<SerializableFunction<Double, Double>> functions)
```

The controller is a different function of the time applied to each voxel.
Specifically we consider a sine function with a different phase for each voxel:

```java
Controller<ControllableVoxel> mind = new TimeFunctions(
       Grid.create(w, h, (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * ((double) x / (double) w)))
);
```

`TimeFunction` has a public `control()` method, which is called by the simulator at each time step, and this applies to each voxel its corresponding signal:

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

Here we consider a centralized controller, which is a controller that collects the inputs from all the voxels, and actuates them in a centralized fashion.
First we create a `CentralizedSensing` object that stores inputs, outputs, and the controller function.
Then we We build the `MultiLayerPerceptron` objectm with ReLU activation function, and no hidden layer:

```java
CentralizedSensing<SensingVoxel> centralizedMind = new CentralizedSensing<>(SerializationUtils.clone(sensingBody));

MultiLayerPerceptron mlp = new MultiLayerPerceptron(
       MultiLayerPerceptron.ActivationFunction.RELU,
       centralizedMind.nOfInputs(),
       new int[0],                  // hidden layers size
       centralizedMind.nOfOutputs()
);
```

Then we randomly sample the params of the MLP from a gaussian distribution, and finally we set the MLP as the controller function:

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

Here we consider a centralized controller, which is a controller that collects the inputs from each voxel individually, and actuates each one according to the inputs and its params.
First we create a `DistributedSensing` object that stores inputs, outputs, and the controller function.

```java
DistributedSensing distributedMind = new DistributedSensing(SerializationUtils.clone(sensingBody), 1);
```

Then we loop over each voxel, building a MLP for each of them, and sampling their params like we did for the centralized controller:

```java
for (Grid.Entry<SensingVoxel> entry : sensingBody) {
   // create a MLP for each voxel
   MultiLayerPerceptron localMlp = new MultiLayerPerceptron(
           MultiLayerPerceptron.ActivationFunction.RELU,
           distributedMind.nOfInputs(entry.getX(), entry.getY()),
           new int[0], // hidden layers size
           distributedMind.nOfOutputs(entry.getX(), entry.getY())
   );
   // set random params for each MLP
   double[] localWs = mlp.getParams();
   IntStream.range(0, localWs.length).forEach(i -> localWs[i] = random.nextGaussian());
   localMlp.setParams(localWs);
   // update the distributed controller with the local MLP
   distributedMind.getFunctions().set(entry.getX(), entry.getY(), localMlp);
}
```

Building the robot:

```java
Robot<SensingVoxel> distributedRobot = new Robot<>(distributedMind, SerializationUtils.clone(sensingBody));
```

## Task

We consider a locomotion task with the following constructor:

```java
public Locomotion(double finalT, double[][] groundProfile, List<Locomotion.Metric> metrics, Settings settings) {
    this(finalT, groundProfile, groundProfile[0][1] + 1.0D, metrics, settings);
}
```

This requires us to specify:
* Simulation length (i.e. 20 simulated seconds)
* Type of terrain to create (i.e. flat ground)
* The metrics to collect (i.e. TRAVELED_X_DISTANCE)
* The settings for the world initialization (we keep the default ones)

```java
final Locomotion locomotion = new Locomotion(
    20,
    Locomotion.createTerrain("flat"),
    Lists.newArrayList(
        Locomotion.Metric.TRAVELED_X_DISTANCE
    ),
    new Settings()
);
```

Running a simulation with the given task, and printing the collected metrics:

```java
locomotion.apply(robot).stream().forEach(System.out::println);
```

## Visualization

```java
// this runs 2 threads: 
ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(2);
ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
// this configure the viewer
GridOnlineViewer gridOnlineViewer = new GridOnlineViewer(
       Grid.create(1, 1, "Simulation"),
       uiExecutor
);
// this sets the delay from the simulation to the viewer
gridOnlineViewer.start(5);
GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
       Grid.create(1, 1, Pair.of("Robot", robot)),
       locomotion,
       gridOnlineViewer,
       executor
);
runner.run();
```



