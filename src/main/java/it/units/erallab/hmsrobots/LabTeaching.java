package it.units.erallab.hmsrobots;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.*;
import it.units.erallab.hmsrobots.core.objects.*;
import it.units.erallab.hmsrobots.core.sensors.AreaRatio;
import it.units.erallab.hmsrobots.core.sensors.Touch;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.stream.IntStream;

public class LabTeaching {
    public static void main(String[] args) {

        /*
        ##################
        ##     VOXEL    ##
        ##################
        */

        // voxel made of the default material
        final ControllableVoxel defaultMaterial = new ControllableVoxel();

        // voxel made of the hard material
        final ControllableVoxel hardMaterial = new ControllableVoxel(
                Voxel.SIDE_LENGTH,
                Voxel.MASS_SIDE_LENGTH_RATIO,
                50d, // high frequency
                Voxel.SPRING_D,
                Voxel.MASS_LINEAR_DAMPING,
                Voxel.MASS_ANGULAR_DAMPING,
                Voxel.FRICTION,
                Voxel.RESTITUTION,
                Voxel.MASS,
                Voxel.LIMIT_CONTRACTION_FLAG,
                Voxel.MASS_COLLISION_FLAG,
                Voxel.AREA_RATIO_MAX_DELTA,
                Voxel.SPRING_SCAFFOLDINGS, // all the scaffolding enabled
                ControllableVoxel.MAX_FORCE,
                ControllableVoxel.ForceMethod.DISTANCE
        );

        // voxel made of the soft material
        final ControllableVoxel softMaterial = new ControllableVoxel(
                Voxel.SIDE_LENGTH,
                Voxel.MASS_SIDE_LENGTH_RATIO,
                5d, // low frequency
                Voxel.SPRING_D,
                Voxel.MASS_LINEAR_DAMPING,
                Voxel.MASS_ANGULAR_DAMPING,
                Voxel.FRICTION,
                Voxel.RESTITUTION,
                Voxel.MASS,
                Voxel.LIMIT_CONTRACTION_FLAG,
                Voxel.MASS_COLLISION_FLAG,
                Voxel.AREA_RATIO_MAX_DELTA,
                EnumSet.of(Voxel.SpringScaffolding.SIDE_EXTERNAL, Voxel.SpringScaffolding.CENTRAL_CROSS), // scaffolding partially enabled
                ControllableVoxel.MAX_FORCE,
                ControllableVoxel.ForceMethod.DISTANCE
        );

        /*
        ##################
        ##     BODY     ##
        ##################
        */

        int w = 20;
        int h = 5;

        // the body structure
        final Grid<Boolean> structure = Grid.create(w, h, (x, y) -> true);

        // non-sensing body made of 2 materials
        Grid<ControllableVoxel> body = Grid.create(structure.getW(), structure.getH(), (x, y) -> {
           if (structure.get(x, y)) {
               if (y == 0) {
                   return SerializationUtils.clone(hardMaterial);
               } else {
                   return SerializationUtils.clone(softMaterial);
               }
           } else {
               return null;
           }
        });


        // sensing body with AreaRatio and Touch sensors, default material employed here
        Grid<SensingVoxel> sensingBody = Grid.create(w, h, (x, y) -> {
            if (structure.get(x, y)) {
                if (y == 0) {
                    return new SensingVoxel(List.of(new AreaRatio(), new Touch()));
                } else {
                    return new SensingVoxel(List.of(new AreaRatio()));
                }
            } else {
                return null;
            }
        });


        /*
        ##################
        ##     BRAIN    ##
        ##################
        */

        // simple brain
        Controller<ControllableVoxel> brain = new TimeFunctions(
                Grid.create(w, h, (x, y) -> (Double t) -> Math.sin(-2 * Math.PI * t + Math.PI * ((double) x / (double) w)))
        );

        // building a robot with simple brain and non-sensing body
        Robot<ControllableVoxel> robot = new Robot<>(brain, body);


        // centralized brain
        // create an object holding inputs, outputs, and the controller function
        CentralizedSensing centralizedBrain = new CentralizedSensing(SerializationUtils.clone(sensingBody));
        // build the MLP architecture
        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                MultiLayerPerceptron.ActivationFunction.RELU,
                centralizedBrain.nOfInputs(),
                new int[0], // hidden layers
                centralizedBrain.nOfOutputs()
        );
        double[] ws = mlp.getParams();
        Random random = new Random();
        // randomly sample the parameters of the MLP from a gaussian distribution
        IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextGaussian());
        // set the MLP parameters
        mlp.setParams(ws);
        // set the MLP as controller function
        centralizedBrain.setFunction(mlp);

        // building a robot with centralized brain and sensing body
        Robot<SensingVoxel> centralizedRobot = new Robot(centralizedBrain, SerializationUtils.clone(sensingBody));


        // distributed brain
        DistributedSensing distributedBrain = new DistributedSensing(SerializationUtils.clone(sensingBody), 1);
        // iterate over each voxel
        for (Grid.Entry<SensingVoxel> entry : sensingBody) {
            // build a MLP for each voxel
            MultiLayerPerceptron localMlp = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.RELU,
                    distributedBrain.nOfInputs(entry.getX(), entry.getY()),
                    new int[0], // hidden layers
                    distributedBrain.nOfOutputs(entry.getX(), entry.getY())
            );
            //  randomly sample the parameters of each MLP from a gaussian distribution
            double[] localWs = mlp.getParams();
            IntStream.range(0, localWs.length).forEach(i -> localWs[i] = random.nextGaussian());
            localMlp.setParams(localWs);
            // update the distributed controller with each MLP
            distributedBrain.getFunctions().set(entry.getX(), entry.getY(), localMlp);
        }

        // building a robot with distributed brain and sensing body
        Robot<SensingVoxel> distributedRobot = new Robot<>(distributedBrain, SerializationUtils.clone(sensingBody));

        /*
        ##################
        ##     TASK     ##
        ##################
        */

        // define a locomotion task
        final Locomotion locomotion = new Locomotion(
                60,
                Locomotion.createTerrain("flat"),
                Lists.newArrayList(Locomotion.Metric.TRAVELED_X_DISTANCE),
                new Settings()
        );

        // running the simulator without GUI
        // locomotion.apply(robot).forEach(System.out::println);

        /*
        ##################
        ##     VIEW     ##
        ##################
        */

        // handle the threads:
        ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(2);
        ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
        // configure the viewer
        GridOnlineViewer gridOnlineViewer = new GridOnlineViewer(
                Grid.create(1, 1, "Simulation"),
                uiExecutor,
                GraphicsDrawer.build().setConfigurable("drawers", List.of(
                        it.units.erallab.hmsrobots.viewers.drawers.Robot.build(),
                        it.units.erallab.hmsrobots.viewers.drawers.Voxel.build(),
                        it.units.erallab.hmsrobots.viewers.drawers.Ground.build()
                ))
        );
        // set the delay from the simulation to the viewer
        gridOnlineViewer.start(0);
        // run the simulations
        GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
                Grid.create(1, 1, Pair.of("Robot", robot)),
                locomotion,
                gridOnlineViewer,
                executor
        );
        runner.run();
    }
}
