/*
 * Copyright (C) 2019 Eric Medvet <eric.medvet@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.units.erallab.hmsrobots;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.controllers.*;
import it.units.erallab.hmsrobots.objects.Ground;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.viewers.OnlineViewer;
import it.units.erallab.hmsrobots.objects.Voxel;
import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.objects.WorldObject;
import it.units.erallab.hmsrobots.objects.immutable.Snapshot;
import it.units.erallab.hmsrobots.util.TimeAccumulator;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.VideoFileWriter;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.Logger;
import it.units.erallab.hmsrobots.viewers.SnapshotListener;
import java.util.concurrent.atomic.AtomicLong;
import java.util.stream.Collectors;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;
import org.dyn4j.dynamics.World;
import org.dyn4j.geometry.Vector2;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class Starter {

  public static void main(String[] args) throws IOException {
    List<WorldObject> worldObjects = new ArrayList<>();
    Grid<Boolean> structure = Grid.create(7, 5, (x, y) -> (x < 2) || (x >= 5) || (y > 2));
    Voxel.Builder builder = Voxel.Builder.create()
            .springF(15d)
            .massLinearDamping(0.5d)
            .massAngularDamping(0.05d)
            .areaRatioOffset(0.2d)
            .massSideLengthRatio(0.3d)
            .ropeJointsFlag(false)
            .springScaffoldings(EnumSet.of(
                    Voxel.SpringScaffolding.SIDE_EXTERNAL,
                    Voxel.SpringScaffolding.SIDE_INTERNAL,
                    Voxel.SpringScaffolding.CENTRAL_CROSS
            ));
    //simple
    VoxelCompound vc1 = new VoxelCompound(10, 10, new VoxelCompound.Description(
            Grid.create(structure, b -> b ? builder : null),
            new TimeFunction(Grid.create(structure.getW(), structure.getH(), t -> {
              return (Math.sin(2d * Math.PI * t * 1d));
            }))
    ));
    //centralized mlp
    Grid<List<Pair<Voxel.Sensor, Integer>>> centralizedSensorGrid = Grid.create(structure.getW(), structure.getH(),
            (x, y) -> {
              if (!structure.get(x, y)) {
                return null;
              }
              List<Pair<Voxel.Sensor, Integer>> sensors = new ArrayList<>();
              if (y > 2) {
                sensors.add(Pair.of(Voxel.Sensor.Y_ROT_VELOCITY, 0));
                sensors.add(Pair.of(Voxel.Sensor.Y_ROT_VELOCITY, 1));
                sensors.add(Pair.of(Voxel.Sensor.X_ROT_VELOCITY, 0));
                sensors.add(Pair.of(Voxel.Sensor.X_ROT_VELOCITY, 1));
              }
              if (y == 0) {
                sensors.add(Pair.of(Voxel.Sensor.TOUCHING, 0));
              }
              sensors.add(Pair.of(Voxel.Sensor.AREA_RATIO, 0));
              return sensors;
            }
    );
    int[] innerNeurons = new int[]{10};
    int nOfWeights = CentralizedMLP.countParams(structure, centralizedSensorGrid, innerNeurons);
    double[] weights = new double[nOfWeights];
    Random random = new Random();
    for (int i = 0; i < weights.length; i++) {
      weights[i] = random.nextDouble()*2d-1d;
    }
    VoxelCompound vc2 = new VoxelCompound(10, 10, new VoxelCompound.Description(
            Grid.create(structure, b -> b ? builder : null),
            new CentralizedMLP(structure, centralizedSensorGrid, innerNeurons, weights, t -> 1d*Math.sin(-2d * Math.PI * t * 0.5d))
    ));
    //distributed mlp
    Grid<List<Pair<Voxel.Sensor, Integer>>> distributedSensorGrid = Grid.create(structure, b -> b ? Lists.newArrayList(
            Pair.of(Voxel.Sensor.X_ROT_VELOCITY, 0),
            Pair.of(Voxel.Sensor.Y_ROT_VELOCITY, 0),
            Pair.of(Voxel.Sensor.AREA_RATIO, 0),
            Pair.of(Voxel.Sensor.AREA_RATIO, 1),
            Pair.of(Voxel.Sensor.AREA_RATIO, 2),
            Pair.of(Voxel.Sensor.TOUCHING, 0)
    ) : null);
    innerNeurons = new int[] {4};
    nOfWeights = DistributedMLP.countParams(structure, distributedSensorGrid, 1, innerNeurons);
    weights = new double[nOfWeights];
    for (int i = 0; i < weights.length; i++) {
      weights[i] = random.nextDouble()*2d-1d;
    }
    VoxelCompound vc3 = new VoxelCompound(10, 10, new VoxelCompound.Description(
            Grid.create(structure, b -> b ? Voxel.Builder.create().massCollisionFlag(true) : null),
            new DistributedMLP(
                    structure,
                    Grid.create(structure.getW(), structure.getH(), (x, y) -> {
                      if (x == 300) {
                        return t -> Math.sin(-2d * Math.PI * t * 0.5d);
                      } else {
                        return t -> 0d;
                      }
                    }),
                    distributedSensorGrid,
                    1,
                    innerNeurons,
                    weights
            )
    ));
    //world
    Ground ground = new Ground(new double[]{0, 1, 2999, 3000}, new double[]{50, 0, 0, 50});
    //worldObjects.add(vc1);
    //vc2.translate(new Vector2(25, 0));
    //worldObjects.add(vc2);
    //vc3.translate(new Vector2(50, 0));
    worldObjects.add(vc3);
    worldObjects.add(ground);
    World world = new World();
    worldObjects.forEach((worldObject) -> {
      worldObject.addTo(world);
    });
    ScheduledExecutorService executor = Executors.newScheduledThreadPool(3);
    OnlineViewer viewer = new OnlineViewer(executor);
    viewer.start();
    final double dt = 0.015d;
    final TimeAccumulator t = new TimeAccumulator();
    final AtomicLong c = new AtomicLong(0);
    final int controlStepInterval = 1;
    Runnable runnable = () -> {
      try {
        t.add(dt);
        if (c.incrementAndGet() % (1 + controlStepInterval) == 0) {
          worldObjects.forEach((worldObject) -> {
            if (worldObject instanceof VoxelCompound) {
              ((VoxelCompound) worldObject).control(t.getT(), dt);
            }
          });
        }
        world.step(1);
        Snapshot snapshot = new Snapshot(t.getT(), worldObjects.stream().map(WorldObject::getSnapshot).collect(Collectors.toList()));;
        viewer.listen(snapshot);
      } catch (Throwable ex) {
        ex.printStackTrace();
        System.exit(0);
      }
    };
    executor.scheduleAtFixedRate(runnable, 0, Math.round(dt * 1000d / 1.1d), TimeUnit.MILLISECONDS);
  }

}
