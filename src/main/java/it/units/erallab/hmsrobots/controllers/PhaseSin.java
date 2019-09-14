/*
 * Copyright (C) 2019 eric
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
package it.units.erallab.hmsrobots.controllers;

import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.objects.Voxel;
import java.util.function.Function;

/**
 *
 * @author eric
 */
public class PhaseSin extends TimeFunction {
  
  public PhaseSin(double frequency, double amplitude, Grid<Double> phases) {
    super(getFunctions(frequency, amplitude, phases));
  }
  
  private static Grid<Function<Double,Double>> getFunctions(final double frequency, final double amplitude, final Grid<Double> phases) {
    Grid<Function<Double,Double>> functions = Grid.create(phases);
    for (int x = 0; x<functions.getW(); x++) {
      for (int y = 0; y<functions.getH(); y++) {
        final double phase = phases.get(x, y);
        functions.set(x, y, t -> Math.sin(2d*Math.PI*frequency*t+phase)*amplitude);
      }
    }
    return functions;
  }
  
}