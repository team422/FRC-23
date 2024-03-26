package frc.lib.utils;

import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.fitting.SimpleCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

public class SinRegressionSolver {

  public static void main(String[] args) {
    // Define the model
    ParametricUnivariateFunction timeVaryingSine = new ParametricUnivariateFunction() {
      public double value(double t, double... parameters) {
        // y = (A + kt) * sin(Bt + C) + D
        double A = parameters[0]; // Amplitude
        double k = parameters[1]; // Rate of change of Amplitude
        double B = parameters[2]; // Frequency
        double C = parameters[3]; // Phase
        double D = parameters[4]; // Baseline

        return (A + k * t) * Math.sin(B * t + C) + D;
      }

      public double[] gradient(double t, double... parameters) {
        // Gradient computation for the optimizer
        double A = parameters[0];
        double k = parameters[1];
        double B = parameters[2];
        double C = parameters[3];

        double dFdA = Math.sin(B * t + C);
        double dFdk = t * Math.sin(B * t + C);
        double dFdB = (A + k * t) * t * Math.cos(B * t + C);
        double dFdC = (A + k * t) * Math.cos(B * t + C);
        double dFdD = 1;

        return new double[] { dFdA, dFdk, dFdB, dFdC, dFdD };
      }
    };

    WeightedObservedPoints points = new WeightedObservedPoints();
    // Add your data points here: points.add(x, y);

    // Create the fitter
    SimpleCurveFitter fitter = SimpleCurveFitter.create(timeVaryingSine, new double[] { 1.0, .1, 2.0, 0.0, 0.0 }); // Initial parameter guesses

    // Perform the fitting
    double[] bestFit = fitter.fit(points.toList());

    // Output the results
    System.out.println("Best fit parameters:");
    for (double param : bestFit) {
      System.out.println(param);
    }
  }
}
