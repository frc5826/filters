package com.avisautomata;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class MultivariateKalmanFilterTest {

    private static final int seed = 0;
    private static final double measureVariance = 0.01;
    private static final double movementVariance = 0.01;
    private static ApacheRandom random;

    @BeforeEach
    protected void resetRandom(){
        random = new ApacheRandom();
        random.setSeed(seed);
    }

    private List<GroundTruthGenerator.Moment> createMeasurements(List<GroundTruthGenerator.Moment> truth) {
        return truth.stream().map(m -> new GroundTruthGenerator.Moment(
            m.time(),
            new NormalDistribution(m.value(), measureVariance).sample(),
            new NormalDistribution(m.velocity(), measureVariance).sample(),
            new NormalDistribution(m.acceleration(), measureVariance).sample()
        )).toList();
    }

    private static RealMatrix createF(double deltaT){
        //#6 - Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        return MatrixUtils.createRealMatrix(new double[][]{{1, deltaT}, {1, 0}});
    }

    private static RealVector createZ(GroundTruthGenerator.Moment measurement){
        // #7 - Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        return MatrixUtils.createRealVector(new double[]{measurement.value(), measurement.velocity()});
    }

    //TODO #9 - Right click on the method below and "run". If it works the test will pass.
    @Test
    public void testPosVeloAccel() throws IOException {
        double warmup = 1.0;
        double threshold = 0.25;

        List<GroundTruthGenerator.Moment> truths = GroundTruthGenerator.xBackAndForth().stream().map(GroundTruthGenerator.RobotMoment::x).toList();
        List<GroundTruthGenerator.Moment> measurements = createMeasurements(truths);

        //Create required matrix/vectors
        //TODO #8 - Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        RealVector x = MatrixUtils.createRealVector(new double[]{1, 1});
        RealMatrix Q = MatrixUtils.createRealMatrix(2, 2);
        //TODO white noise
        RealMatrix H = MatrixUtils.createRowRealMatrix(new double[]{1, 1});
        RealMatrix R = MatrixUtils.createRealMatrix(new double[][]{{0.1, 0}, {0, 0.1}});

        MultivariateKalmanFilter filter = new MultivariateKalmanFilter(x);

        double lastTime = 0;
        for(int i = 0; i < measurements.size(); i++){
            GroundTruthGenerator.Moment truth = truths.get(i);
            GroundTruthGenerator.Moment measurement = measurements.get(i);

            filter.move(createF(truth.time() - lastTime), Q);
            filter.measure(H, R, createZ(measurement));

            MultivariateNormalDistribution estimate = filter.getEstimate();

            System.out.println("Means: " + Arrays.toString(estimate.getMeans()));
            System.out.println("Truth: " + truth);
            System.out.println();

            if(truth.time() > warmup){
                assertTrue(Math.abs(estimate.getMeans()[0] - truth.value()) < threshold, "Position was off by more than " + threshold + " at " + truth.time());
                assertTrue(Math.abs(estimate.getMeans()[1] - truth.velocity()) < threshold, "Velocity was off by more than " + threshold + " at " + truth.time());
            }

            lastTime = truth.time();
        }
    }

    private record HZ(RealMatrix H, RealVector z){}

    private record VariableMeasurement(Double time, Double position, Double velocity, Double acceleration){}

    public HZ createVariableZAndH(Double position, Double velocity, Double acceleration){
        //TODO #10 - Go through the equations in Chapter 8, section "Sensor fusion: Different Data Rates" to fill this out. The answer below is not correct.
        return new HZ(MatrixUtils.createRealIdentityMatrix(3), MatrixUtils.createRealVector(new double[]{0, 0, 0}));
    }

    public GroundTruthGenerator.Moment closest(double time, List<GroundTruthGenerator.Moment> moments){
        double diff = Double.MAX_VALUE;
        int bestIdx = 0;

        for(int i = 0; i < moments.size(); i++){
            GroundTruthGenerator.Moment m = moments.get(i);
            if(Math.abs(m.time() - time) < diff){
                diff = Math.abs(m.time() - time);
                bestIdx = i;
            }
        }

        return moments.get(bestIdx);
    }

    //TODO #11 - Right click on the method below and "run". If it works the test will pass.
    @Test
    public void testVariableMeasurements(){
        double warmup = 1.0;
        double threshold = 0.25;

        //This test simulates getting measurements for position, velocity, and acceleration at different times and rates.

        List<GroundTruthGenerator.Moment> truths = GroundTruthGenerator.xBackAndForth().stream().map(GroundTruthGenerator.RobotMoment::x).toList();

        List<VariableMeasurement> measurements = createVariableMeasurements(truths);

        RealVector x = MatrixUtils.createRealVector(new double[]{0, 0, 0});
        MultivariateKalmanFilter filter = new MultivariateKalmanFilter(x);

        RealMatrix Q = MatrixUtils.createRealMatrix(new double[][]{{movementVariance, 0, 0}, {0, movementVariance, 0}, {0, 0, movementVariance}});
        RealMatrix R = MatrixUtils.createRealMatrix(new double[][]{{measureVariance, 0, 0}, {0, measureVariance, 0}, {0, 0, measureVariance}});

        double lastTime = 0;
        for(VariableMeasurement m : measurements){
            filter.move(createF(m.time - lastTime), Q);

            HZ hz = createVariableZAndH(m.position, m.velocity, m.acceleration);
            filter.measure(hz.H, R, hz.z);

            if(m.time() > warmup){
                GroundTruthGenerator.Moment truth = closest(m.time, truths);
                MultivariateNormalDistribution estimate = filter.getEstimate();

                double pDiff = Math.abs(estimate.getMeans()[0] - truth.value());
                assertTrue(pDiff < threshold, "Position was off by more than " + threshold + " at " + truth.time());

                double vDiff = Math.abs(estimate.getMeans()[1] - truth.velocity());
                assertTrue(vDiff < threshold, "Velocity was off by more than " + threshold + " at " + truth.time());
            }

            lastTime = m.time;
        }
    }

    private List<VariableMeasurement> createVariableMeasurements(List<GroundTruthGenerator.Moment> truths) {
        List<VariableMeasurement> measurements = new LinkedList<>();
        GroundTruthGenerator.Moment initial = truths.getFirst();

        measurements.add(new VariableMeasurement(initial.time(), initial.value(), initial.velocity(), initial.acceleration()));

        for(int i = 1; i < truths.size(); i++){
            double lastTime = truths.get(i - 1).time();
            GroundTruthGenerator.Moment current = truths.get(i);
            double timeDelta = current.time() - lastTime;

            int count = 0;

            //Position - Treat this like camera inputs. We get them infrequently.
            if(random.nextDouble() > .9){
                double t = lastTime + (timeDelta * random.nextDouble());
                double position = new NormalDistribution(current.value(), measureVariance).sample();
                measurements.add(new VariableMeasurement(t, position, null, null));
            }

            //Velocity - A frequent update.
            if(random.nextDouble() > .1){
                double t = lastTime + (timeDelta * random.nextDouble());
                double velocity = new NormalDistribution(current.velocity(), measureVariance).sample();
                measurements.add(new VariableMeasurement(t, null, velocity, null));
            }

            //Acceleration - A frequent update.
            if(random.nextDouble() > .25){
                double t = lastTime + (timeDelta * random.nextDouble());
                double acceleration = new NormalDistribution(current.acceleration(), measureVariance).sample();
                measurements.add(new VariableMeasurement(t, null, null, acceleration));
            }
        }


        return measurements.stream().sorted(Comparator.comparing(VariableMeasurement::time)).toList();
    }

}
