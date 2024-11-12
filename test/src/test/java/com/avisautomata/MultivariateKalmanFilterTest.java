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
        //Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        return MatrixUtils.createRealIdentityMatrix(3);
    }

    private static RealVector createZ(GroundTruthGenerator.Moment measurement){
        //Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        return MatrixUtils.createRealVector(new double[]{1, 1, 1});
    }

    @Test
    public void testPosVeloAccel() throws IOException {
        double warmup = 1.0;
        double threshold = 0.25;

        List<GroundTruthGenerator.Moment> truths = GroundTruthGenerator.xBackAndForth().stream().map(GroundTruthGenerator.RobotMoment::x).toList();
        List<GroundTruthGenerator.Moment> measurements = createMeasurements(truths);

        //Create required matrix/vectors
        //Go through the equations in Chapter 6 to fill this out. The answer below is not correct.
        RealVector x = MatrixUtils.createRealVector(new double[]{1, 1, 1});
        RealMatrix Q = MatrixUtils.createRealIdentityMatrix(3);
        RealMatrix H = MatrixUtils.createRealIdentityMatrix(3);
        RealMatrix R = MatrixUtils.createRealIdentityMatrix(3);

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

}
