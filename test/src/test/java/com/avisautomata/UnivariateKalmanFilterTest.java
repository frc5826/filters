package com.avisautomata;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.LinkedList;
import java.util.List;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.assertTrue;
public class UnivariateKalmanFilterTest {

    private static final int seed = 0;
    private static final double measureVariance = 0.01;
    private static final double movementVariance = 0.01;
    private static List<GroundTruthGenerator.Moment> truth;
    private static ApacheRandom random;

    private record Measurement(double time, double position, double distance){}

    @BeforeAll
    protected static void loadGroundTruth() {
        List<GroundTruthGenerator.RobotMoment> moments = GroundTruthGenerator.xBackAndForth();
        truth = moments.stream().map(GroundTruthGenerator.RobotMoment::x).toList();
    }

    @BeforeEach
    protected void resetRandom(){
        random = new ApacheRandom();
        random.setSeed(seed);
    }

    private static Double getTruthForTime(double time){
        for(GroundTruthGenerator.Moment moment : truth){
            if(Math.abs(moment.time() - time) < Double.MIN_NORMAL){
                return moment.value();
            }
        }
        throw new RuntimeException("Unable to get truth for time " + time);
    }

    private static List<Measurement> getMeasurements() {
        double measureStd = Math.sqrt(measureVariance);
        double moveStd = Math.sqrt(movementVariance);

        List<Double> positions = truth.stream().map(GroundTruthGenerator.Moment::value).map(pos -> new NormalDistribution(random, pos, measureStd).sample()).toList();
        List<Double> times = truth.stream().map(GroundTruthGenerator.Moment::time).toList();
        List<Double> distances = new LinkedList<>();

        for(int i = 0; i < truth.size(); i++){
            double distance = 0;
            if(i > 0) {
                distance = truth.get(i).value() - truth.get(i - 1).value();
            }
            distances.add(new NormalDistribution(random, distance, moveStd).sample());
        }


        return IntStream.range(0, truth.size()).mapToObj(i -> new Measurement(times.get(i), positions.get(i), distances.get(i))).toList();
    }

    @Test
    public void testSimpleKalmanFilter() {
        double warmup = 1.0;
        double threshold = 0.25;

        UnivariateKalmanFilter filter = new UnivariateKalmanFilter();

        for(Measurement m : getMeasurements()){
            filter.move(new NormalDistribution(m.distance, movementVariance));
            filter.measure(new NormalDistribution(m.position, measureVariance));

            double truth = getTruthForTime(m.time);
            double diff = Math.abs(truth - filter.getMean());

            System.out.println("Time: " + m.time);
            System.out.println("Estimate: " + filter.getMean());
            System.out.println("Truth: " + truth);
            System.out.println("Variance: " + filter.getVariance());
            System.out.println("Error: " + diff);
            System.out.println();

            if(m.time > warmup) {
                assertTrue(diff < threshold, "Last iteration failed. " + diff + " >= " + threshold);
            }
        }
    }



}
