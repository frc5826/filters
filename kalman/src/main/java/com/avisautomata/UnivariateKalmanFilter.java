package com.avisautomata;

import org.apache.commons.math3.distribution.NormalDistribution;

public class UnivariateKalmanFilter {

    private NormalDistribution estimate;

    public UnivariateKalmanFilter(NormalDistribution initial){
        this.estimate = initial;
    }

    public UnivariateKalmanFilter(){
        this(new NormalDistribution());
    }

    public void measure(NormalDistribution measurement){
        double estimateMean = estimate.getMean();
        double estimateVar = estimate.getNumericalVariance();

        double measurementMean = measurement.getMean();
        double measurementVar = measurement.getNumericalVariance();

        double residual = measurementMean - estimateMean;
        double kalmanGain = estimateVar / (estimateVar + measurementVar);

        estimateMean += (kalmanGain * residual);
        estimateVar = estimateVar * (1 - kalmanGain);

        estimate = new NormalDistribution(estimateMean, Math.sqrt(estimateVar));
    }

    public void move(NormalDistribution movement){
        double estimateMean = estimate.getMean();
        double estimateVar = estimate.getNumericalVariance();

        double movementMean = movement.getMean();
        double movementVar = movement.getNumericalVariance();

        double mean = estimateMean + movementMean;
        double variance = estimateVar + movementVar;

        estimate = new NormalDistribution(mean, Math.sqrt(variance));
    }

    public NormalDistribution getEstimate(){
        return estimate;
    }

    public double getVariance(){
        return estimate.getNumericalVariance();
    }

    public double getMean() {
        return estimate.getNumericalMean();
    }


}
