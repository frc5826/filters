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
        //TODO #1 - Go through the equations in Chapter 4 to fill this out.
    }

    public void move(NormalDistribution movement){
        //TODO #2 - Go through the equations in Chapter 4 to fill this out.
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
