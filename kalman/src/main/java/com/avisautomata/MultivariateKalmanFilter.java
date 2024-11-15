package com.avisautomata;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class MultivariateKalmanFilter {

    private RealVector x;
    private RealMatrix P;

    //Used if we don't supply a B and u vector
    private RealMatrix defaultB;
    private RealVector defaultU;

    private MultivariateKalmanFilter(MultivariateNormalDistribution initial){
        this.x = MatrixUtils.createRealVector(initial.getMeans());
        this.P = initial.getCovariances();

        this.defaultB = MatrixUtils.createRealIdentityMatrix(this.x.getDimension());
        this.defaultU = MatrixUtils.createRealVector(new double[this.x.getDimension()]);
    }

    public MultivariateKalmanFilter(int dim) {
        this(new MultivariateNormalDistribution(new double[dim], defaultCovar(dim)));
    }

    public MultivariateKalmanFilter(RealVector initial) {
        this(new MultivariateNormalDistribution(initial.toArray(), defaultCovar(initial.getDimension())));
    }

    private static double[][] defaultCovar(int dim){
        double[][] output = new double[dim][dim];
        for(int i = 0; i < dim; i++){
            for(int j = 0; j < dim; j++){
                output[i][j] = i == j ? 1 : 0;
            }
        }
        return output;
    }

    public void move(RealMatrix F, RealMatrix B, RealVector u, RealMatrix Q){
        //TODO #4 - Go through the equations in Chapter 6 to fill this out.
    }

    public void move(RealMatrix F, RealMatrix Q){
        this.move(F, defaultB, defaultU, Q);
    }

    public void measure(RealMatrix H, RealMatrix R, RealVector z){
        //TODO #5 - Go through the equations in Chapter 6 to fill this out.
    }

    public MultivariateNormalDistribution getEstimate(){
        return new MultivariateNormalDistribution(x.toArray(), P.getData());
    }

}
