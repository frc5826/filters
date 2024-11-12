package com.avisautomata;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.Random;

public class ApacheRandom extends Random implements RandomGenerator {

    @Override
    public void setSeed(int seed) {
        super.setSeed(seed);
    }

    @Override
    public void setSeed(int[] seed) {
        throw new UnsupportedOperationException();
    }
}
