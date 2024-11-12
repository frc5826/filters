package com.avisautomata;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class GroundTruthGenerator {

    private static final double TIME_STEPS = 0.01;
    public static final String EXTENSION = ".moments";
    public static final Path DIRECTORY = Paths.get(  "test", "src", "test", "resources");
    private static final ObjectMapper MAPPER = new ObjectMapper();

    public static List<RobotMoment> get(String filename) throws IOException {
        Path path = getPath(filename);
        return MAPPER.readValue(new FileReader(path.toFile()), new TypeReference<List<RobotMoment>>() {});
    }

    public record Moment(double time, double value, double velocity, double acceleration) {}

    public record RobotMoment(double time, Moment x, Moment y, Moment h) {}

    public static void main(String[] args) throws IOException {
        long start = System.currentTimeMillis();
        System.out.println("Saving xBackAndForth");
        saveTest(xBackAndForth(), "x-back-forth");
        System.out.println("Saving yBackAndForth");
        saveTest(yBackAndForth(), "y-back-forth");
        System.out.println("Saving hBackAndForth");
        saveTest(hBackAndForth(), "h-back-forth");
        System.out.println("Saving allBackAndForth");
        saveTest(allBackAndForth(), "all-back-forth");
        System.out.println("Completed in " + (System.currentTimeMillis() - start) + "ms");
    }

    private static Path getPath(String filename){
        return Paths.get(DIRECTORY.toString(), filename + EXTENSION);
    }

    private static void saveTest(List<RobotMoment> t, String filename) throws IOException {
        Path path = getPath(filename);
        System.out.println(path.toAbsolutePath());

        MAPPER.writeValue(new FileWriter(path.toFile()), t);

        if(!t.equals(MAPPER.readValue(new FileReader(path.toFile()), new TypeReference<List<RobotMoment>>() {}))){
            throw new IllegalStateException("Unable to verify " + filename + " was written to disk correctly.");
        }
    }


    //Straight line and back
    //Start at 0,0,0 while moving +1 m/s in the x direction.
    //After 4 seconds, accelerate at -1 m/ss for 2 seconds.
    //Continue moving at our current velocity for another 4 seconds.
    public static List<RobotMoment> xBackAndForth(){
        List<RobotMoment> moments = new LinkedList<>();
        double timeStep = TIME_STEPS;

        //Move towards +x at 1m/s for 4 seconds. Continue at 1 m/s for 4 seconds.
        moments.addAll(zip(
                move(timeStep, 0, 0, 1, 0, 4),
                move(timeStep, 0, 0, 0, 0, 4),
                move(timeStep, 0, 0, 0, 0, 4)
        ));
        RobotMoment last = moments.get(moments.size() - 1);

        //Decelerate for 2 secs. At 1 sec we should be stopped, and 2 sec we should be moving at -1.
        moments.addAll(zip(
                move(timeStep, 4, last.x.value, last.x.velocity, -1, 2),
                move(timeStep, 4, last.y.value, last.y.velocity, last.y.acceleration, 2),
                move(timeStep, 4, last.h.value, last.h.velocity, last.h.acceleration, 2)
        ));
        last = moments.get(moments.size() - 1);

        //Continue on towards -x for 4 sec
        moments.addAll(zip(
                move(timeStep, 6, last.x.value, last.x.velocity, 0, 4),
                move(timeStep, 6, last.y.value, last.y.velocity, last.y.acceleration, 4),
                move(timeStep, 6, last.h.value, last.h.velocity, last.h.acceleration, 4)
        ));

        return moments;
    }

    //Same as xBackAndForth, just in the y direction
    public static List<RobotMoment> yBackAndForth(){
        List<RobotMoment> moments = new LinkedList<>();
        double timeStep = TIME_STEPS;

        //Move towards +x at 1m/s for 4 seconds. Continue at 1 m/s for 4 seconds.
        moments.addAll(zip(
                move(timeStep, 0, 0, 0, 0, 4),
                move(timeStep, 0, 0, 1, 0, 4),
                move(timeStep, 0, 0, 0, 0, 4)
        ));
        RobotMoment last = moments.get(moments.size() - 1);

        //Decelerate for 2 secs. At 1 sec we should be stopped, and 2 sec we should be moving at -1.
        moments.addAll(zip(
                move(timeStep, 4, last.x.value, last.x.velocity, last.x.acceleration, 2),
                move(timeStep, 4, last.y.value, last.y.velocity, -1, 2),
                move(timeStep, 4, last.h.value, last.h.velocity, last.h.acceleration, 2)
        ));
        last = moments.get(moments.size() - 1);

        //Continue on towards -x for 4 sec
        moments.addAll(zip(
                move(timeStep, 6, last.x.value, last.x.velocity, last.x.acceleration, 4),
                move(timeStep, 6, last.y.value, last.y.velocity, 0, 4),
                move(timeStep, 6, last.h.value, last.h.velocity, last.h.acceleration, 4)
        ));

        return moments;
    }

    //Change the heading
    //Start spinning at  pi / 2 rads / sec for 4 seconds
    //Accelerate -pi / 2 rads / sec for 2 seconds
    //Continue at that speed for 4 more seconds
    public static List<RobotMoment> hBackAndForth(){
        List<RobotMoment> moments = new LinkedList<>();
        double timeStep = TIME_STEPS;

        //Move towards +x at 1m/s for 4 seconds. Continue at 1 m/s for 4 seconds.
        moments.addAll(zip(
                move(timeStep, 0, 0, 0, 0, 4),
                move(timeStep, 0, 0, 0, 0, 4),
                move(timeStep, 0, 0, Math.PI / 2, 0, 4)
        ));
        RobotMoment last = moments.get(moments.size() - 1);

        //Decelerate for 2 secs. At 1 sec we should be stopped, and 2 sec we should be moving at -1.
        moments.addAll(zip(
                move(timeStep, 4, last.x.value, last.x.velocity, last.x.acceleration, 2),
                move(timeStep, 4, last.y.value, last.y.velocity, last.y.acceleration, 2),
                move(timeStep, 4, last.h.value, last.h.velocity, -Math.PI / 2, 2)
        ));
        last = moments.get(moments.size() - 1);

        //Continue on towards -x for 4 sec
        moments.addAll(zip(
                move(timeStep, 6, last.x.value, last.x.velocity, last.x.acceleration, 4),
                move(timeStep, 6, last.y.value, last.y.velocity, last.y.acceleration, 4),
                move(timeStep, 6, last.h.value, last.h.velocity, 0, 4)
        ));

        return moments;
    }

    //A combination of all the back and forth methods
    public static List<RobotMoment> allBackAndForth(){
        List<RobotMoment> moments = new LinkedList<>();
        double timeStep = TIME_STEPS;

        //Move towards +x at 1m/s for 4 seconds. Continue at 1 m/s for 4 seconds.
        moments.addAll(zip(
                move(timeStep, 0, 0, 1, 0, 4),
                move(timeStep, 0, 0, 1, 0, 4),
                move(timeStep, 0, 0, Math.PI / 2, 0, 4)
        ));
        RobotMoment last = moments.get(moments.size() - 1);

        //Decelerate for 2 secs. At 1 sec we should be stopped, and 2 sec we should be moving at -1.
        moments.addAll(zip(
                move(timeStep, 4, last.x.value, last.x.velocity, -1, 2),
                move(timeStep, 4, last.y.value, last.y.velocity, -1, 2),
                move(timeStep, 4, last.h.value, last.h.velocity, -Math.PI / 2, 2)
        ));
        last = moments.get(moments.size() - 1);

        //Continue on towards -x for 4 sec
        moments.addAll(zip(
                move(timeStep, 6, last.x.value, last.x.velocity, 0, 4),
                move(timeStep, 6, last.y.value, last.y.velocity, 0, 4),
                move(timeStep, 6, last.h.value, last.h.velocity, 0, 4)
        ));

        return moments;
    }

    public static Map<String, List<RobotMoment>> load() throws IOException {
        return Files.list(DIRECTORY).filter(p -> p.getFileName().toString().endsWith(EXTENSION)).collect(Collectors.toMap(p -> p.getFileName().toString().replace(EXTENSION, ""), p -> {
            try {
                return MAPPER.readValue(new FileReader(p.toFile()), new TypeReference<List<RobotMoment>>() {});
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }));
    }


    public static List<RobotMoment> zip(List<Moment> x, List<Moment> y, List<Moment> h){
        if(x.size() != y.size() || y.size() != h.size()){
            throw new IllegalArgumentException("List sizes differ");
        }
        return IntStream.range(0, x.size()).mapToObj(i -> new RobotMoment(x.get(i).time, x.get(i), y.get(i), h.get(i))).toList();
    }


    public static List<Moment> move(double timeStep, double initialTime, double initialValue, double initialVelocity, double initialAcceleration, double duration) {
        List<Moment> moments = new LinkedList<>();

        double currentTime = initialTime;
        double endTime = initialTime + duration;

        double currentValue = initialValue;
        double currentVelocity = initialVelocity;
        double currentAcceleration = initialAcceleration;

        while(Math.abs(currentTime - endTime) > timeStep){
            currentTime += timeStep;
            currentValue += (currentVelocity * timeStep) + (0.5 * currentAcceleration * Math.pow(timeStep, 2));
            currentVelocity += currentAcceleration * timeStep;
            moments.add(new Moment(currentTime, currentValue, currentVelocity, currentAcceleration));
        }

        return moments;
    }



}
