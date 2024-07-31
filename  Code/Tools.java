package iBug;

import javax.vecmath.Point3d;
import simbad.sim.*;
import java.util.List;


public class Tools {

    // Returns the luminosity from the left sensor
    public static double getLeftSensorLum(MyRobot myRobot) {
        return Math.pow(myRobot.leftLightSensor.getLux(), 0.1); 
    }

    // Returns the luminosity from the right sensor
    public static double getRightSensorLum(MyRobot myRobot) {
        return Math.pow(myRobot.rightLightSensor.getLux(), 0.1); 
    }

    // Returns the luminosity from the center sensor
    public static double getCenterSensorLum(MyRobot myRobot) {
        return Math.pow(myRobot.centerLightSensor.getLux(), 0.1); 
    }

    // Returns the the point that hits the wall
    public static Point3d getSensedPoint(MyRobot myRobot, int sonar){
        double v;
        RangeSensorBelt bumpers = myRobot.bumpers;

        if (bumpers.hasHit(sonar))
            v = myRobot.getRadius() + bumpers.getMeasurement(sonar);
        else
            v = myRobot.getRadius() + bumpers.getMaxRange();

        double x = v * Math.cos(bumpers.getSensorAngle(sonar));
        double z = v * Math.sin(bumpers.getSensorAngle(sonar));

        return new Point3d(x, 0, z);
    }

    // Returns the input angle changed by pi
    public static double wrapToPi(double a){
        if (a > Math.PI)
            return a-Math.PI*2;
        if (a <=- Math.PI)
            return a + Math.PI*2;
        return a;
    }

    // Adds current luminosity to the list of luminosities
    public static List<Double> addLuminosity(MyRobot myRobot, List<Double> Luminosities) {
        double centerSensorLum = Tools.getCenterSensorLum(myRobot);
        Luminosities.add(centerSensorLum);
        return Luminosities;
    }

    // Returns the maximum element of a list of doubles 
    public static double getMaxFromList(List<Double> lums) {
        double max = 0.0;
        for (double lum : lums) {
            if (lum > max) {
                max = lum;
            }
        }
        return max;
    }

    // Returns the mean of the elements of a list of doubles
    public static double getMeanFromList(List<Double> lums) {
        double mean = 0.0;
        for (double lum : lums) {
            mean += lum;
        }
        mean /= lums.size();
        return mean;
    }

    // Returns the maximum between two doubles
    public static double getMaxBetween2Elements(double e1, double e2) {
        double max;
        if (e1 > e2) {
            max = e1;
        } else {
            max = e2;
        }
        return max;
    }

    // Returns whether or not the robot must stop circumnavigating
    public static boolean stopCircumNavigate(List<Double> luminosities, int N_FUTURE_ELEMENTS, int iter) {

        // Get the current size of luminosities
        int lumsSize = luminosities.size();  

        // Create 2 lists with the first 'lumsSize - N_FUTURE_ELEMENTS' and 'lumsSize - N_FUTURE_ELEMENTS - 1' elements respectively
        List<Double> pastLums1 = luminosities.subList(0, lumsSize - N_FUTURE_ELEMENTS - 1);
        List<Double> pastLums2 = luminosities.subList(0, lumsSize - N_FUTURE_ELEMENTS);

        // Create a list with the last 'N_FUTURE_ELEMENTS' elements
        List<Double> futureLums = luminosities.subList(lumsSize - N_FUTURE_ELEMENTS, lumsSize);

        // Get the maximum element from the 'pastLums1' and 'pastLums2' lists
        // Then get the maximum between the two.
        double pastLumsMax1 = getMaxFromList(pastLums1);
        double pastLumsMax2 = getMaxFromList(pastLums2);
        double pastLumsMax = getMaxBetween2Elements(pastLumsMax1, pastLumsMax2);

        // Get the mean of the 'futureLums' list
        double futureLumsMean = getMeanFromList(futureLums);

        // Stop circumnavigating if 'futureLumsMean' is less than 'pastLumsMax'
        if (futureLumsMean < pastLumsMax) {
            return true;
        } else {
            return false; 
        }
    }
}
