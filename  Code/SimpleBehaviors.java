package iBug;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import simbad.sim.RangeSensorBelt;
import java.util.ArrayList;
import java.util.List;


public class SimpleBehaviors {
    static double K1 = 0.8;  // controls the rotational velocity in 'circumNavigate'
    static double K2 = 0.1;  // controls the translational velocity in 'circumNavigate'
    static double K3 = 13;  // controls the 'phRot' angle in 'circumNavigate'
    static double K4 = 0.5;  // controls the rotational velocity in 'rotatetoGoal'
    static double K5 = 0.5;  // controls the translational velocity in 'movetoGoal'
    static double SAFETY = 0.45;  // safety factor: controls the distance from the walls
    static double EPS_LEFT_RIGHT = 1e-3;  // threshold for the absolute difference between left and right luminosities
    static double EPS_HAS_REACHED = 1e-3;  // threshold that determines when the robot has reached under the light source
    static int N_FUTURE_ELEMENTS = 5;  // Number of elements to check in the future (used in 'moveToGoal')
    static boolean rotationIsComplete;  // determines when the rotation towards the goal phase is complete
    static List<Double> luminosities = new ArrayList<>();  // Initialize a list where the luminosities will be stored, 
                                                           // when the robot starts circumnavigating an obstacle
    

    // ********************** 1. Stops the robot **********************
    public static void stop(MyRobot myRobot, int iter) {
        myRobot.setTranslationalVelocity(0);
        myRobot.setRotationalVelocity(0);
    }


    // ********************** 2. Rotates the robot towards the goal **********************
    public static boolean rotateToGoal(MyRobot myRobot, int iter) {

        // Get sensor luminosities
        double leftSensorLum = Tools.getLeftSensorLum(myRobot);
        double rightSensorLum = Tools.getRightSensorLum(myRobot);
        double centerSensorLum = Tools.getCenterSensorLum(myRobot);

        // Get the average between left and right luminosities
        double leftRightMean = (leftSensorLum + rightSensorLum) / 2.0;

        // Get the absolute difference between left and right luminosities
        double leftRightAbsDiff = Math.abs(leftSensorLum - rightSensorLum);

        // Set rotational velocity
        if ((leftRightMean <= centerSensorLum) || (leftRightAbsDiff > EPS_LEFT_RIGHT)){
            if (rightSensorLum > leftSensorLum) {
                myRobot.setRotationalVelocity(-K4);
            } else {
                myRobot.setRotationalVelocity(K4);
            } 
        } else {
            myRobot.setRotationalVelocity(0);
            rotationIsComplete = true;
        }

        return rotationIsComplete;
    }


    // ********************** 3. Makes the robot move towards the goal **********************
    public static iBug.robotState moveToGoal(MyRobot myRobot, iBug.robotState state, int iter){   
        
        // Order the robot to move forward
        myRobot.setTranslationalVelocity(K5);

        // Declare the bumpers variable
        RangeSensorBelt bumpers = myRobot.bumpers;

        // Get sensor luminosities
        double leftSensorLum = Tools.getLeftSensorLum(myRobot);
        double rightSensorLum = Tools.getRightSensorLum(myRobot);
        double centerSensorLum = Tools.getCenterSensorLum(myRobot);

        // Get the absolute difference between left and right luminosities
        double leftRightAbsDiff = Math.abs(leftSensorLum - rightSensorLum);

        // When the 'leftRightAbsDiff' variable has become lower than a threshold,
        // given by 'EPS_LEFT_RIGHT', the robot has lost track of the goal
        if (leftRightAbsDiff > EPS_LEFT_RIGHT) {
            // Then, stop the robot from moving forward and set the state in 'RotatetoGoal'
            myRobot.setTranslationalVelocity(0);
            state = iBug.robotState.RotateToGoal;

            // When rotation is complete, change robot's state in 'MoveToGoal'
            rotationIsComplete = SimpleBehaviors.rotateToGoal(myRobot, iter);
            if (rotationIsComplete) {
                state = iBug.robotState.MoveToGoal;
            } 
        }

        // Obstacle found... when on of the bumpers has hit a wall
        if (bumpers.oneHasHit()) {

            // Add current luminosity in the list of luminosities
            luminosities = Tools.addLuminosity(myRobot, luminosities);

            // 'stopCircumNavigate' can be called properly only when this condition is True
            if (luminosities.size() > N_FUTURE_ELEMENTS + 1) {
                // 'stopCN' determines whether or not to stop circumnavigating
                boolean stopCN = Tools.stopCircumNavigate(luminosities, N_FUTURE_ELEMENTS, iter);
                if (!stopCN) {
                    state = iBug.robotState.CircumNavigate;
                }
            } else {
                state = iBug.robotState.CircumNavigate;
            }
        }

        // Stop the robot when it has reached under the light source
        if (Math.abs(centerSensorLum - myRobot.maxLuminosity) < EPS_HAS_REACHED) {
            state = iBug.robotState.Stop;
        }

        return state;
    }


    // ********************** 4. Makes the robot circumnavigate the current obstacle found **********************
    public static iBug.robotState circumNavigate(MyRobot myRobot, boolean CLOCKWISE, iBug.robotState state, int iter){

        // Declare the bumpers variable
        RangeSensorBelt bumpers = myRobot.bumpers;

        // Find the bumper index with the minimum distance from the wall
        int min_idx = 0;
        for (int i = 1; i < bumpers.getNumSensors(); i++)
            if (bumpers.getMeasurement(i) < bumpers.getMeasurement(min_idx))
            min_idx = i;

        // Get the hit point and find its ditance from the robot center
        Point3d hitPoint = Tools.getSensedPoint(myRobot, min_idx);
        double d = hitPoint.distance(new Point3d(0, 0, 0));  

        // 'v' determines whether or not to rotate clockwise or counter-clock wise
        Vector3d v = CLOCKWISE? new Vector3d(-hitPoint.z, 0, hitPoint.x): new Vector3d(hitPoint.z, 0, -hitPoint.x);

        // Find phLin, phRot and phRef
        double phLin = Math.atan2(v.z, v.x);
        double phRot = Math.atan(K3 * (d - SAFETY));
        if (CLOCKWISE)
            phRot = -phRot;
        double phRef = Tools.wrapToPi(phLin + phRot); 
        
        // Set the rotational and translational velocities
        myRobot.setRotationalVelocity(K1 * phRef);
        myRobot.setTranslationalVelocity(K2 * Math.cos(phRef));    
        
        // ----------------------- My own addition to the original code -----------------------

        // Add current luminosity in the list of luminosities
        luminosities = Tools.addLuminosity(myRobot, luminosities);

        // 'stopCircumNavigate' can be called properly only when this condition is True
        if (luminosities.size() > N_FUTURE_ELEMENTS + 1) {

            // 'stopCN' and 'leftBiggerThanRight' will determine whether or
            // not to stop circumnavigating and get in 'MoveToGoal' mode
            boolean stopCN = Tools.stopCircumNavigate(luminosities, N_FUTURE_ELEMENTS, iter);
            boolean leftBiggerThanRight = Tools.getRightSensorLum(myRobot) < Tools.getLeftSensorLum(myRobot);
            if (stopCN && !leftBiggerThanRight) {
                state = iBug.robotState.MoveToGoal;
            }
        }

        return state;
    }
}
