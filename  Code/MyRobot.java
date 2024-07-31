package iBug;

import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import simbad.sim.*;


public class MyRobot extends Agent {
    double maxLuminosity = 0.799569;  // luminosity of the center sensor under the light source
    Point3d goal;
    RangeSensorBelt bumpers; 
    LightSensor leftLightSensor;
    LightSensor rightLightSensor;
    LightSensor centerLightSensor;
    iBug iBugAlgo;
    int iter;


    public MyRobot(Vector3d position, String name) {
        super(position, name);

        // Set the light sensors of the robot
        leftLightSensor = RobotFactory.addLightSensor(this, new Vector3d(0.6, 0.47, -0.6), 0, "left");
        rightLightSensor = RobotFactory.addLightSensor(this, new Vector3d(0.6, 0.47, 0.6), 0, "right");
        centerLightSensor = RobotFactory.addLightSensor(this, new Vector3d(0, 0.47, 0), 0, "center");

        // Set the bumpers of the robot
        bumpers = RobotFactory.addBumperBeltSensor(this, 24);

        // Create an 'iBug'-algorithm object
        iBugAlgo = new iBug(this);
    }

    public void initBehavior() {
        // Initialize 'iter', so as to keep track
        // of the number of iterations 
        iter = 0; 
    }

    public void performBehavior() {
        // Increase number of iterations, 
        // each time 'performBehavior' is called
        iter++;

        // Implement one step of the 'iBug' algorithm
        iBugAlgo.step(iter);        
    }
}