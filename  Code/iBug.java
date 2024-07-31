package iBug;


public class iBug {

    // Set robot's states
    public enum robotState {
        RotateToGoal, MoveToGoal, CircumNavigate, Stop
    }

    // Declare the appropriate variables
    robotState state;
    MyRobot myRobot;   
    boolean CLOCKWISE = false;
    boolean rotationIsComplete;  

    // Initialize the robot object and its state 
    public iBug(MyRobot myRobot) {
        this.myRobot = myRobot;
        state = robotState.RotateToGoal;
    }

    // Implements one step of the 'iBug' algorithm
    public void step(int iter) {
        
        // 1. Rotate towards light
        if (state == robotState.RotateToGoal) {
            rotationIsComplete = SimpleBehaviors.rotateToGoal(myRobot, iter);
            if (rotationIsComplete) {
                state = robotState.MoveToGoal;
            }      
        }      

        // 2. Move towards light
        if (state == robotState.MoveToGoal) {
            state = SimpleBehaviors.moveToGoal(myRobot, state, iter);
        }

        // 3. Circumnavigate
        if (state == robotState.CircumNavigate) {
            state = SimpleBehaviors.circumNavigate(myRobot, CLOCKWISE, state, iter);
        }
        
        // 4. Stop
        if (state == robotState.Stop) {
            myRobot.setTranslationalVelocity(0);
            myRobot.setRotationalVelocity(0);
        }
    }

}
