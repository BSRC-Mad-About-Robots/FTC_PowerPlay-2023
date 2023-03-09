package org.firstinspires.ftc.teamcode.drive.opmode;

import java.util.Vector;

public class State {

    public void intake(

    ){
        State[] Dependency ={};
    }
    public void DeliveryToX2(){}
    public void DeliveryToX4(){}
    public void DeliveryToY3(){}
    public void DeliveryToW3(){}
    public void SSR(){}
    public void Park(){}
    public void initialPos(){}
    public void IDposition(){}
    public void addDependentState(State S)
    {
        dependentStates.add(S);
    }

    Vector<State> dependentStates = new Vector<State>();
}
