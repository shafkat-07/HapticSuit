# Kinematics

This contains the kinematics used convert a force into 4 motor speeds and directions. 

## Derivation

This contains the workings out for how the equations were derived. We started first using a single motor, then computing how it would change if it were allowed to rotate. Next we looked at how 2 motors would behave, and then finally 4 motors. The final equations are shown below:

![4 Motors](../kinematics/derivation/final.png)

## Matlab

Next we include the code used to compute the inverse kinematics. To run this open it in matlab and click the green arrow.

### Plotter

Finally we include the code used to plot the force and motor combinations. To run this use the following:

```bash
python animtated_plot.py
```