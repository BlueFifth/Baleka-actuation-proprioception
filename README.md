# Baleka-actuation-proprioception
Repository containing data and MATLAB files used for tuning of Baleka 2.5's legs

This corresponds to the methods shown in the attached paper (Force Control and Proprioception.pdf), however the data is different from the paper as the tuning was re-done when new motor drivers were installed.

In brief, the steps are as follows to replicate the experiment (assuming you have the same hardware and setup):

1. Attach Half-leka to the boom, and set up the Axia8-m20 sensor as a force plate. Make sure Half-leka is on the floor (not the force plate!) in its rest position, i.e. as crouched as it can go, with both upper links in contact with the lower links.

2. Ensure that the motor input blocks in ComplianceSpeedgoat.slx have their tuning removed. This means go to Half-leka hardware -> Baleka Leg Control and ensure unity gain on the torque commands.
 
3. Start running ComplianceSpeedgoat.slx

After 4s, the robot should stand up to ~30cm leg length. If safe to do so, press the green button. This will raise the robot further to a setpoint of 45cm leg length. Once the robot is raised, it is ready to be placed on the force plate.

4. Place the robot on the force plate/sensor. Try your best to make sure the foot is touching the middle of the plate/sensor. Push down on the robot (should all hardware be set up correctly - especially back emf protection - this will be safe). Vary the strength and direction of your pushes, but make sure that the leg doesn't lose contact with the plate - if it does then you'll need to do more data processing. Once you've collected enough data take the robot off the plate and let it stand on the ground. Press the green button to collapse the robot to a low crouch. Now you can stop the program and cut power. Ensure you have saved and/or exported your data.

5. Repeat steps 3 & 4 with varying spring constants until satisfied.

6. Run TuningActuation.mlx, making any adjustments for your data (filename, ordering of fields, etc.)

7. In ComplianceSpeedgoat.slx edit the torque scaling in Half-leka hardware -> Baleka Leg Control to reflect the actuation scaling found in step 6. 

8. Repeat experiments detailed in steps 3-5 with the actuation scaling.

9. Run TuningProprioception.mlx making any adjustments for your data (filename, ordering of fields, etc.)

