# Profiling & Other Analysis

How fast does our control loop need to be? Here control loop means the thing that computes desired wheel velocities, not the actual PID controller.
Let's assume we want to perform a 90 degree turn at 0.6 m/s. The radius of that turn is 0.09m, so the arc length is 2*pi*0.09/4 = 0.14137m. This means it will take 0.14137m / 0.6m/s = 0.2356s to complete a turn. If we want to have 100 control update over the course of this turn, we need our control loop to have a period of 0.002356s, which is 425Hz.

The current control loop speed is 100Hz (0.01s) and our current ArcTurn speed is 0.75 * 0.6 = 0.5 m/s. At that speed it takes 0.14137m / 0.5cm/s = 0.28274s to complete the turn. This means we are getting 0.28274/0.01 = 28.27 updates during the turn! That's not that bad.

If we ran at 1kHz and we want 100 updates, then we our turn time could be as short as 0.1 seconds, which means 0.14137/0.1 = 1.4137m/s!
