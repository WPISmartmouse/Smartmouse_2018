# Profiling & Other Analysis

## How fast does our control loop need to be?

Control loop means the thing that computes desired wheel velocities, not the actual PID controller.

Let's assume we want to perform a 90 degree turn at 0.6 m/s. The radius of that turn is 0.09m, so the arc length is 2*pi*0.09/4 = 0.14137m. This means it will take 0.14137m / 0.6m/s = 0.2356s to complete a turn. If we want to have 100 control update over the course of this turn, we need our control loop to have a period of 0.002356s, which is 425Hz.

The current control loop speed is 100Hz (0.01s) and our current ArcTurn speed is 0.75 * 0.6 = 0.5 m/s. At that speed it takes 0.14137m / 0.5cm/s = 0.28274s to complete the turn. This means we are getting 0.28274/0.01 = 28.27 updates during the turn! That's not as bad as I thought...

If we ran at 1kHz and we want 100 updates, then we our turn time could be as short as 0.1 seconds, which means 0.14137/0.1 = 1.4137m/s!

### Our Goal

We estimate a 43 second solve will required the robot to drive at 90 cm/s (0.9m/s), and we want 100 updates during a 90 degree turn. 100 updates means our 90 degree arc would be 100 straight line segments, that's pretty accurate! Professor Onal also agreed that number is good enough. At 0.9 m/s, the turn would take 0.14137m / 0.9m/s = 0.15707s. Our control loop period must therefore be 0.0015707s, which is 636.6Hz. For sanity, let's round that up to 650Hz.

*Our target control loop rate is 650Hz*.

This doesn't necessarily answer the question of how fast our PID controller should be. We need to experiment to see if faster PID control loop rate gives better response time and steady state error.

### Profiling results

|Key      | Average time (micros)|
|---------|----------------------|
|Schedule |  31.571              |
|Sensors  |  79.741              |
|KC       | 186.953              |
|Motors   |   7.012              |

**Total Time: *305.277 us**

This means we could run is 3275.7Hz, or 3.275kHz?!?! But we're only running at 100Hz...

*This is just the some of these measured times, which does not include a small amount of other code. There could be another ~10us of total loop time.

