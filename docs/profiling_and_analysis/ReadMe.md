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

|Key   | Average time (micros)|
|---------|----------------------|
|Schedule | 31.571       |
|Sensors | 79.741       |
|KC    | 186.953       |
|Motors  |  7.012       |

**Total Time: †305.277 us**

This means we could run at 1,0,000/305.277 = 3275.7Hz, or 3.275kHz?!?! But we're only running at 100Hz.

†This is just the some of these measured times, which does not include a small amount of other code. There could be another ~10us of total loop time.


## How fast do our sensors need to be?

To get 650Hz, they mathematically need to take less than 1.5ms to read. Next we account for 200us of KC + 32us of scheduling + 7us of motors + 10us of other stuff. This sums to 249us. **Which means our sensors actually should take less than 1.2ms to read.**

## how high resolution do our encoders need to be?

Let's set our constraints. We want to estimate our wheel velocity every 1.5ms (which we do because 650Hz). We want to be able to measure between 0.2m/s and 1m/s (0.2 is the will then be the slowest we are able to go), with less than 5% error.

If we read the encoder every 1.5ms, and we are traveling 0.2m/s, given wheel radius of 0.0145m, then our rotational speed is 0.2m/1s * 1/(2pi*.0145)m = 2.195 rotations per second. If we want to measure this 650 times a second, that's 2.195rot/1s * 1s/650cycle=0.00337 rot/cycle. Here a cycle refers to a control loop update. If we have a 10 bit resolution encoder, then we have 0.00337rotations/1cycle* 2^10counts/1rotations = 3counts/cycle. This is not enough, because being off by one count would mean 33% error in velocity estimate. Essentially, we want 20counts/cycle so that we have 5% error. Of course this error is less problematic at higher speeds (more counts). But to acheive 5% error at 20m/s we need 0.00337*2^n=20, so n=12.535, which rounds up to 13. We can then see what happens at 1m/s. At 1m/s we go 1/1*1/(2pi*0.0145)=10.9762rotations/s, then 10.9762rot/s*1s/650cycle=0.0168865rot/cycle, then 0.0168865rot/cycle*2^13count/1rotation=138.33count/cycle.

So, we need a 13 bit encoder on the output shaft if we want to be able to measure speeds within .2 and 1m/s at 650Hz with at most 5% error.

Lets consider what would happen if we put it on the input shaft. We have a gear ratio of 50:1 (the input shaft spins 50 times faster than the output shaft). Solve the following for n: 0.00337rotations (output)/1cycle * 50 rotations(input)/1rotation(output) * n counts/1rotations = 20counts/cycle. If you solve this, n=119.

This means we need an encoder with 119 CPR or more.

Lets figure out how many interrupts will happen per second at 1m/s. 119counts/1revolution * 10.9762rotation/1second = 1.306kHz (interrupts per second). If each interrupts take XXXXX microseconds, then ...

## Profiling encoder counts

In order to decide whether we should do our encoder counting on the teensy or another chip, we need to count how much CPU encoder counting eats up. We do this by comparing the amount of time a blocking piece of code takes when motors are moving versus stationary. When the motor is stationary, the encoders will not generate any interrupts. When they are moving, the encoder will generate lots of interrupts and the same for loop will take up more time.

Run the `profile_encoders` program in `real/main` to generate these numbers:

microseconds per count = 0.575us

So, if we want use 119CPR encoders at 0.9m/s how long per control loop will it take?

0.9m/s * 1rot/(2*pi*0.0145)m * 1s/650cycle * 50 rot inner/1rot * 119count/1rot inner * 0.575us / 1count = 0.107us per cycle

In other words, it will take 100 *nanoseconds* per control loop to count our encoders at 90cm/s with 119 CPR encoders on the input shaft.

[here's the dimensional analysis in a more readable form](https://latex.codecogs.com/gif.latex?\LARGE&space;\frac{0.9m}{1s}&space;*&space;\frac{1\text{&space;outer&space;shaft&space;rotation}}{2\pi*0.0145m}&space;*&space;\frac{1s}{650\text{&space;cycle}}&space;*&space;\frac{50\text{&space;inner&space;shaft&space;rotation}}{1\text{&space;outer&space;shaft&space;rotation}}&space;*&space;\frac{119\text{&space;counts}}{1\text{&space;inner&space;shaft&space;rotation}}&space;*&space;\frac{0.575us}{1\text{&space;count}}&space;=&space;\frac{0.107us}{\text{cycle}})

In other words, yes we can count encoder interrupts on the teensy and it won't matter at all.

## Other requirements we haven't figured out

 - how many rangefinders do we need
 - what is the worst case accuracy we can tolerate. How accurate do our range finders need to be.
