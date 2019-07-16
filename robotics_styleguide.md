This is a short styleguide for writing code that runs on robots.

A few notes:
* I use "user" to mean "user of your library", not "end user of the robot". Of course, the perfect robot has no end-user ;)

# Project Planning
Collaboration has existed since the dawn of humanity. It's older than language. It's one of the most widely and deeply considered topics in the world. Don't try to reinvent the wheel.
* Write down a true and honest description of the problem
* Write down a true and honest description of *what you need the robot to do.*
* Now start talking about how you do it
    * Start with the simplest possible strategy that achieves what you need
    * Does this strategy solve the problem you described?
    * If it doesn't solve it exactly, what approximations have you chosen?
    * What heuristics and models have you chosen?
    * If you don't have an answer, you are not doing engineering, you are doing unstructured babble. That does not work in autonomy.

If you found the above "cool", great. Most reasonable people do. But it's just a rehash of Waterfall with more bombastic tone.

* Include time for calibration. If you have *any* sensor, assume you'll need to calibrate it
* If you're using OpenCV, allocate time for dealing with failed calibrations
* If you have multiple sensors, this is going to be hard and you need to think hard about it

# Human Processes
Here, a "process" is some procedure that a _human being_ executes, reading from a list of instructions
* The First Law of Human Processes: A process involving a human with more than two steps is subtly broken.
* The Second Law of Human Processes: A process involving more than 3 steps is broken in some fundamental way

* Observation: The performance of an autonomous system with human supervisors will increase with time, despite no software or hardware changes
    * Conclusion 1: Human operators will accumulate knoweldge about how to coerce the system in a working state
    * Conclusion 2: Often, they won't realize it, and that knowledge won't propagate throughout the organization


# Deep Learning
* A good DL model looks like the code you would have hand-written, making the heuristics optimizable
* The miracle of inverse-optimal-control is that you don't need to have a differentiable optimizer
* Unit test learning code. No exceptions.
* Structure model-building code carefully. Treat training as a form of compilation.
* Data is *one form* of source code, but you also have *actual source code*, and you have to make it good
* Make sure that you have a one-click way to regenerate any model that you've run on the robot
    * Don't wait until you need this capability to get it

## Measuring success
* It should be clear to everyone by now, measure and track PR curves, don't rely on accuracy
* If it's not clear: Imagine classifying whether a reddit comment has an original joke: You can get 99.999% accuracy if you return False for everything.

## Project Planning Pathologies
* Easy trap 1: "End-to-end learn everything, this problem is hard to model"
    *
* Easy trap 2: "Engineer everything, this problem is well understood"
    * If you are operating real sensors with real actuators in the real world, there is a 0% chance you have accurately modelled the world
    * *Sometimes* you can model it adequately
    * Any large robotics system has a config filled with heuristic values. Why not optimiize those via some actual optimization strategy. Acknowledge that you're currently doing gradient descent.

# Tip for Problem Solving
* Write down the truest possible description of the problem. Chances are you have a POMDP.
* Write down exactly

# Matrices
* Use Eigen, avoid dynamic sizes unless the size really is unknown
* Don't use matrices as collections of vectors
    * A user cannot know whether rows are columns are supposed to be distinct entries
    * Instead, consider using a `std::vector<Eigen::Vector3d>`, or some other container
    * Remember, this isn't Python -- you don't need to matrixify everything to make it perform well

# Geometry
* All transformations should be named `destination_from_source`, so that `a_from_b * b_from_c == a_from_c`

* Users should not be exposed to the representation details of your chosen transform. Nobody should be talking about quaternions or matrices
    * Except under very select high-performance situations

* Get comfortable with the concept of liegroups. Don't say "we don't use SO3, we prefer rotation matrices". That's a ridiculous statement.

* Don't use Euler angles. Please. In a pinch, use axis-angle


# Last Resort Optimizations
* Often, it *will* turn out that organizing your code into Eigen matrix/array operations will speed things up.
* SIMD intrinsics are not something to be afraid of
    * But, consider trying ISPC, or reading about how to coerce the compiler into generating vector instructions



# On Complexity
* Be careful when people justify doing something foolish as "avoiding complexity"

* Consider that the CPU on your robot has at least 2 orders of magnitude more "complexity" than your entire machine
* Consider that the OS on your robot has at least 2 orders of magnitude more "complexity" than your entire machine
* State machines for generating behavior in unstructured environments are very unscalable

# Flaws with OpenCV
* [col, row] ordering is inconsistent
    * .at<>(row, col)
    * Point2f(col, row)
* Calibration does not appear to use robust cost functions
* Matrices are not typesafe, and require somewhat challenging reasoning re: memory
* Matrices and images are the same type. Guys, use the type system! It's here to *prevent* this kind of thing!