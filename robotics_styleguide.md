This is a short guideline for writing code that runs on robots. Common parlance is to call such a thing a styleguide.

It begins with some notes on project planning and execution. Robots are large projects. As of 2019, they must involve both far-field science and rigorous engineering to succeed.

At the very end are some notes on actual text style.

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

* Include time for calibration. If you have *any* sensor, assume you'll need to calibrate it, and do engineering to create and manage those calibrations
    * If you're using OpenCV, allocate time for dealing with failed calibrations
* If you have multiple sensors, this is going to be hard and you need to think hard about it

# Human Processes
Here, a "process" is some procedure that a _human being_ executes, reading from a list of instructions
* The First Law of Human Processes: A process involving a human with more than two steps is subtly broken.
* The Second Law of Human Processes: A process involving more than 3 steps is broken in some fundamental way

* Observation: The performance of an autonomous system with human supervisors will increase with time, despite no software or hardware changes
    * Conclusion 1: Human operators will accumulate knoweldge about how to coerce the system in a working state
    * Conclusion 2: Often, they won't realize it, and that knowledge won't propagate throughout the organization
    * Conclusion 3: You cannot trust intervention rates as a signal about the real performance of your system

## Measuring success
* It should be clear to everyone by now, measure and track PR curves, don't rely on accuracy
* If it's not clear: Imagine classifying whether a reddit comment has an original joke: You can get 99.999% accuracy if you return False for everything.

## Project Planning Pathologies
* Easy trap 1: "Black-box end-to-end learn everything, this problem is hard to model"
    * Kitchen-sink machine learning: "Throw every feature you can think of, and then the kitchen sink, into the network" would be awesome if it worked
    * Most projects will never gather the gobs of data required to make 2019 state-of-the-art black-box end-to-end DL work for their problem before running out of money. Disclaimer: there's five qualifiers on that statement.
* Easy trap 2: "Engineer everything, this problem is well understood"
    * If you are operating real sensors with real actuators in the real world, there is a 0% chance you have correctly modelled the world
    * *Sometimes* you can model it adequately
    * Any large robotics system has a config filled with heuristic values. Why not optimize those via some actual optimization strategy. Remember that you're currently doing stochastic gradient descent, but less structured, every time you tune a value

* Failing to consider **decision space curvature**
    * Decision importance and decision difficulty are orthogonal concepts
    * When agonzing about a decision, consider: If the *second best* decision is not very different, then the decision doesn't matter

* Decision Theatre
    - Producing pages and pages of documents that will not be read in their entirety by decision makers
    - Getting consensus when it is not required
    - Failing to get consensus when it **is** required

# Tips for Problem Solving
* Write down the truest possible description of the problem. Chances are you have a POMDP.
* Write down exactly the approximations you're making
    * Record clearly the limitations that those approximations imply

# Matrices
* Use Eigen, avoid dynamic sizes unless the size really is unknown
* Don't use matrices as collections of vectors
    * A user cannot know whether rows are columns are supposed to be distinct entries
    * Instead, consider using a `std::vector<Eigen::Vector3d>`, or some other container
    * Remember, this isn't Python -* you don't need to matrixify everything to make it perform well

# Geometry
* All transformations should be named `destination_from_source`, so that `a_from_b * b_from_c == a_from_c`
    * `point_frame_a = a_from_b * point_frame_b`

* Users should not be exposed to the representation details of your chosen transform. Nobody should be talking about quaternions or matrices
    * Except under very select high-performance situations

* Get comfortable with the concept of liegroups. Don't say "we don't use SO3, we prefer rotation matrices". That's a ridiculous statement.

* Don't use Euler angles. Please. In a pinch, use axis-angle

* If you're project is about to use Euler angles in a critical place because it's hard to get liegroups right, seriously, email me and I'll help you.

# Optimization
* In an industrial setting, one usually...
    - Has tight control over the form of cost functions they must optimize
    - is willing to compromise on exact specification to satisfy an optimizer
    - TODO(jpanikulam): More

# Logging
* 99% of the interaction betwen engineers and data you generate will be offline
    - Via training pipelines for deep learning
    - Via offline validation of behavior
    - This sounds obvious, but consider:
        + Does your log playback system support rewind?
        + Does your log playback system support deterministic testing?
        + Does your log format permit reconstructing the state of the system?
        + Ex: The Ouster LIDAR format does not log the lidar mode (1024x20 or 2048x10), meaning you *cannot* automatically parse a bagged Ouster unless you know the format. That's silly. Just send the extra 2 bits to identify the mode!!!
    

# Deep Learning
* A good DL model looks like the code you would have hand-written, making the heuristics optimizable
* The miracle of inverse-optimal-control is that you don't need to have a differentiable optimizer to learn a cost function
* Unit test learning code. Do not make exceptions.
* Structure model-building code carefully. Treat training as a form of compilation.
* Data is *one form* of source code, but you also have *actual source code*, and you have to make it good
* Make sure that you have a one-click way to regenerate any model that you've run on the robot
    * Don't wait until you need this capability to get it

# Debugging
* Visualize everything in 3D.
    * If you're doing robotics, your problem is 3D. Visualize it in 3D.
    * Stop making excuses. Visualize it in 3D. If you haven't done this, your code is **broken**
* When you run into a bug, the *first thing you should do* is spend 5min looking for obvious bugs.
* The *second thing you should do* is write a debugging tool
    * You'll need it when you run into your bug the second time
* Don't don't don't resort to trial and error. Inverting transforms, negating random things, tweaking values without reason. How many times has this ever worked!?!?

# Last Resort Optimizations
* Often, it really will turn out that organizing your code into Eigen matrix/array operations will speed things up.
* SIMD intrinsics are not something to be afraid of
    * But, consider trying ISPC, or reading about how to coerce the compiler into generating vector instructions
    * OpenCL is not that hard

# On Complexity
* Be careful when people justify doing something foolish as "avoiding complexity"
* Consider that the CPU on your robot has at least 2 orders of magnitude more "complexity" than your entire machine
* Consider that the OS on your robot has at least 2 orders of magnitude more "complexity" than your entire machine
* Consider that A* is 2 orders of magnitude simpler than a well-designed optimal controller. A* is not going to land a rocket.
* State machines for generating behavior in unstructured environments are very unscalable

# Flaws with OpenCV
* [col, row] ordering is inconsistent -- Largely because the Matrix type and the image type are the same (When was the last time you needed to perform GEMM on two uint8 images?)
    * .at<>(row, col)
    * Point2f(col, row)
* Calibration does not appear to use robust cost functions, and is prone to surprising errors for even expert users
* Matrices are not typesafe. It is true that templates bloat binaries, but this is often something I'm willing to accept
* Matrices are strictly shared_ptrs, you can't make them const when passed to an image
    - This can be a huge footgun when many OpenCV functions will modify inputs in place
* Matrices and images are the same type. Guys, use the type system! It's here to *prevent* this kind of thing!


# OpenGL
Do not resort to trial-and-error to make 3D views work. These are structured things that have been understood for centuries - label the matrices and think carefully about what they mean.

OpenGL nominal matrix formats are usually described as follows:

* pt_image_frame = proj_from_view * view_from_world * world_from_model * pt_model_frame

* Don't confuse the OpenGL *memory layout* with the OpenGL *matrix multiplication convention*. OpenGL stores matrices in column-major order. This has no effect on what matrix multiplication means. It most definitely does not reinvent the conventions of matrix multiplication, as some tutorials suggest.

* OpenGL MultMatrix calls *right* multiply the view_from_world matrix by the supplied matrix
    * This means glMultMatrix(`world_1_from_world_2`) --> `view_from_world_2 = view_from_world_1 * world_1_from_world_2`

# Style
* Enforce it automatically where possible. Use clang-format (for C++) and flake8 as a test in your CI pipeline
* Recommend clang-format with some chosen .clang-format config for everyone
* Caring is the most important thing, getting the style exactly right is less important
