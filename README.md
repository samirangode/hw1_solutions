# Optimal Control 16-745 HW1: Due Thursday, February 18
![CI](https://github.com/Optimal-Control-16-745/HW1/workflows/CI/badge.svg)

In this homework we'll be exploring topics in dynamics and optimization. Here's an overview of the problems:
1. Implement an implicit integrator and compare performance against an explicit method
2. Find an equilibrium state to balance a quadruped on one leg using Newton's method and unconstrained optimization
3. Simulate a falling brick by solving a quadratic program. Implement your own augmented Lagrangian QP solver

## Getting Started
All the homeworks are distributed as Jupyter notebooks. Follow these instructions to get everything set up.

1. Install Julia. Download v1.5.3 from the [Julia website](https://julialang.org/downloads/). Extract the binaries onto a stable location on your computer, and add the `/bin` folder to your system path.
2. Clone this repo, and open a terminal in the root directory
2. Start a Julia REPL in your terminal using `julia`. This should work as long as the binaries are on your system path.
3. Install the [IJulia](https://github.com/JuliaLang/IJulia.jl) using the Julia package manager. In the REPL, enter the package manager using `]`, then enter `add IJulia` to add it to your system path.
4. In the REPL (hit backspace to exit the package manager), enter `using IJulia`
5. Launch the notebook using `notebook()` or `jupyterlab()`

## Running the Autograder
The autograder will run automatically whenever you push to GitHub. You can check the status of your autograding results in the `Actions` tab on GitHub, or see if your tests are passing by checking the badge at the top of this README.

To run the autograder locally, boot up a REPL in the root directory of this repository. Activate the current environment in the package manager (i.e. after entering `]`) using
`activate .`. Then run the test suite (still in the package manager) enter `test HW1` and it will launch the testing suite. You will see a printout of the progress of the tests.

For more information on the autograder, see [this file](https://github.com/Optimal-Control-16-745/JuliaIntro/blob/main/docs/Autograding.md).

## Questions / Issues
If you have questions as you work through the homework, please post to the `hw1` channel on Slack, or sign up for office hours.

