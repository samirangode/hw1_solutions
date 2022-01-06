include("autograder.jl")

autograder = true
gradequestion("Q1")
gradequestion("Q2")
gradequestion("Q3")

solutiondir = joinpath(@__DIR__, "..")  # This will change when grading
checktestsets("Q1", solutiondir) 
checktestsets("Q2", solutiondir) 
checktestsets("Q3", solutiondir) 
