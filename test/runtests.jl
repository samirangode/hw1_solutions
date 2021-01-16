using HW1
using Test
using Pkg
Pkg.status()

HW1.notebook()

@testset "Question 1" begin
    @testset "Part a" begin
        d = HW1.q1a
        @test typeof(d[:double]) == Float64
        @test typeof(d[:single]) == Float32
        @test typeof(d[:vecint]) == Vector{Int}
        @test typeof(d[:vecdouble]) == Vector{Float64}
        @test size(d[:rowvec],1) == 1
        @test size(d[:colvec],2) == 1
        @test size(d[:matrix2x2]) == (2,2)
        @test typeof(d[:tuple]) == Tuple{Int,Int,Int}
        @test typeof(d[:set]) == Set{Int}
        @test typeof(d[:string]) isa String
        @test typeof(d[:char]) isa Char 
        @test typeof(d[:symbol]) isa Symbol
        @test typeof(d[:dict]) isa Dict{String,Int}
    end
    @testset "Part b" begin
        @test HW1.doubler(2) == 4
        @test HW1.doubler(1.23) == 2.46 
        vec = [1,2,3.]
        @test HW1.shift(vec, 2) == vec .+ 2
        @test HW1.shift(vec, -2.6) == vec .- 2.6
        @test vec == [1,2,3.]
        @test HW1.shift!(vec, 4.2f0)
        vec == [1,2,3.] .+ 4
    end
    @testset "Part c" begin
        @test_nowarn HW1.test_benchmarktools() 
    end

end