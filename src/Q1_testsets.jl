@testset "1a" begin
    f = pendulum_dynamics
    x1 = [0.12, -1.5]
    x2 = [-0.5, 10.2]
    h = 0.1
    @testset "pendulum dynamics" begin
        @test pendulum_dynamics([0,0.]) ≈ zeros(2)                             # POINTS = 0
        @test pendulum_dynamics(x1) ≈ load(resfile, "xdot")                    # POINTS = 1
    end
    @testset "implicit midpoint" begin
        @test implicit_midpoint(f, zeros(2), zeros(2), h) ≈ zeros(2)           # POINTS = 0
        @test implicit_midpoint(f, x1, x1, 0) ≈ zeros(2)                       # POINTS = 1
        @test implicit_midpoint(f, x1, x2, h) ≈ load(resfile, "midpoint_res")  # POINTS = 1
    end
end;

@testset "1b" begin
    f = pendulum_dynamics
    x0 = [deg2rad(45), 0]
    h = 0.1
    @testset "Implicit Midpoint Solve" begin
        x_next = copy(x0)
        residuals = implicit_midpoint_solve!(pendulum_dynamics, x0, x_next, h)
        @test residuals[end] < 1e-6                # POINTS = 4
        @test length(residuals) < 5                # POINTS = 1
        @test x_next ≈ load(resfile, "x_next")     # POINTS = 5
        A = [1,2]
        @test A ≈ [1,3]                            # POINTS = 10
    end
end;

@testset "1c" begin
    @test mean(energy_implicit) ≈ load(resfile, "implicit_energy_mean")  # POINTS = 1
    @test std(energy_implicit) < 0.05                                    # POINTS = 1
end;

@testset "1d" begin
    @testset "rk4" begin
        f = pendulum_dynamics
        x = [-0.12, 1.37]
        h = 0.1
        @test rk4(f, x, h) ≈ load(resfile, "rk4")  # POINTS = 1
    end
end;

@testset "1d" begin
    @testset "energy comparison" begin
        @test length(energy_rk4) == length(energy_implicit)
        @test energy_rk4[end] < energy_implicit[end]                  # POINTS = 1
        @test mean(diff(energy_rk4)) ≈ -2.5e-5 atol=1e-5              # POINTS = 2
        @test std(diff(energy_rk4)) < 1e-3  # should be about linear  # POINTS = 1
    end
end;

@testset "1e" begin
    @test size(eigs_implicit) == (100,2)          # POINTS = 0
    @test mean(eigs_implicit) ≈ 1.0 atol=1e-6     # POINTS = 2
    @test std(eigs_implicit) ≈ 0 atol=1e-10       # POINTS = 2
    @test Amid ≈ load(resfile, "Amid") atol=1e-6  # POINTS = 1
end;