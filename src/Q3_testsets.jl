@testset "3b" begin
    Random.seed!(1)
    # Setting up and solving a random QP
    n,m,p = 10,0,15 
    qp = QPData(n,m,p)
    P = rand(n,n)
    qp.P .= P'P   # make it P.S.D
    qp.q .= randn(n)
    qp.A .= randn(m,n)
    qp.b .= randn(m)
    qp.C .= randn(p,n)
    qp.d .= randn(p)

    # Initial guess
    x = randn(n)

    # Solve
    xstar, λstar, μstar = solve_qp(qp, x)
    
    # Check optimality conditions
    @test norm(primal_residual(qp, xstar, λstar, μstar)) < 1e-3  # POINTS = 5
    @test norm(dual_residual(qp, xstar, λstar, μstar)) < 1e-6    # POINTS = 5
    @test norm(complimentarity(qp, xstar, λstar, μstar)) < 1e-3  # POINTS = 5
    
    # Compare with OSQP
    using OSQP, SparseArrays
    model = OSQP.Model()
    OSQP.setup!(model, P=sparse(qp.P), q=qp.q, A=sparse([qp.A; qp.C]), l=[qp.b; fill(-Inf,p)], u=[qp.b; qp.d],
        eps_abs=1e-6, eps_rel=1e-6)
    res = OSQP.solve!(model)
    @test norm(res.x - xstar) < 1e-3           # POINTS = 5
    @test norm(res.y - [λstar; μstar]) < 1e-3  # POINTS = 5
end;

@testset "3c" begin
    @testset "build qp" begin
        q = [1.2,-0.36]
        v = [10,-1.2]
        qp = build_qp(q, v)
        @test qp.P ≈ load(resfile, "P") atol=1e-6  # POINTS = 0.5
        @test qp.q ≈ load(resfile, "q") atol=1e-6  # POINTS = 0.5
        @test qp.A ≈ load(resfile, "A") atol=1e-6  # POINTS = 0.25
        @test qp.b ≈ load(resfile, "b") atol=1e-6  # POINTS = 0.25
        @test qp.C ≈ load(resfile, "C") atol=1e-6  # POINTS = 0.25
        @test qp.d ≈ load(resfile, "d") atol=1e-6  # POINTS = 0.25
    end
end;

using Statistics
@testset "3c" begin
    @testset "simulate brick" begin
        h = 0.01
        qans = load(resfile, "qs")
        vans = load(resfile, "vs")
        qs, vs = simulate_brick(h=h)
        eps = 1e-6

        @test [q[1]/0.01 for q in diff(qs)] ≈ [v[1] for v in vs[1:end-1]] atol=1e-6  # Sanity check velocities              POINTS = 0.5
        @test std([q[1] for q in diff(qs)]) < eps                                    # no horizontal acceleration           POINTS = 0.5
        @test all(q->q[1] > 0, diff(qs))                                             # positive horizontal velocity         POINTS = 0.5
        @test all(q->q[2] > -eps, qs)                                                # no penetration through the floor     POINTS = 1
        @test all(v->v[1] ≈ 1.0, vs)                                                 # constant horizontal velocity         POINTS = 0.5
        @test all(v->v[2] < eps, vs)                                                 # all vertical velocity is negative    POINTS = 1
        @test all(v->abs(v[2]) < eps, vs[101:end])                                   # zero vertical velocity after impact (actual impact time is before this)  # POINTS = 1
        @test qs ≈ qans atol=1e-3  # POINTS = 1.5
        @test vs ≈ vans atol=1e-3  # POINTS = 1.5
    end
end;