@testset "KKT Residual" begin
    Random.seed!(1)
    ix,iu,iλ = get_partition(model)
    x = copy(x_guess)
    u = randn(control_dim(model))
    λ = randn(state_dim(model))
    ∇f = jacobian(model, x, u)
    A = ∇f[:,ix]
    B = ∇f[:,iu]
    r = kkt_conditions(model, x, u, λ, A, B)
    @test r[ix] ≈ load(resfile, "kkt_residual")[ix] atol=1e-8  # POINTS = 2
    @test r[iu] ≈ load(resfile, "kkt_residual")[iu] atol=1e-8  # POINTS = 2
    @test r[iλ] ≈ load(resfile, "kkt_residual")[iλ] atol=1e-8  # POINTS = 1
end;

@testset "KKT Jacobian" begin
    Random.seed!(1)
    ix,iu,ic = get_partition(model)
    x = copy(x_guess)
    u = randn(control_dim(model))
    λ = randn(state_dim(model))
    ∇f = jacobian(model, x, u)
    A = ∇f[:,ix]
    B = ∇f[:,iu]
    H = kkt_jacobian(model, x, u, λ, A, B)
    Hans = load(resfile, "kkt_jacobian")
    
    # Basic tests
    @test size(H) == size(Hans)  # POINTS = 0.25
    @test issymmetric(H)         # POINTS = 0.25
    
    # Test inertia
    Heig = eigen(H)
    @test count(x->x>0, Heig.values) == sum(state_dim(model) + control_dim(model))  # POINTS = 0.25
    @test count(x->x<0, Heig.values) == sum(state_dim(model))                       # POINTS = 0.25
    
    # Test values
    @test H[ix,ix] ≈ Hans[ix,ix]  # POINTS = 1.0
    @test H[ix,iu] ≈ Hans[ix,iu]  # POINTS = 0.5
    @test H[ix,ic] ≈ Hans[ix,ic]  # POINTS = 0.5
    @test H[iu,iu] ≈ Hans[iu,iu]  # POINTS = 0.5
    @test H[iu,ic] ≈ Hans[iu,ic]  # POINTS = 0.5
    @test H[ic,ic] ≈ Hans[ic,ic]  # POINTS = 0.5
    
    # Test Regularization
    ρdefault = -Hans[end,end]
    Hreg = kkt_jacobian(model, x, u, λ, A, B, 1e-3)
    @test Hreg[ix,ix] ≈ Hans[ix,ix] + I*(1e-3 - ρdefault)  # POINTS = 0.2
    @test Hreg[iu,iu] ≈ Hans[iu,iu] + I*(1e-3 - ρdefault)  # POINTS = 0.2
    @test Hreg[ic,ic] ≈ Hans[ic,ic] - I*(1e-3 - ρdefault)  # POINTS = 0.1
end;

@testset "2b" begin
    ix,iu,ix2 = get_partition(model)
    xstar, ustar, λstar = newton_solve(model, x_guess, verbose=false);
    ∇f = jacobian(model, xstar, ustar)
    A = ∇f[:,ix]
    B = ∇f[:,iu]
    r = kkt_conditions(model, xstar, ustar, λstar, A, B)
    @test norm(r) < 1e-6  # POINTS = 20
end;

@testset "2c" begin
    @test final_residual < 1e-6              # POINTS = 2
    @test length(eigs) == state_dim(model)   # POINTS = 1
    @test maximum(real(eigs)) > 0            # POINTS = 2
end;