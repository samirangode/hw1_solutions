using BenchmarkTools
using ForwardDiff
using OSQP
using SparseArrays
include(joinpath(@__DIR__,"..","src","qpal.jl"))

n,m,p = 10,2,15
P = @SMatrix rand(n,n)
P = Symmetric(P'P)
q = @SVector randn(n)
A = @SMatrix randn(Float64,m,n)
b = @SVector randn(Float64, m)
C = @SMatrix randn(p,n)
d = @SVector randn(p)
a = @MVector zeros(Bool,p)

qp = QPAL(P,q, A,b, C,d, a, 10.0, 10.0)

x = @SVector rand(n)
λ = @SVector zeros(Float64, m)
μ = @SVector zeros(Float64, p)
auglag(qp, x, λ, μ)
algrad(qp, x, λ, μ)
alhess(qp, x, λ, μ)

al(x) = auglag(qp, x, λ, μ)
ForwardDiff.gradient(al, x) ≈ algrad(qp, x, λ, μ)
ForwardDiff.hessian(al, x) ≈ alhess(qp, x, λ, μ)

## Test solve
xsol, λsol, μsol = solve(qp, x, λ, μ, verbose=0, ρ0=10, eps_primal=1e-8)

# Optimality conditions
norm(primal_residual(qp, xsol, λsol, μsol))
norm(dual_residual(qp, xsol, λsol, μsol))
norm(min.(0,μsol))
norm(μ .* cin(qp, xsol))

## Solve with OSQP
model = OSQP.Model()
OSQP.setup!(model, P=sparse(P), q=Vector(q), 
    A=sparse([A; C]), l = [b; fill(-Inf,p)], u=[b; Vector(d)], eps_abs=1e-8, eps_rel=1e-8) 
res = OSQP.solve!(model)
norm(res.x - xsol)
norm(res.y - [λsol; μsol])


## Compare Timing results
res = OSQP.solve!(model)
@btime solve($qp, $x, $λ, $μ, verbose=0, ρ0=10)

model = OSQP.Model()
OSQP.setup!(model, P=sparse(P), q=Vector(q), 
    A=sparse(C), l = fill(-Inf,p), u=Vector(d), verbose=false)

@btime begin
    OSQP.warm_start!($model, x=$(Vector(x)), y=$(Vector(μ)))
    OSQP.solve!($model)
end


