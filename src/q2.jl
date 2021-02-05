import Pkg; Pkg.activate(joinpath(@__DIR__,"..")); Pkg.instantiate()
using RigidBodyDynamics
using MeshCat
using MeshCatMechanisms
using Random
using StaticArrays
using Rotations
using LinearAlgebra
using ForwardDiff
using SparseArrays

include("single_leg_pendulum.jl")

include("quadruped.jl")
const URDFPATH = joinpath(@__DIR__, "..","a1","urdf","a1.urdf")

##

# function ev_dynamics(x::AbstractVector{T}, u::AbstractVector{F}) where {T,F}
#     if typeof(x[1]) <: ForwardDiff.Dual
# #         state = MechanismState{T}(a1.mechanism);
#         a1.state = statecache[T]
#         dr = dr_cache[T];
#     else
# #         state = MechanismState{F}(a1.mechanism);
#         a1.state = statecache[F]
#         dr = dr_cache[F]
#         x = convert(AbstractVector{F},x);
#     end
#     ẋ = zeros(typeof(x[1]),size(x,1));
#     u_total = vcat(SVector(0.,0.,0.),u)
#     ẋ = dynamics!(ẋ,dr,a1.state,x,u_total)
#     return ẋ
# end

# function dynamics(x::AbstractVector{T1}, u::AbstractVector{T2}) where {T1,T2} 
#     T = promote_type(T1,T2)
#     state = statecache[T]
#     res = dyncache[T]

#     # Convert from state ordering to the ordering of the mechanism
#     # q = state2mech(x[SVector{15}(1:15)]) 
#     # v = state2mech(x[SVector{15}(16:30)])
#     copyto!(state, x)
#     # τ = controls2torques(u)
#     τ = [zeros(3); u]
#     # set_configuration!(state, q)
#     # set_velocity!(state, v)
#     dynamics!(res, state, τ)
#     # q̇ = mech2state(res.q̇)
#     # v̇ = mech2state(res.v̇)
#     q̇ = res.q̇
#     v̇ = res.v̇
#     return [q̇; v̇]
# end

# function jacobian(x, u)
#     ix = SVector{30}(1:30)
#     iu = SVector{12}(31:42)
#     faug(z) = dynamics(z[ix], z[iu])
#     z = [x; u]
#     ForwardDiff.jacobian(faug, z)
# end

function kkt_conditions(model,x,u,λ,A,B)
    Lx = x - x_guess + A'λ
    Lu = 1e-3*u + B'λ
    c = dynamics(model,x,u)
    return [Lx; Lu; c]
end

function kkt_jacobian(model, x, u, λ, A, B, ρ=1e-5)
    # Get initial guess and sizes
    # x_guess = mechstate2fullstate(initial_state(model))
    n = 30   # number of states
    m = 12   # number of controls
    nq = 15  # number of position states
    nv = 15  # number of velocity states
    
    # HINT: You may find these ranges to be helpful
    ix = 1:n
    iu = n .+ (1:m)
    ix2 = n+m .+ (1:n)
    
    # Evaluate the continuous time Jacobian
    # ∇f = jacobian(x, u)
    # A = ∇f[:,1:n]
    # B = ∇f[:,n .+ (1:m)]
    
    # Create the KKT matrix
    H = zeros(2n+m, 2n+m)
    H[ix,ix] .= I(n)*(1 + ρ)
    H[ix,ix2] .= A'
    H[iu,iu] .= I(m)*(1e-3 + ρ)
    H[iu,ix2] .= B'
    H[ix2,ix] .= A
    H[ix2,iu] .= B
    H[ix2,ix2] .= -I(n)*ρ
    return Symmetric(H,:L)
end

model = DoubleTreeA1()
mvis = initialize_visualizer(model)
open(mvis)


##
# a1 = SingleLegPendulum(load_a1_urdf(URDFPATH))
# build_rear_foot_constraints_revolute!(a1)

# setting initial standing position
q_guess = [0.7494121122172587, -0.30800825827034156, 0.0739469451153811, -0.8973, 1.106, -0.3587759235809219, -0.8007, 0.8020,0.158,0.1667,0.482,1.2031,-0.9088,-0.9091,-0.906814375366219]
q_guess = initial_state(model)
set_configuration!(mvis, q_guess)
# set_configuration!(a1.state, q_guess)

# const statecache = StateCache(a1.mechanism)
# const dyncache = DynamicsResultCache(a1.mechanism)
const statecache = StateCache(model.mech)
const dyncache = DynamicsResultCache(model.mech)

## Root finding 
q_guess = initial_state(model)
u = zeros(12)
x_guess = [q_guess; zeros(15)] 
x = copy(x_guess) 
ẋ = similar(x)
λ = zeros(size(x_guess))
ρ = 1e-3

# dL = Matrix{Float64}(I,length(x)+length(u),length(x)+length(u))
# dL[length(x)+1:end,length(x)+1:end] = dL[length(x)+1:end,length(x)+1:end]* α
# H = zeros(length(x)*2+length(u),length(x)*2+length(u))
# H[length(x)+length(u)+1:end,length(x)+length(u)+1:end] .= -Matrix{Float64}(I,length(x),length(x)) * α
# H[1:length(x)+length(u),1:length(x)+length(u)]= dL
# res = zeros(length(x))
# A = zeros(length(x),length(x))
# B = zeros(length(x),length(u))
# A = ForwardDiff.jacobian(t->ev_dynamics(t,u),x)
# B = ForwardDiff.jacobian(t->ev_dynamics(x,t),u)

res_hist = zeros(20,2)
xbar = copy(x)
ubar = copy(u)
λbar = copy(λ)
for i in 1:50
    # Update Guess with Newton's Method
    u′ = copy(u)
    x′ = copy(x)
    ∇f = jacobian(x, u)
    A = ∇f[:,1:30]
    B = ∇f[:,31:42]
    r = kkt_conditions(model, x,u,λ, A,B)

    H = kkt_jacobian(model, x, u, λ, A, B, ρ)

    # Calculate delta x and update 
    δx = -H \ r
#     println(residual[15:30])
    α = 1.0
    for i = 1:10
        xbar .= x + α*δx[1:30]  
        ubar .= u + α*δx[31:42]  
        λbar .= λ + α*δx[43:72]
        ∇fbar = jacobian(model, xbar, ubar)
        Abar = ∇fbar[:,1:30]
        Bbar = ∇fbar[:,31:42]
        rbar = kkt_conditions(model, xbar, ubar, λbar, Abar, Bbar)

        if norm(rbar) < norm(r) + Inf
            x .= xbar
            u .= ubar
            λ .= λbar
            ρ = 1e-3
            break
        else
            α *= 0.5
        end
        i == 10 && (ρ *= 2)
    end
    println("α = $α, ρ = $ρ")
    set_configuration!(mvis, x[1:15])
    sleep(0.1)
    # x .= xbar
    # u .= ubar
    # λ .= λbar

    # x .= x .+ δx[1:length(x)]
    # u .= u .+ δx[length(x)+1:length(x)+length(u)]
    # λ .= λ .+ δx[length(x)+length(u)+1:end]

    # res_hist[i,:] = [norm(), norm(res)]
    println(norm(r))
    # if i == 50
    #     println("Dynamics Constraints Residual ",norm(c))
    #     println("Cost Function Constraints Residual ", norm(res))
    # end
end


##
function kkt_conditions(model, x, u, λ)
    # Get initial guess and sizes
    # x_guess = mechstate2fullstate(initial_state(model))
    n = 30   # number of states
    m = 12   # number of controls
    nq = 15  # number of position states
    nv = 15  # number of velocity states
    
    # Evaluate the continuous time Jacobian
    ∇f = jacobian(model, x, u)
    A = ∇f[:,1:n]
    B = ∇f[:,n .+ (1:m)]
    
    # Evaluate the KKT conditions
    ∇ₓL = (x-x_guess) + A'λ
    ∇ᵤL = 1e-6*u + B'λ
    c = dynamics(model, x, u)
    
    # Stack the conditions
    return [∇ₓL; ∇ᵤL; c]
end

function kkt_jacobian(model, x, u, λ, ρ=1e-5)
    # Get initial guess and sizes
    # x_guess = mechstate2fullstate(initial_state(model))
    n = 30   # number of states
    m = 12   # number of controls
    nq = 15  # number of position states
    nv = 15  # number of velocity states
    
    # HINT: You may find these ranges to be helpful
    ix = 1:n
    iu = n .+ (1:m)
    ix2 = n+m .+ (1:n)
    
    # Evaluate the continuous time Jacobian
    ∇f = jacobian(model, x, u)
    A = ∇f[:,1:n]
    B = ∇f[:,n .+ (1:m)]
    
    # Create the KKT matrix
    H = zeros(2n+m, 2n+m)
    H[ix,ix] .= I(n)
    H[ix,ix2] .= A'
    H[iu,iu] .= I(m)*(1e-6 + ρ)
    H[iu,ix2] .= B'
    H[ix2,ix] .= A
    H[ix2,iu] .= B
    H[ix2,ix2] .= -I(n)*ρ
    return Symmetric(H,:L)
end

function run_newton(model, x0, u0, λ0)
    x = copy(x0)
    u = copy(u0)
    λ = copy(λ0)
    for i = 1:20
        r = kkt_conditions(model, x, u, λ)
        H = kkt_jacobian(model, x, u, λ, α)
        println(norm(r))
        if norm(r) < 1e-8
            break
        end

        dy = -H\r
        x += dy[ix]
        u += dy[iu]
        λ += dy[ix2]
    end
end

model = DoubleTreeA1()
q_guess = SA[0.7494121122172587, -0.30800825827034156, 0.0739469451153811, -0.8973, 1.106, -0.3587759235809219, -0.8007, 0.8020,0.158,0.1667,0.482,1.2031,-0.9088,-0.9091,-0.906814375366219]
# q_guess = SVector(SA[0.446517  -0.555426  0.335584  -0.814024  1.1142  -0.360247  -0.798245  0.801138  0.159834  0.167102  0.482847  1.20464  -0.908296  -0.907996  -0.905929]')
v_guess = @SVector zeros(15)
x_guess = [q_guess; v_guess]
x = copy(x_guess)
u = zeros(12) 
λ = zero(x)
dynamics(model, x, u)

r = kkt_conditions(model, x, u, λ)
H = kkt_jacobian(model, x, u, λ)

##
y = [x;u;λ]
res(y) = kkt_conditions(model, y[1:30], y[31:42], y[43:end])
∇res(y) = kkt_jacobian(model, y[1:30], y[31:42], y[43:end], 0)
∇resfd(y) = FiniteDiff.finite_difference_jacobian(res, y)

Ireg = Diagonal([ones(42); -ones(30)])
ρ = 0

##
r = norm(res(y))
dy = -(∇resfd(y) + Ireg*ρ)\res(y)
α = 1
for i = 1:10
    if norm(res(y + α*dy)) < r
        y .= y .+ α*dy
        if i <= 3
            ρ = 0
        end
        break
    else
        α *= 0.5
    end
    if i == 10
        ρ += 10
    end
end
println(α)
println(norm(res(y)[43:end]))
norm(res(y))

##
xsol = y[1:30]
usol = y[31:42]
norm(dynamics(model, xsol, usol))

# ∇res(y) = FiniteDiff.finite_difference_jacobian(res, y)

# run_newton(model, x, u, λ)

##
using FiniteDiff
include("quadruped.jl")
model = DoubleTreeA1()
# q_guess = SA[0.7494121122172587, -0.30800825827034156, 0.0739469451153811, -0.8973, 1.106, -0.3587759235809219, -0.8007, 0.8020,0.158,0.1667,0.482,1.2031,-0.9088,-0.9091,-0.906814375366219]
q_guess = SVector(SA[0.446517  -0.555426  0.335584  -0.814024  1.1142  -0.360247  -0.798245  0.801138  0.159834  0.167102  0.482847  1.20464  -0.908296  -0.907996  -0.905929]')
v_guess = @SVector zeros(15)
x_guess = [q_guess; v_guess]

x = copy(x_guess)
u = zeros(12)
λ = zero(x)

kkt_conditions(model, x, u, λ)
res(y) = kkt_conditions(model, y[1:30], y[31:42], y[43:end])
∇res(y) = FiniteDiff.finite_difference_jacobian(res, y)
y = [x;u;λ]

r = norm(res(y))
dy = -∇res(y)\res(y)
α = 1
for i = 1:10
    if norm(res(y + α*dy)) < r
        y .= y .+ α*dy
        break
    else
        α *= 0.5
    end
    i == 10 && @warn "LS failed"
end
println(α)
println(norm(y[43:end]))
norm(res(y))

##
mvis = initialize_visualizer(model)
open(mvis)
set_configuration!(mvis, xsol[1:15])

## Try Newton on just the dynamics