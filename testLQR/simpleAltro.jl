#=
Já que o único objetivo utilizado em todos os exemplos, e o único que funcionou
até agora foi o LQRObjective, vamos tentar avaliá-lo. Primeiramente, vimos que
ele recebe como parâmetros AbstractArrays, então vamos tentar substituir o
construtor pelo seu respectivo código e ver o que acontece.
=#

using TrajectoryOptimization
using LinearAlgebra
using DelimitedFiles # used to write arrays in files

function dynamics!(ẋ,x,u) # inplace dynamics
    ẋ[1] = x[2]
    ẋ[2] = u[1]
end

n = 2 # number of states
m = 1 # number of controls
model = Model(dynamics!,n,m) # create model
model_d = rk3(model) # create discrete model w/ rk3 integration

x0 = [0.; 0.] # initial state
xf = [5.; 0.] # goal state

dt = 0.01 # time step
N = 1000 # number of knot points

U0 = [0.01*rand(m) for k = 1:N-1]; # initial control trajectory

Q = 1.0*Diagonal(I,n)
Qf = 1.0*Diagonal(I,n)
R = 1.0e-1*Diagonal(I,m)

# obj = LQRObjective(Q,R,Qf,xf,N) # objective

H = zeros(size(R,1),size(Q,1))
q = -Q*xf
r = zeros(size(R,1))
c = 0.5*xf'*Q*xf
qf = -Qf*xf
cf = 0.5*xf'*Qf*xf

ℓ = QuadraticCost(Q, R, H, q, r, c)
ℓN = QuadraticCost(Qf, qf, cf)

obj = Objective([k < N ? ℓ : ℓN for k = 1:N])

bnd = BoundConstraint(n,m,u_max=20, u_min=-20) # control limits
goal = goal_constraint(xf) # terminal constraint

constraints = Constraints(N) # define constraints at each time step
for k = 1:N-1
    constraints[k] += bnd
end
constraints[N] += goal

prob = Problem(model_d, obj, constraints=constraints, x0=x0, xf=xf, N=N, dt=dt) # construct problem
initial_controls!(prob,U0) # initialize problem with controls

solver = solve!(prob, ALTROSolverOptions{Float64}())

writedlm("./xSol.txt", prob.X)
writedlm("./uSol.txt", prob.U)
