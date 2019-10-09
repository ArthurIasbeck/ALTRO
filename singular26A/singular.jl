using TrajectoryOptimization
using LinearAlgebra
using DelimitedFiles # Utilizado para escrever vetores em arquivos

T = Float64

function dynamics!(ẋ,x,u) # inplace dynamics
    ẋ[1] = x[2]
    ẋ[2] = u[1]
    ẋ[3] = x[1]^2
end

n = 3 # number of states
m = 1 # number of controls
model = Model(dynamics!,n,m) # create model
model_d = rk3(model) # create discrete model w/ rk3 integration

x0 = [0.; 1.; 0.] # initial state
xf = [0.; 0.; 0.269] # goal state

dt = 0.01 # time step
N = 500 # number of knot points

U0 = [0.01*rand(m) for k = 1:N-1]; # initial control trajectory

# Q = nxn
# R = mxm
# Qf = nxn
# H = mxn
# q = 1xn
# r = 1xm
# c = 1x1
# qf = 1xn
# cf = 1x1

# Definição da função objetivo

# Parâmetros originais do QuadraticCost utilizados neste códigos. Eles remetem
# a um custo do tipo LQR

# Q = 1.0*Diagonal(I,n)
# Qf = 1.0*Diagonal(I,n)
# R = 1.0e-1*Diagonal(I,m)
# H = zeros(size(R,1),size(Q,1))
# q = -Q*xf
# r = zeros(size(R,1))
# c = 0.5*xf'*Q*xf
# qf = -Qf*xf
# cf = 0.5*xf'*Qf*xf

Q = 0.0*Diagonal(I,n)
Qf = 0.0*Diagonal(I,n)
R = 0.000001*Diagonal(I,m)
H = zeros(size(R,1),size(Q,1))
q = 0.0*-Q*xf
r = zeros(size(R,1))
c = 0.0*xf'*Q*xf
qf = [0.0, 0.0, 1.0]
cf = 0.0*xf'*Qf*xf

ℓ = QuadraticCost(Q, R, H, q, r, c)
ℓN = QuadraticCost(Qf, qf, cf)

obj = Objective([k < N ? ℓ : ℓN for k = 1:N])

bnd = BoundConstraint(n,m,u_max=1, u_min=-1) # control limits
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
