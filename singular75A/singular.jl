using TrajectoryOptimization
using LinearAlgebra
using DelimitedFiles # used to write arrays in files

function dynamics!(ẋ,x,u) # inplace dynamics
    ẋ[1] = x[2]
    ẋ[2] = u[1]
    ẋ[3] = x[1]^2 + x[2]^2
end

n = 3 # number of states
m = 1 # number of controls
model = Model(dynamics!,n,m) # create model
model_d = rk3(model) # create discrete model w/ rk3 integration

x0 = [0.; 1.; 0.] # initial state
xf = [0.; 0.; 0.8285] # goal state

dt = 1 # time step
N = 5 # number of points

#  Testar vários valores pra discretização e construir um gráfico. Começar
#  pelo ALTRO e estudar melhor a fundo como ele funciona. 

U0 = [0.01*rand(m) for k = 1:N-1]; # initial control trajectory

Q = 1.0*Diagonal(I,n)
Qf = 1.0*Diagonal(I,n)
R = 1.0e-1*Diagonal(I,m)
obj = LQRObjective(Q,R,Qf,xf,N) # objective

bnd = BoundConstraint(n,m,u_max=1, u_min=-1) # control limits
# goal = goal_constraint(xf) # terminal constraint

constraints = Constraints(N) # define constraints at each time step
for k = 1:N-1
    constraints[k] += bnd
end
# constraints[N] += goal

prob = Problem(model_d, obj, constraints=constraints, x0=x0, xf=xf, N=N, dt=dt) # construct problem
initial_controls!(prob,U0) # initialize problem with controls

solver = solve!(prob, ALTROSolverOptions{Float64}())

writedlm("./xSol.txt", prob.X)
writedlm("./uSol.txt", prob.U)
