using TrajectoryOptimization
using LinearAlgebra
using DelimitedFiles # Utilizado para escrever vetores em arquivos

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

# Definição da função objetivo
Q = zeros(n,n)
R = zeros(m,m);
Qf = zeros(n,n);
H = zeros(m,n);
q = zeros(1,n);
r = zeros(1,n);
c = 0;
qf = [.0 .0 1.];
cf = 0;

"
Q = [0.1 0.0 0.0; 0.0 0.1 0.0; 0.0 0.0 0.1] nxn
R = [0.1] mxm
Qf = [1000 0 0; 0 1000 0; 0 0 1000] nxn
H = [0.0 0.0 0.0] mxn
q = [-0.0, -0.0, -0.026900000000000004] 1xn
r = [0.0] 1xm
c = 0.0036180500000000007 cte
qf = [-0.0, -0.0, -269.0] 1xn
cf = 36.1805 cte
"

# O QuadraticCost não está funcionando. Preciso procurar algum exemplo em que
# ele foi utilizado pra me basear

costfun      = QuadraticCost(Q, R, H, q, r, c)
costfun_term = QuadraticCost(Qf, R*0, H, qf, r*0, cf)
obj = Objective(costfun, costfun_term, N)

# obj = LQRObjective(Q,R,Qf,xf,N) # objective

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
