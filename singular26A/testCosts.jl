using TrajectoryOptimization
using LinearAlgebra

N = 51;
n,m = 2,1

# Custo LQR --------------------------------------------------------------------
# Q = Diagonal(0.1I,n)
# R = Diagonal(0.1I,m)
# Qf = Diagonal(1000I,n)
# xf = [π,0]
# costfun = LQRCost(Q,R,Qf)
# costfun_term = LQRCostTerminal(Qf,xf)

# Custo Quadrático -------------------------------------------------------------
# H = zeros(m,n)
# q = -Q*xf
# r = zeros(m)
# c = xf'Q*xf/2
# qf = -Qf*xf
# cf = xf'Qf*xf/2
# costfun      = QuadraticCost(Q, R, H, q, r, c)
# costfun_term = QuadraticCost(Qf, R*0, H, qf, r*0, cf)

# Custo genérico ---------------------------------------------------------------
# Define the stage and terminal cost functions
function mycost(x,u)
    R = Diagonal(0.1I,1)
    Q = 0.1
    return cos(x[1] + u'R*u + Q*x[2]^2)
end
function mycost(xN)
    return cos(xN[1]) + xN[2]^2
end

# Create the nonlinear cost function
nlcost = GenericCost(mycost,mycost,n,m)
nlobj = Objective(nlcost, N)
