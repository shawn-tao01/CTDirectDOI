"""
Instructions

This file is for testing the JuDO interface with the backend CTDirectDOI. 

Run this file in your own Julia environment, with JuDO and DynOptInterface installed the same way as in https://github.com/JuDO-dev/JuDO.jl:
- JuDO
- JuMP
- DynOptInterface
- CTDirectDOI
- Plots

"""
using JuDO, JuMP, DynOptInterface, CTDirectDOI
using Plots


const g   = 9.81
const l   = 0.5
const m_1 = 1.0
const m_2 = 0.3
const t_0 = 0.0
const t_f = 2.0
const u_max = 20.0
const r_max = 2.0

dop = DynModel(CTDirectDOI.Optimizer)

set_attribute(dop, CTDirectDOI.GridSize(),  200)
set_attribute(dop, CTDirectDOI.DiscMethod(), :trapeze)

@phase(dop, t)
@constraint(dop, initial(t) == t_0)
@constraint(dop,   final(t) == t_f)
@variable(dop, -u_max <= u <= u_max, DefinedOn(t))   # control force
@variable(dop,     0  <= r <= r_max, DefinedOn(t))   # cart position
@variable(dop, θ,                    DefinedOn(t))   # pole angle
@variable(dop, ν,                    DefinedOn(t))   # cart velocity
@variable(dop, ω,                    DefinedOn(t))   # pole angular velocity

@constraint(dop, initial(r) == 0.0)
@constraint(dop, initial(θ) == 0.0)
@constraint(dop, initial(ν) == 0.0)
@constraint(dop, initial(ω) == 0.0)

@constraint(dop, final(r) == 1.0)
@constraint(dop, final(θ) == π)
@constraint(dop, final(ν) == 0.0)
@constraint(dop, final(ω) == 0.0)

@constraint(dop, derivative(r) == ν)
@constraint(dop, derivative(θ) == ω)
@constraint(dop, derivative(ν) ==
    (l*m_2*sin(θ)*ω^2 + u + m_2*g*cos(θ)*sin(θ)) / (m_1 + m_2*sin(θ)^2))
@constraint(dop, derivative(ω) ==
    (-l*m_2*cos(θ)*sin(θ)*ω^2 - u*cos(θ) - (m_1+m_2)*g*sin(θ)) /
    (l*(m_1 + m_2*sin(θ)^2)))

@objective(dop, Min, integral(u^2))

struct LinearInterpolant <: DynOptInterface.AbstractDynamicSolution
    y_a::Float64
    y_b::Float64
end
(li::LinearInterpolant)(t::Real) = li.y_a + (t - t_0) * (li.y_b - li.y_a) / (t_f - t_0)

JuDO.warmstart!(dop, LinearInterpolant(0.0, 1.0), r)
JuDO.warmstart!(dop, LinearInterpolant(0.0, π),   θ)


JuDO.optimize!(dop)   

r_sol = dyn_value(dop, r)
θ_sol = dyn_value(dop, θ)
ν_sol = dyn_value(dop, ν)
ω_sol = dyn_value(dop, ω)
u_sol = dyn_value(dop, u)

ts = range(t_0, t_f; length=300)

p1 = plot(ts, r_sol.(ts);  ylabel="Cart position (m)",      legend=false, lc=:black)
p2 = plot(ts, rad2deg.(θ_sol.(ts)); ylabel="Pole angle (°)", legend=false, lc=:blue)
p3 = plot(ts, u_sol.(ts);  ylabel="Control force (N)",      legend=false, lc=:red)
plot(p1, p2, p3; layout=(3,1), xlabel="Time (s)", size=(600,700))

savefig(p1, "cart_position.png")
savefig(p2, "pole_angle.png")
savefig(p3, "control_force.png")