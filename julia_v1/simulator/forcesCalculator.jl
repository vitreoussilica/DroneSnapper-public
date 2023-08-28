module DSForces
using ..DataTypes
import LinearAlgebra as LA
export calc_forces_and_moments

function calc_forces_and_moments(state::Vector{Float64}, state_dot::Vector{Float64}, datcom_interp, wind_interp, atmosphere::AtmoshpereState, geometry::DSGeometry, control_surf::ControlSurfPositions, thrust::Thrust, mass::Float64, aeroparams)
    f_aero = calc_f_aero(state, state_dot, datcom_interp, wind_interp, atmosphere, geometry, control_surf, aeroparams)

    moments = calc_moments(state, state_dot, f_aero, datcom_interp, wind_interp, atmosphere, geometry, control_surf, thrust, aeroparams)

    forces = calc_forces(state, f_aero, atmosphere, geometry, thrust, mass)

    return forces, moments
end

function calc_f_aero(state, state_dot, datcom_interp, wind_interp, atmosphere::AtmoshpereState, geometry::DSGeometry, control_surf::ControlSurfPositions, aeroparams::DSAeroParams)
    x::Float64, y::Float64, z::Float64 = 0, 0, 0
    CL::Float64, CD::Float64, CY::Float64 = 0, 0, 0

    speeds_lin = state[1:3]

    alpha = atan(speeds_lin[3], speeds_lin[1])
    alpha_deg = rad2deg(alpha)
    beta = atan(speeds_lin[2], speeds_lin[1])
    V = LA.norm(speeds_lin)
    V_dot = LA.norm(state_dot[1:3])
    dq = geometry.c / 2 * state_dot[5] / V
    dp = geometry.b / 2 * state_dot[4] / V
    dr = geometry.b / 2 * state_dot[6] / V

    CL += wind_interp["CL"](alpha_deg, control_surf.canard,V)
    CY += datcom_interp["cyb"](alpha_deg) * beta
    CD += wind_interp["CD"](rad2deg(alpha),control_surf.canard,V)
    
    CL += datcom_interp["clq"](alpha_deg) * dq 
    CY += datcom_interp["cyp"](alpha_deg) * dp
    
    L = atmosphere.rho * V^2 / 2 * CL * geometry.S
    z -= cos(alpha)*L
    x += sin(alpha)*L

    D = atmosphere.rho * V^2 / 2 * CD * geometry.S
    z -= sin(alpha)*D
    x -= cos(alpha)*D

    y += atmosphere.rho * V^2 / 2 * CY * geometry.S



    forces = Float64[x, y, z]
    return forces    
end

function calc_moments(state, state_dot, f_aero, datcom_interp, wind_interp, atmosphere, geometry, control_surf, thrust, aeroparams::DSAeroParams)
    x::Float64, y::Float64, z::Float64 = 0, 0, 0
    CM::Float64, CL::Float64, CN::Float64 = 0, 0, 0

    speeds_lin = state[1:3]

    alpha = atan(speeds_lin[3], speeds_lin[1])
    alpha_deg = rad2deg(alpha)
    beta = atan(speeds_lin[2], speeds_lin[1])
    V = LA.norm(speeds_lin)
    V_dot = LA.norm(state_dot[1:3])

    dq = geometry.c / 2 * state_dot[5] / V
    dp = geometry.b / 2 * state_dot[4] / V
    dr = geometry.b / 2 * state_dot[6] / V

    CM += wind_interp["CM"](rad2deg(alpha),control_surf.canard,V)
    CN += datcom_interp["cnb"](alpha_deg) * beta
    CL += datcom_interp["clb"](alpha_deg) * beta

    CM += datcom_interp["cmq"](alpha_deg) * dq
    CL += datcom_interp["clp"](alpha_deg) * dp + datcom_interp["clr"](alpha_deg) * dr
    CN += datcom_interp["cnp"](alpha_deg) * dp + datcom_interp["cnr"](alpha_deg) * dr

    CL += aeroparams.aileron_rolling * (control_surf.aileron_right + control_surf.aileron_left)/2
    
    y += atmosphere.rho * V^2 / 2 * CM * geometry.S * geometry.c
    x += atmosphere.rho * V^2 / 2 * CL * geometry.S * geometry.c
    z += atmosphere.rho * V^2 / 2 * CN * geometry.S * geometry.c

    moments = Float64[x, y, z]

    moments += LA.cross((geometry.CA - geometry.CG), f_aero)

    moments += thrust.moment
    moments += LA.cross((geometry.CT - geometry.CG), thrust.force)

    return moments    
end

function calc_forces(state, f_aero, atmosphere, geometry, thrust, mass)
    forces::Vector{Float64} = f_aero
    forces += thrust.force

    e0, ex, ey, ez = state[7], state[8], state[9], state[10]
    forces += atmosphere.g*mass*[2*(ex*ez-ey*e0), 2*(ey*ez+ex*e0), ez^2+e0^2-ex^2-ey^2]

    return forces    
end

end
