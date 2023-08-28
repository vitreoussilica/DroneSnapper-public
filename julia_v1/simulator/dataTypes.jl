module DataTypes
export AtmoshpereState, DSGeometry, ControlSurfPositions, Thrust, DSAeroParams, DSAircraft, Controller

struct AtmoshpereState
    rho::Float64
    g::Float64
end

struct DSGeometry
    S::Float64 #wing area
    c::Float64 #mean aerodynamic chord
    b::Float64 #wing span
    CG::Vector{Float64} #mass center position
    CA::Vector{Float64} #aeroforces pinpoint position
    CT::Vector{Float64} #thrust position
end

mutable struct ControlSurfPositions #control surface position settings
    canard::Float64
    aileron_left::Float64
    aileron_right::Float64
end

mutable struct Thrust
    force::Vector{Float64}
    moment::Vector{Float64}
end

struct DSAeroParams
    aileron_rolling::Float64 #deltaCL (rolling coef) per radian for ailerons
end

mutable struct DSAircraft
    geometry::DSGeometry
    # control::ControlSurfPositions
    aeroparams::DSAeroParams
    # thrust::Thrust
    mass::Float64
    inertia::Matrix{Float64}
end

mutable struct Controller
    thr_integrator::Float64
    last_t::Float64
    canard_integrator::Float64
    last_thrust::Float64
    last_canard::Float64
    q_des::Float64
end
Controller() = Controller(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

end