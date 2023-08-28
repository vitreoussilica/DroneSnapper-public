module SimCore
import FlightMechanics as FM
import LinearAlgebra as LA
using ..DataTypes
using ..DSForces
using ..CoefLoader
using ..MathHelpers
using ..PrintingUtils
export run_flight_sim_1


function run_flight_sim_1(state_init, state_dot_init, datcom_interp, wind_interp, atmos, aircraft_init::DSAircraft, steps::Integer, dt, autopilot, trims)
    mass = aircraft_init.mass
    inertia = aircraft_init.inertia
    control = ControlSurfPositions(0, 0, 0)
    thrust = Thrust([0, 0, 0], [0,0,0])
    geom = aircraft_init.geometry
    aeroparams = aircraft_init.aeroparams
    controller = Controller()

    range = 1:steps

    state_list = zeros(Float64, 13, steps)
    state_dot_list = zeros(Float64, 13, steps)
    controls_list = zeros(Float64, 4, steps)
    t_list = zeros(Float64, 1, steps)
    ap_list = zeros(Float64, 1, steps)

    state_list[:, 1] = state_init
    state_dot_list[:, 1] = state_dot_init
    state = state_init
    state_dot = state_dot_init

    for i in range
        t = i * dt
        control, thrust = autopilot(t, state, state_dot, aircraft_init, atmos, datcom_interp, wind_interp, control , thrust, i, controller, trims)

        forces, moments = calc_forces_and_moments(state, state_dot, datcom_interp, wind_interp, atmos, geom, control, thrust, mass, aeroparams)
    
        state_list[:, i] = state
        state_dot_list[:, i] = state_dot
        controls_list[1, i] = control.canard
        controls_list[2, i] = thrust.force[1]
        controls_list[3, i] = control.aileron_left
        controls_list[4, i] = control.aileron_right
        t_list[1, i] = t
        ap_list[1, i] = controller.q_des

        func_to_pass(called) = FM.six_dof_quaternion_fixed_mass(called, mass, inertia, forces, moments)
        state, state_dot = rg4flight(func_to_pass, dt, state_list[:, i])

        alpha_curr = atan(state[3], state[1])
        alpha_curr_deg = rad2deg(alpha_curr)
        quatl = LA.norm(state[7:10])

        if i%(steps÷20)==0
            println("t: $t\n")
        end

        if alpha_curr_deg > 18 || alpha_curr_deg < -12 || control.canard > 20 || control.canard < -10 || quatl < 0.999 || quatl > 1.001
            println("OUT OF Bounds\ni: $i")
            println("\talpha: $alpha_curr_deg")
            println("\tcanard: ", control.canard)
            println("quaterion len: $quatl")
            println("\tstate:")
            print_state(state)
            println("\tstate dot:")
            print_state_dot(state_dot)
            println("\tforces: $forces\n\tmoments: $moments")
            break
        end
    end
    return state_list, state_dot_list, controls_list, t_list, ap_list
end


end