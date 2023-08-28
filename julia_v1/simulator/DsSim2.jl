# Standalone simulator not using separate SimCore

include("./dataTypes.jl")
include("./forcesCalculator.jl")
include("./coefLoader.jl")
include("./mathhelpers.jl")
include("./utils_print.jl")
include("./SimCore.jl")
include("./Autopilot.jl")
import FlightMechanics as FM
import LinearAlgebra as LA
using DataFrames
using CSV
# using Plots
using .DataTypes
using .DSForces
using .CoefLoader
using .MathHelpers
using .PrintingUtils
using .SimCore
using .Autopilot

datcom_interp = load_datcom("input file path datcom matlab type")
wind_interp = load_windtunnel("input file path matlab type")
const trims = DataFrame(CSV.File("input file path"))

const atmos = AtmoshpereState(1.225, 9.80665)

const mass::Float64 = 1.0 #kg
const inertia = Float64[1.0 0    0
                        0   1.0 0
                        0   0    1.0]

const geom = DSGeometry(1.0, 1.0, 1.0, [-1.0, 0, 0], [-0.5, 0, 0], [0, 0, 0])
const aeroparams = DSAeroParams(-0.01)


const V = 30
const alpha = deg2rad(5) 
const linear_speeds = Float64[V*cos(alpha), 0, V*sin(alpha)]
const angular_speeds = [0, 0, 0]
const quaternion = FM.euler2quaternion(0.0, alpha, 0.0)
const position = [0, 0, 0]
const state_init = Float64[linear_speeds; angular_speeds; quaternion; position]
const state_dot_init = zeros(13)

const DS_init = DSAircraft(geom, aeroparams, mass, inertia)

const dt = 0.001
const t_run = 20
const no_steps::Integer = trunc(t_run ÷ dt)


function main()

    state_list, state_dot_list, controls_list, t_list, ap_list = run_flight_sim_1(state_init, state_dot_init, datcom_interp, wind_interp, atmos, DS_init, no_steps, dt, fixed, trims)

    simtit = replace(string(t_run), "." => ";") * "s_flight" 
    plot_flight_multi_wControls(state_list, state_dot_list, controls_list, dt, "plots/v1/" * simtit, simtit)

    arr_to_csv = [permutedims(t_list, (2,1)) permutedims(state_list, (2,1)) permutedims(state_dot_list, (2,1)) permutedims(controls_list, (2,1)) permutedims(ap_list, (2,1))]
    to_csv = DataFrame(arr_to_csv, [:t, :u, :v, :w, :p, :q, :r, :q0, :q1, :q2, :q3, :xe, :ye, :ze,#=
                                      =#:du, :dv, :dw, :dp, :dq, :dr, :dq0, :dq1, :dq2, :dq3, :dxe, :dye, :dze,#=
                                      =#:canard, :thrust, :aileronL, :aileronR, :q_des])
    CSV.write("flight_saves/v1/"*simtit*".csv", to_csv)
end

main()
