module PrintingUtils
using ..DataTypes
using Plots
import FlightMechanics as FM
import LinearAlgebra as LA
export print_state, print_state_dot, plot_flight_multi, plot_flight_multi_wControls

function print_state(state::Vector{Float64})
    speeds_lin = state[1:3]
    speeds_ang = state[4:6]
    position = state[11:13]
    angles = FM.quaternion2euler(state[7],state[8],state[9],state[10])

    println("\t\tlin speed: ", speeds_lin)
    println("\t\tang speed: ", speeds_ang)
    println("\t\tyaw: ", angles[1], "\tpitch: ", angles[2], "\troll: ", angles[3])
    println("\t\tposition: ", position)
end

function print_state_dot(state::Vector{Float64})
    speeds_lin = state[1:3]
    speeds_ang = state[4:6]
    quat = state[7:10]
    position = state[11:13]

    println("\t\tlin accel: ", speeds_lin)
    println("\t\tang accel: ", speeds_ang)
    println("\t\tquaternion dot: ", quat)
    println("\t\tposition dot: ", position)
end

function plot_flight_multi(state_list, state_dot_list, dt, path, title="")
    x = state_list[end-2, :]
    y = state_list[end-1, :]
    z = state_list[end, :]
    p = rad2deg.(state_list[4, :])
    q = rad2deg.(state_list[5, :])
    r = rad2deg.(state_list[6, :])
    attitude_list = FM.quaternion2euler.(state_list[7, :],state_list[8, :],state_list[9, :],state_list[10, :])
    attitude_list = rad2deg.(reduce(hcat, attitude_list))
    pitch = attitude_list[2, :]
    roll = attitude_list[3, :]
    yaw = attitude_list[1, :]
    alpha = rad2deg.(atan.(state_list[3, :], state_list[1, :]))
    V = map(LA.norm, eachcol(state_list[1:3, :]))

    stop = dt*(size(x)[1]-1)
    t = range(0, stop, length=size(x)[1])

    xz_plot = scatter(x, z, yflip=true, xflip=false, xlabel="x", ylabel="z")
    xy_plot = scatter(x, y, xlabel="x", ylabel="y")
    pitch_plot = scatter(t, pitch, xlabel="t", ylabel="pitch [deg]")
    yaw_plot = scatter(t, yaw, xlabel="t", ylabel="yaw [deg]")
    roll_plot = scatter(t, roll, xlabel="t", ylabel="roll [deg]")
    pitch_rate_plot = scatter(t, q, xlabel="t", ylabel="pitch rate[deg/s]")
    yaw_rate_plot = scatter(t, p, xlabel="t", ylabel="yaw rate[deg/s]")
    roll_rate_plot = scatter(t, r, xlabel="t", ylabel="roll rate[deg/s]")
    alpha_plot = scatter(t, alpha, xlabel="t", ylabel="alpha [deg]")
    V_plot = scatter(t, V, xlabel="t", ylabel="V [m/s]")

    plot(xz_plot, xy_plot, pitch_plot, yaw_plot, roll_plot, pitch_rate_plot, yaw_rate_plot, roll_rate_plot, alpha_plot, V_plot,
         layout=(10,1), legend=false, size=(600, 2400), left_margin=25Plots.mm, right_margin=10Plots.mm, title=title)
    savefig(path)
end

function plot_flight_multi_wControls(state_list, state_dot_list, controls_list, dt, path, title="")
    x = state_list[end-2, :]
    y = state_list[end-1, :]
    z = state_list[end, :]
    p = rad2deg.(state_list[4, :])
    q = rad2deg.(state_list[5, :])
    r = rad2deg.(state_list[6, :])
    attitude_list = FM.quaternion2euler.(state_list[7, :],state_list[8, :],state_list[9, :],state_list[10, :])
    attitude_list = rad2deg.(reduce(hcat, attitude_list))
    pitch = attitude_list[2, :]
    roll = attitude_list[3, :]
    yaw = attitude_list[1, :]
    alpha = rad2deg.(atan.(state_list[3, :], state_list[1, :]))
    V = map(LA.norm, eachcol(state_list[1:3, :]))

    canard = controls_list[1, :]
    thrust = controls_list[2, :]

    stop = dt*(size(x)[1]-1)
    t = range(0, stop, length=size(x)[1])

    xz_plot = scatter(x, z, yflip=true, xflip=false, xlabel="x", ylabel="z")
    xy_plot = scatter(x, y, xlabel="x", ylabel="y")
    pitch_plot = scatter(t, pitch, xlabel="t", ylabel="pitch [deg]")
    yaw_plot = scatter(t, yaw, xlabel="t", ylabel="yaw [deg]")
    roll_plot = scatter(t, roll, xlabel="t", ylabel="roll [deg]")
    pitch_rate_plot = scatter(t, q, xlabel="t", ylabel="pitch rate[deg/s]")
    yaw_rate_plot = scatter(t, p, xlabel="t", ylabel="yaw rate[deg/s]")
    roll_rate_plot = scatter(t, r, xlabel="t", ylabel="roll rate[deg/s]")
    alpha_plot = scatter(t, alpha, xlabel="t", ylabel="alpha [deg]")
    V_plot = scatter(t, V, xlabel="t", ylabel="V [m/s]")

    canard_plot = scatter(t, canard, xlabel="t", ylabel="Canard")
    thrust_plot = scatter(t, thrust, xlabel="t", ylabel="Thrust [N]")

    plot(xz_plot, xy_plot, pitch_plot, yaw_plot, roll_plot, pitch_rate_plot, yaw_rate_plot, roll_rate_plot, alpha_plot, V_plot, canard_plot, thrust_plot,
         layout=(12,1), legend=false, size=(600, 2880), left_margin=25Plots.mm, right_margin=10Plots.mm, title=title)
    savefig(path)
end

end