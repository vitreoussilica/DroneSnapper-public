module MathHelpers
export rungekutta4, rg4flight

function rungekutta4(f, y0, t)
    n = length(t)
    y = zeros((n, length(y0)))
    y[1,:] = y0
    for i in 1:n-1
        h = t[i+1] - t[i]
        k1 = f(y[i,:], t[i])
        k2 = f(y[i,:] .+ k1 * h/2, t[i] + h/2)
        k3 = f(y[i,:] .+ k2 * h/2, t[i] + h/2)
        k4 = f(y[i,:] .+ k3 * h, t[i] + h)
        y[i+1,:] = y[i,:] .+ (h/6) * (k1 + 2*k2 + 2*k3 + k4)
    end
    return y
end

function rg4flight(f, dt, state)
    k1 = f(state)
    k2 = f(state+(k1*dt/2))
    k3 = f(state+(k2*dt/2))
    k4 = f(state+(k3*dt))
    state_dot = (k1 + 2*k2 + 2*k3 + k4)/6
    new_state = state+ dt * state_dot
    return new_state, state_dot
end
end