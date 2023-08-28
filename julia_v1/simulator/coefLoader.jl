module CoefLoader
using MAT
using Interpolations
export load_datcom, load_windtunnel

function load_datcom(filepath)
    datcom = matread(filepath)
    data = datcom["aerody_data"]["Delta0"]
    names = String[ "clq", "cmq", "cyb", "cnb", "clb", "clp", "cyp", "cnp", "cnr", "clr"]
    aoa = dropdims(data["alpha"]; dims=1)
    interpolants = Dict()

    for i in names
        var = dropdims(data[i]; dims=1)
        itp = interpolate((aoa,), var, Gridded(Linear()))
        f_itp(z) = itp(z)
        interpolants[i] = f_itp
    end
    return interpolants
end

function load_windtunnel(filepath)
    windtun = matread(filepath)
    aerodata = windtun["ac_aerodynamic_data"]["averaged_coeffs"]
    
    delta = dropdims(windtun["delta"]; dims=1)
    alpha = -5:5:20
    vel = 15:5:30

    names = Dict("CL" => "CL", "CD" => "CD", "CM" => "Cm")
    interpolants = Dict()

    for (key, val) in names
        var = aerodata[val]
        itp = interpolate((alpha, delta, vel), var, Gridded(Linear()))
        ext = extrapolate(itp, Line())
        fext(a, b, c) = ext(a, b, c)

        interpolants[key] = fext
    end
    return interpolants
end

end
