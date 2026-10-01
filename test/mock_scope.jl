#

# A fake scope on a local TCP port: records every line it receives and answers the
# ones listed in `responses` (a `String` is sent with a trailing "\n", a byte vector
# is sent as is, a function is called with the connection). Other queries get
# `default` if it is set, and no reply otherwise.

using Sockets
using Siglent

mutable struct MockScope
    server::Sockets.TCPServer
    port::Int
    received::Vector{String}
    responses::Dict{String,Any}
    default::Union{Nothing,String}
end

function MockScope(responses=Dict{String,Any}(); default=nothing)
    port, server = listenany(ip"127.0.0.1", 50000)
    m = MockScope(server, Int(port), String[],
                  Dict{String,Any}("*IDN?" => "Siglent Technologies,SDS804X HD,SN0001,1.0",
                                   "*OPC?" => "1"),
                  default)
    merge!(m.responses, responses)
    @async try
        c = accept(server)
        try
            while !eof(c)
                line = readline(c)
                push!(m.received, line)
                r = get(m.responses, line, nothing)
                if r === nothing && occursin('?', line)
                    r = m.default
                end
                if r isa AbstractString
                    write(c, r, "\n")
                elseif r isa AbstractVector{UInt8}
                    write(c, r)
                elseif r !== nothing
                    r(c)
                end
            end
        finally
            close(c)
        end
    catch e
        e isa Base.IOError || rethrow()
    end
    return m
end

function connect_mock(m::MockScope; kws...)
    return SiglentScope("127.0.0.1", m.port; timeout=5, kws...)
end

function Base.close(m::MockScope)
    close(m.server)
end

"""Run `f()`, then return the lines the mock received while it ran."""
function wire(f, m::MockScope, s)
    n = length(m.received)
    f()
    get_opc(s)                      # round trip, so every earlier line has arrived
    return m.received[n + 1:end - 1]
end

"""Mock + connected scope for the duration of `f(m, s)`."""
function with_mock(f, responses=Dict{String,Any}(); default=nothing, kws...)
    m = MockScope(responses; default)
    s = connect_mock(m; kws...)
    try
        f(m, s)
    finally
        close(s)
        close(m)
    end
end
