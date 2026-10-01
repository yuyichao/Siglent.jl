#

using Test

using Siglent

include("mock_scope.jl")

const BMP = vcat(UInt8['B', 'M'], reinterpret(UInt8, [htol(UInt32(20))]), fill(0x11, 14))

function png_chunk(type, data)
    return vcat(reinterpret(UInt8, [hton(UInt32(length(data)))]),
                codeunits(type), data, zeros(UInt8, 4))
end
const PNG = vcat(UInt8[0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a],
                 png_chunk("IHDR", UInt8[1, 2, 3]),
                 png_chunk("IDAT", fill(0x0a, 5)),   # newlines inside the image are data
                 png_chunk("IEND", UInt8[]))

@testset "Connection" begin
    with_mock() do m, s
        @test s.idn == "Siglent Technologies,SDS804X HD,SN0001,1.0"
        @test m.received == ["*IDN?"]
        @test isopen(s)
        @test s.timeout == 5
        @test s.wgen_channel == "C1"
        @test !s.verbose
        close(s)
        @test !isopen(s)
    end
    with_mock(; pollint=0.002, trailer_timeout=0.05, wgen_channel="C2") do m, s
        @test s.pollint == 0.002
        @test s.trailer_timeout == 0.05
        @test s.wgen_channel == "C2"
    end
end

@testset "Write and query" begin
    with_mock(Dict("PAD?" => "  padded \r", "EMPTY?" => "")) do m, s
        @test wire(m, s) do
            scpi_write(s, ":TRIGger:STOP")
        end == [":TRIGger:STOP"]
        @test scpi_query(s, "PAD?") == "padded"
        @test scpi_query(s, "EMPTY?") == ""
        @test scpi_query(s, "*OPC?") == "1"
    end
end

@testset "Verbose" begin
    with_mock(; verbose=true) do m, s
        out = mktemp() do path, io
            redirect_stdout(io) do
                scpi_write(s, ":ACQuire:CSWeep")
            end
            flush(io)
            read(path, String)
        end
        @test out == ":ACQuire:CSWeep\n"
    end
end

@testset "Binary block" begin
    responses = Dict{String,Any}(
        "PREFIX?" => vcat(codeunits("DESC,#14"), UInt8[1, 2, 3, 4], codeunits("\n")),
        "DOUBLE?" => vcat(codeunits("#9000000003"), UInt8[0x0a, 0x00, 0x0a], codeunits("\n\n")),
        "EMPTYBLK?" => codeunits("#10\n"),
        "AFTER?" => "after")
    with_mock(responses) do m, s
        @test scpi_query_block(s, "PREFIX?") == UInt8[1, 2, 3, 4]
        @test scpi_query(s, "AFTER?") == "after"
        # payload may contain newlines; a "\n\n" terminator must not leave a stray line
        @test scpi_query_block(s, "DOUBLE?") == UInt8[0x0a, 0x00, 0x0a]
        @test scpi_query(s, "AFTER?") == "after"
        @test scpi_query_block(s, "EMPTYBLK?") == UInt8[]
        @test scpi_query(s, "AFTER?") == "after"
    end
end

@testset "Image" begin
    responses = Dict{String,Any}(
        ":PRINt? BMP" => vcat(BMP, codeunits("\n")),
        ":PRINt? PNG" => PNG,                         # no terminator at all
        "BLOCKIMG?" => vcat(codeunits("#15"), PNG[1:5], codeunits("\n")),
        "AFTER?" => "after")
    with_mock(responses; trailer_timeout=0.05) do m, s
        @test get_print(s, "BMP") == BMP
        @test scpi_query(s, "AFTER?") == "after"
        @test get_print(s, :PNG) == PNG
        @test scpi_query(s, "AFTER?") == "after"
        @test Siglent._query_image(s, "BLOCKIMG?") == PNG[1:5]
        @test scpi_query(s, "AFTER?") == "after"
    end
    with_mock(Dict("BAD?" => "xyz")) do m, s
        @test_throws ErrorException Siglent._query_image(s, "BAD?")
    end
end

@testset "Timeout" begin
    with_mock(; timeout=0.3) do m, s
        t0 = time()
        err = try
            scpi_query(s, "NOREPLY?")
            nothing
        catch e
            e
        end
        @test err isa ErrorException
        @test occursin("Timed out after 0.3 s", err.msg)
        @test time() - t0 < 3
        # the socket is closed so a late reply cannot be mistaken for a later one
        @test !isopen(s)
    end
    with_mock(Dict("SHORT?" => codeunits("#210abc")); timeout=0.3) do m, s
        @test_throws ErrorException scpi_query_block(s, "SHORT?")
    end
end

@testset "Connection lost" begin
    m = MockScope(Dict{String,Any}("BYE?" => c -> (write(c, "#15ab"); close(c))))
    s = connect_mock(m)
    try
        err = try
            scpi_query_block(s, "BYE?")
            nothing
        catch e
            e
        end
        @test err isa Exception
        @test occursin("Connection closed after 2 of 5 bytes", sprint(showerror, err))
    finally
        close(s)
        close(m)
    end
end

@testset "Typed queries" begin
    responses = Dict{String,Any}(
        ":ACQuire:SRATe?" => "2.00E+09",
        ":ACQuire:SEQuence:COUNt?" => "1000",
        ":CHANnel2:SWITch?" => "ON",
        ":CHANnel3:SWITch?" => "OFF",
        ":TRIGger:MODE?" => "SINGle",
        ":TRIGger:STATus?" => "Stop",
        ":MEASure:SIMPle:VALue? PKPK" => "****",
        ":TIMebase:SCALe?" => "1.00E-07")
    with_mock(responses) do m, s
        @test get_idn(s) == s.idn
        @test get_acquire_srate(s) === 2e9
        @test get_acquire_sequence_count(s) === 1000
        @test get_channel_switch(s, 2) === true
        @test get_channel_switch(s, 3) === false
        @test get_trigger_mode(s) == "SINGle"
        @test get_trigger_status(s) == "Stop"
        @test get_timebase_scale(s) === 1e-7
        # values that do not parse fall back to the raw string
        @test get_measure_simple_value(s, :PKPK) == "****"
    end
end
