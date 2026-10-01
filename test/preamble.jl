#

using Test

using Siglent

include("preamble_data.jl")

@testset "Preamble fields" begin
    raw = make_preamble()
    p = parse_preamble(raw)
    @test p isa WaveformPreamble
    @test p.raw == raw
    @test p.raw !== raw
    @test p.descriptor_name == "WAVEDESC"
    @test p.template_name == "WAVEACE"
    @test p.instrument_name == "Siglent SDS"
    @test p.comm_type == 1
    @test p.comm_order == 0
    @test p.wave_desc_length == 346
    @test p.wave_array_1 == 2000
    @test p.wave_array_count == 1000
    @test p.first_point == 3
    @test p.data_interval == 2
    @test p.read_frames == 10
    @test p.sum_frames == 500
    @test p.vertical_gain === 0.5
    @test p.vertical_offset === -0.25
    @test p.code_per_div === 30.0
    @test p.adc_bit == 12
    @test p.frame_index == 4
    @test p.horiz_interval === Float64(5f-10)
    @test p.horiz_offset === -1.5e-7
    @test p.timebase_index == 12
    @test p.coupling === :AC
    @test p.probe === 10.0
    @test p.fixed_vgain_index == 7
    @test p.bandwidth_limit === Symbol("20M")
    @test p.wave_source == "C3"
end

@testset "Preamble enums" begin
    @test parse_preamble(make_preamble(coupling=0)).coupling === :DC
    @test parse_preamble(make_preamble(coupling=2)).coupling === :GND
    @test parse_preamble(make_preamble(coupling=9)).coupling === Symbol("9")
    @test parse_preamble(make_preamble(bwlimit=0)).bandwidth_limit === :OFF
    @test parse_preamble(make_preamble(bwlimit=2)).bandwidth_limit === Symbol("200M")
    @test parse_preamble(make_preamble(source=0)).wave_source == "C1"
    @test parse_preamble(make_preamble(source=7)).wave_source == "C8"
end

@testset "Preamble strings" begin
    raw = make_preamble()
    # text after the NUL terminator is ignored
    put_str!(raw, 16, "WAVEACE\0junk")
    @test parse_preamble(raw).template_name == "WAVEACE"
    # a field that fills all 16 bytes has no NUL
    put_str!(raw, 76, "0123456789abcdef")
    @test parse_preamble(raw).instrument_name == "0123456789abcdef"
end

@testset "Preamble validation" begin
    @test length(parse_preamble(make_preamble(len=400)).raw) == 400
    @test parse_preamble(view(make_preamble(), :)).sum_frames == 500
    @test_throws ErrorException parse_preamble(make_preamble()[1:345])
    @test_throws ErrorException parse_preamble(UInt8[])
    @test_throws ErrorException parse_preamble(make_preamble(name="GARBAGE!"))
end

@testset "Preamble show" begin
    str = sprint(show, MIME"text/plain"(), parse_preamble(make_preamble()))
    @test startswith(str, "WaveformPreamble:")
    @test occursin("descriptor_name    = \"WAVEDESC\"", str)
    @test occursin("sum_frames         = 500", str)
    @test occursin("raw                = 346 bytes", str)
    @test !occursin("UInt8[", str)
end
