#

using Test

using Siglent

include("mock_scope.jl")

include("preamble_data.jl")

@testset "Command strings" begin
    with_mock() do m, s
        @test wire(() -> rst!(s), m, s) == ["*RST"]
        @test wire(() -> autoset!(s), m, s) == [":AUToset"]
        @test wire(() -> trigger_stop!(s), m, s) == [":TRIGger:STOP"]
        @test wire(() -> acquire_csweep!(s), m, s) == [":ACQuire:CSWeep"]
        @test wire(() -> system_selfcal!(s), m, s) == [":SYSTem:SELFCal"]
        @test wire(() -> set_acquire_srate!(s, 2e9), m, s) == [":ACQuire:SRATe 2.0E9"]
        @test wire(() -> set_timebase_delay!(s, -1.234567e-7), m, s) ==
            [":TIMebase:DELay -1.234567E-7"]
        @test wire(() -> set_acquire_sequence_count!(s, 1000), m, s) ==
            [":ACQuire:SEQuence:COUNt 1000"]
        @test wire(() -> set_trigger_mode!(s, :SINGle), m, s) == [":TRIGger:MODE SINGle"]
        # index arguments
        @test wire(() -> set_channel_scale!(s, 2, 0.5), m, s) == [":CHANnel2:SCALe 0.5"]
        @test wire(() -> set_channel_switch!(s, 1, true), m, s) == [":CHANnel1:SWITch ON"]
        @test wire(() -> set_channel_switch!(s, 4, false), m, s) == [":CHANnel4:SWITch OFF"]
        @test wire(() -> set_function_scale!(s, 3, 1), m, s) == [":FUNCtion3:SCALe 1"]
        @test wire(() -> set_decode_bus_uart_baud!(s, 2, "9600bps"), m, s) ==
            [":DECode:BUS2:UART:BAUD 9600bps"]
        @test wire(() -> set_ref_data_position!(s, "A", 0.1), m, s) ==
            [":REFA:DATA:POSition 0.1"]
        @test wire(() -> set_measure_advanced_p!(s, 5, true), m, s) ==
            [":MEASure:ADVanced:P5 ON"]
        # optional arguments
        @test wire(() -> set_format_data!(s, :CUSTom, 5), m, s) == [":FORMat:DATA CUSTom,5"]
        @test wire(() -> set_format_data!(s, :SINGle), m, s) == [":FORMat:DATA SINGle"]
        @test wire(() -> set_channel_probe!(s, 1, :CUSTom, 10), m, s) ==
            [":CHANnel1:PROBe CUSTom,10"]
        @test wire(() -> set_cursor_mitem!(s, :DELay, :C1), m, s) ==
            [":CURSor:MITem DELay,C1"]
        @test wire(() -> set_cursor_mitem!(s, :DELay, :C1, :C2), m, s) ==
            [":CURSor:MITem DELay,C1,C2"]
        # multiple and repeated arguments
        @test wire(() -> set_waveform_sequence!(s, 0, 1), m, s) == [":WAVeform:SEQuence 0,1"]
        @test wire(() -> set_measure_threshold_absolute!(s, 1.5, 1, 0.5), m, s) ==
            [":MEASure:THReshold:ABSolute 1.5,1,0.5"]
        @test wire(() -> set_digital_bus_map!(s, 1, :D0, :D1, :D2), m, s) ==
            [":DIGital:BUS1:MAP D0,D1,D2"]
        @test wire(() -> set_trigger_spi_data!(s, 1, 0, :X), m, s) ==
            [":TRIGger:SPI:DATA 1,0,X"]
        # quoted strings are quoted once
        @test wire(() -> set_digital_label!(s, 3, "abc"), m, s) == [":DIGital:LABel3 \"abc\""]
        @test wire(() -> set_channel_label_text!(s, 1, "\"ch\""), m, s) ==
            [":CHANnel1:LABel:TEXT \"ch\""]
        @test wire(() -> save_csv!(s, "local/a.csv", :C1, false), m, s) ==
            [":SAVE:CSV \"local/a.csv\",C1,OFF"]
        # compound value passed as a tuple
        @test wire(() -> save_setup!(s, (:EXTernal, "\"local/a.xml\"")), m, s) ==
            [":SAVE:SETup EXTernal,\"local/a.xml\""]
    end
end

@testset "WGEN commands" begin
    with_mock(; default="0") do m, s
        @test wire(() -> set_wgen_output!(s, true, 50), m, s) == ["C1:OUTPut ON,LOAD,50"]
        @test wire(() -> set_wgen_arbwave_index!(s, 2), m, s) == ["C1:ARbWaVe INDEX,2"]
        @test wire(() -> set_wgen_arbwave_name!(s, "wave_1"), m, s) ==
            ["C1:ARbWaVe NAME,wave_1"]
        @test wire(() -> set_wgen_basic_wave!(s, :FRQ, 2000), m, s) ==
            ["C1:BaSic_WaVe FRQ,2000"]
        @test wire(() -> set_wgen_sync!(s, true; channel="C2"), m, s) == ["C2:SYNC ON"]
        s.wgen_channel = "C2"
        @test wire(() -> set_wgen_sync!(s, false), m, s) == ["C2:SYNC OFF"]
        @test wire(() -> get_wgen_output(s), m, s) == ["C2:OUTPut?"]
        # commands without a channel prefix
        @test wire(() -> set_wgen_voltprt!(s, true), m, s) == ["VOLTPRT ON"]
        @test wire(() -> get_wgen_storelist(s), m, s) == ["SToreList?"]
        @test wire(() -> get_wgen_storelist(s, :USER), m, s) == ["SToreList? USER"]
    end
    with_mock(; wgen_channel="C3") do m, s
        @test wire(() -> set_wgen_sync!(s, true), m, s) == ["C3:SYNC ON"]
    end
end

@testset "Query strings" begin
    with_mock(; default="0") do m, s
        q(f) = wire(f, m, s)
        @test q(() -> get_acquire_srate(s)) == [":ACQuire:SRATe?"]
        @test q(() -> get_channel_scale(s, 2)) == [":CHANnel2:SCALe?"]
        @test q(() -> get_measure_simple_value(s, :PKPK)) == [":MEASure:SIMPle:VALue? PKPK"]
        @test q(() -> get_measure_advanced_p_statistics(s, 2, :MEAN)) ==
            [":MEASure:ADVanced:P2:STATistics? MEAN"]
        @test q(() -> get_trigger_pattern_level(s, :C1)) == [":TRIGger:PATTern:LEVel? C1"]
        @test q(() -> get_system_edumode(s)) == [":SYSTem:EDUMode?"]
        @test q(() -> get_system_edumode(s, :MEASure)) == [":SYSTem:EDUMode? MEASure"]
        @test q(() -> get_function_fft_span(s, 1)) == [":FUNCtion1:FFT:SPAN?"]
        @test q(() -> get_memory_switch(s, 2)) == [":MEMory2:SWITch?"]
        # paths that are misprinted in the guide follow the section heading
        @test q(() -> get_trigger_spi_latchedge(s)) == [":TRIGger:SPI:LATChedge?"]
        @test q(() -> get_decode_bus_manchester_polarity(s, 1)) ==
            [":DECode:BUS1:MANChester:POLarity?"]
        @test q(() -> get_decode_bus_spi_clkthreshold(s, 2)) ==
            [":DECode:BUS2:SPI:CLKThreshold?"]
        @test q(() -> get_digital_label(s, 3)) == [":DIGital:LABel3?"]
        @test q(() -> get_trigger_runt_tupper(s)) == [":TRIGger:RUNT:TUPPer?"]
        @test q(() -> get_trigger_iis_latchedge(s)) == [":TRIGger:IIS:LATChedge?"]
    end
end

@testset "Preamble query" begin
    raw = make_preamble()
    with_mock(Dict(":WAVeform:PREamble?" =>
                   vcat(codeunits("DESC,#9000000346"), raw, codeunits("\n\n")),
                   ":WAVeform:DATA?" => vcat(codeunits("#14"), UInt8[1, 0, 2, 0],
                                             codeunits("\n\n")))) do m, s
        p = get_waveform_preamble(s)
        @test p isa WaveformPreamble
        @test p.raw == raw
        @test p.sum_frames == 500
        @test get_waveform_data(s) == UInt8[1, 0, 2, 0]
        @test get_opc(s) == "1"
    end
end

@testset "Generated API" begin
    fns = [n for n in names(Siglent) if n !== :Siglent && getfield(Siglent, n) isa Function]
    cmds = [n for n in fns if endswith(string(n), '!')]
    getters = [n for n in fns if startswith(string(n), "get_")]
    @test length(cmds) == 572
    @test length(getters) == 568
    @test isempty(intersect(cmds, getters))
    # everything else is the hand-written low-level interface
    @test sort(setdiff(fns, cmds, getters)) ==
        [:parse_preamble, :scpi_query, :scpi_query_block, :scpi_write]
    for n in cmds
        @test !startswith(string(n), "get_")
    end
    for n in getters
        @test !endswith(string(n), '!')
    end
    # every wrapper documents its SCPI syntax and the page in the guide
    meta = Base.Docs.meta(Siglent)
    function docstring(n)
        b = Base.Docs.Binding(Siglent, n)
        haskey(meta, b) || return ""
        return join((join(string.(d.text)) for d in values(meta[b].docs)), '\n')
    end
    undocumented = [n for n in vcat(cmds, getters) if !occursin("(guide PDF p. ", docstring(n))]
    @test isempty(undocumented)
end
