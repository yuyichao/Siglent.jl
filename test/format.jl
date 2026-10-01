#

using Test

import Siglent as S

@testset "Argument formatting" begin
    @test S._arg(true) == "ON"
    @test S._arg(false) == "OFF"
    @test S._arg(5) == "5"
    @test S._arg(-3) == "-3"
    @test S._arg(UInt8(7)) == "7"
    @test S._arg(2e9) == "2.0E9"
    @test S._arg(-1.234567e-7) == "-1.234567E-7"
    @test S._arg(0.5) == "0.5"
    @test S._arg(1f-7) == "1.0E-7"
    @test S._arg(2f9) == "2.0E9"
    @test S._arg(Float16(0.5)) == "0.5"
    @test S._arg(1 // 4) == "0.25"
    @test S._arg(π) == "3.141592653589793"
    # full precision survives the round trip
    for x in (1 / 3, 1.2345678901234567e-9, -9.87654321e12)
        @test parse(Float64, S._arg(x)) === x
    end
    @test S._arg(:CUSTom) == "CUSTom"
    @test S._arg("C1") == "C1"
    @test S._arg(SubString("xC2", 2)) == "C2"
    @test S._arg((1, :X, true)) == "1,X,ON"
    @test S._arg([0.5, 2]) == "0.5,2.0"
    @test S._arg(()) == ""
end

@testset "Quoting" begin
    @test S._quote("abc") == "\"abc\""
    @test S._quote("\"abc\"") == "\"abc\""
    @test S._quote(:abc) == "\"abc\""
    @test S._quote(nothing) === nothing
    @test S._quote(("EXTernal", "\"a.xml\"")) == ("EXTernal", "\"a.xml\"")
end

@testset "Command assembly" begin
    @test S._cmd(":TRIGger:STOP") == ":TRIGger:STOP"
    @test S._cmd(":A", nothing) == ":A"
    @test S._cmd(":A", 1) == ":A 1"
    @test S._cmd(":A", 1, nothing, 2) == ":A 1,2"
    @test S._cmd(":A", :X, "LOAD", 50) == ":A X,LOAD,50"
    @test S._cmd(":A", (1, 2), true) == ":A 1,2,ON"
    @test S._cmd(SubString("x:A", 2), 1) == ":A 1"
end

@testset "Response parsing" begin
    @test S._parse_int("1000") === 1000
    @test S._parse_int("-5") === -5
    @test S._parse_int("1.00E+03") === 1000
    @test S._parse_int("1.5") == "1.5"
    @test S._parse_int("****") == "****"
    @test S._parse_float("2.00E+09") === 2e9
    @test S._parse_float("-1.5") === -1.5
    @test S._parse_float("12") === 12.0
    @test S._parse_float("****") == "****"
    @test S._parse_bool("ON") === true
    @test S._parse_bool("on") === true
    @test S._parse_bool("1") === true
    @test S._parse_bool("OFF") === false
    @test S._parse_bool("0") === false
    @test S._parse_bool("MAYBE") == "MAYBE"
end
