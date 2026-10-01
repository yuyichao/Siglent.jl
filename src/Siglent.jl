"""
Siglent.jl -- Low-level SCPI command/query wrappers for Siglent SDS series oscilloscopes.

Pure Julia: talks SCPI to the scope over a raw TCP socket (port 5025 by default).
Every command and query in the Siglent "SDS Series Programming Guide" (EN11D, Feb 2023)
has a thin wrapper here. Not every model implements every command (e.g. DIGital needs
the MSO option, WGEN the function generator); unsupported ones are rejected by the scope.

    using Siglent

    s = SiglentScope("192.168.1.50")
    set_acquire_srate!(s, 2e9)
    get_acquire_srate(s)               # -> Float64
    set_channel_switch!(s, 1, true)    # :CHANnel1:SWITch ON
    get_channel_switch(s, 1)           # -> Bool
    trigger_stop!(s)                   # :TRIGger:STOP
    pre = get_waveform_preamble(s)     # -> Vector{UInt8}
    close(s)

Naming
------
The function name is the long-form SCPI path, lowercased, with `:` replaced by `_`
and index placeholders dropped (`:CHANnel<n>:SCALe` -> `channel_scale`).

* `set_<name>!(s, idx..., args...)` -- command form of a Command/Query entry.
* `get_<name>(s, idx..., args...)`  -- query form.
* `<name>!(s, idx..., args...)`     -- command-only entries and commands with no
  argument (`acquire_csweep!`, `trigger_stop!`, `save_setup!`, `rst!`).

Index placeholders in the path (`CHANnel<n>`, `FUNCtion<n>`, `BUS<n>`, `REF<r>`, ...)
become the leading positional arguments, in path order. WGEN commands take the output
as a keyword, `channel`, defaulting to the `wgen_channel` given to the constructor
(`"C1"` unless set), and are prefixed with `wgen_`.

Arguments
---------
Arguments are formatted by `_arg`: `Bool` -> `ON`/`OFF`, integers as is, floats in
shortest round-trip exponent form (`2.0E9`), strings and `Symbol`s verbatim, tuples/vectors joined by `,`. Parameters the
guide documents as quoted strings are quoted automatically. Optional arguments
default to `nothing`, which drops them from the command.

Return values
-------------
Queries documented as NR1 return `Int`, NR2/NR3 return `Float64`, `{ON|OFF}` returns
`Bool`, `:WAVeform:PREamble?` returns a decoded `WaveformPreamble`, other binary
blocks return `Vector{UInt8}`, and everything else returns the stripped
response `String`. If a typed response does not parse (e.g. `****` for an invalid
measurement), the raw `String` is returned instead.

The guide's own typos (query path differing from the section heading) are resolved in
favor of the section heading, which matches the guide's examples. The multimeter
(METEr) commands are omitted; they exist only on the SHS800X/SHS1000X handhelds.
"""
module Siglent

using Sockets

export SiglentScope, scpi_write, scpi_query, scpi_query_block, WaveformPreamble, parse_preamble

const DEFAULT_PORT = 5025           # Siglent raw SCPI socket

mutable struct SiglentScope
    const sock::TCPSocket
    timeout::Float64                # seconds, per read
    pollint::Float64                # seconds, polling interval while waiting for a read
    trailer_timeout::Float64        # seconds to wait for a terminator after an image
    verbose::Bool                   # echo every command sent
    wgen_channel::String            # default output for the wgen_* functions
    idn::String
end

"""
    SiglentScope(host, port=$DEFAULT_PORT; timeout=30.0, pollint=0.01, trailer_timeout=0.1,
            verbose=false, wgen_channel="C1")

Connect to the scope at `host:port` and read its `*IDN?` into `s.idn`.
"""
function SiglentScope(host::AbstractString, port::Integer=DEFAULT_PORT;
                 timeout=30.0, pollint=0.01, trailer_timeout=0.1,
                 verbose=false, wgen_channel="C1")
    sock = connect(host, port)
    Sockets.nagle(sock, false)
    s = SiglentScope(sock, timeout, pollint, trailer_timeout, verbose, wgen_channel, "")
    s.idn = scpi_query(s, "*IDN?")
    return s
end

Base.close(s::SiglentScope) = (close(s.sock); nothing)
Base.isopen(s::SiglentScope) = isopen(s.sock)

# ---------------------------------------------------------------------- #
# Low-level socket I/O                                                   #
# ---------------------------------------------------------------------- #
"""Run `f()` on another task; error if it does not finish within `s.timeout` seconds."""
function _with_timeout(f, s::SiglentScope, what)
    task = @async f()
    if timedwait(() -> istaskdone(task), s.timeout; pollint=s.pollint) == :timed_out
        close(s.sock)
        error("Timed out after $(s.timeout) s waiting for $what")
    end
    return fetch(task)
end

_read_line(s::SiglentScope) = _with_timeout(s, "response line") do
    String(readuntil(s.sock, UInt8('\n'); keep=false))
end

_read_exact(s::SiglentScope, n) = _with_timeout(s, "$n bytes") do
    buf = read(s.sock, n)
    length(buf) == n || error("Connection closed after $(length(buf)) of $n bytes")
    buf
end

"""Send one SCPI command (newline terminated)."""
function scpi_write(s::SiglentScope, cmd)
    s.verbose && println(cmd)
    write(s.sock, cmd, '\n')
    flush(s.sock)
    nothing
end

"""Send a query and return the stripped response line."""
function scpi_query(s::SiglentScope, cmd)
    scpi_write(s, cmd)
    return String(strip(_read_line(s)))
end

"""Send a query and return the payload of its IEEE 488.2 `#<n><len>` binary block."""
function scpi_query_block(s::SiglentScope, cmd)
    scpi_write(s, cmd)
    # skip anything before '#' (Siglent may prefix e.g. "DESC,")
    _with_timeout(s, "block header") do
        readuntil(s.sock, UInt8('#'); keep=false)
    end
    n_digits = Int(_read_exact(s, 1)[1] - UInt8('0'))
    len = parse(Int, String(_read_exact(s, n_digits)))
    payload = _read_exact(s, len)
    # consume the trailing terminator (usually "\n", sometimes "\n\n")
    _with_timeout(s, "block terminator") do
        readuntil(s.sock, UInt8('\n'); keep=false)
    end
    return payload
end

"""
Send `:PRINt?`-style query and return the raw image file.

The scope sends the bare BMP/PNG file with no length header, so the length is taken
from the BMP header or by walking the PNG chunks. An IEEE block is also accepted.
"""
function _query_image(s::SiglentScope, cmd)
    scpi_write(s, cmd)
    head = _read_exact(s, 1)
    if head[1] == UInt8('#')
        n_digits = Int(_read_exact(s, 1)[1] - UInt8('0'))
        len = parse(Int, String(_read_exact(s, n_digits)))
        img = _read_exact(s, len)
    elseif head[1] == UInt8('B')                # BMP: total size at offset 2
        hdr = vcat(head, _read_exact(s, 5))
        len = Int(ltoh(reinterpret(UInt32, hdr[3:6])[1]))
        img = vcat(hdr, _read_exact(s, len - 6))
    elseif head[1] == 0x89                      # PNG: signature then chunks to IEND
        img = vcat(head, _read_exact(s, 7))
        while true
            chunk = _read_exact(s, 8)
            len = Int(ntoh(reinterpret(UInt32, chunk[1:4])[1]))
            append!(img, chunk, _read_exact(s, len + 4))
            String(chunk[5:8]) == "IEND" && break
        end
    else
        error("Unrecognized image header byte 0x$(string(head[1]; base=16))")
    end
    # drop a trailing terminator if one follows shortly, so it is not
    # mistaken for the response to the next query
    # (libuv stops reading between `read` calls, so restart it to see new bytes)
    Base.start_reading(s.sock)
    timedwait(() -> bytesavailable(s.sock) > 0, s.trailer_timeout; pollint=s.pollint)
    if bytesavailable(s.sock) > 0 && peek(s.sock, UInt8) == UInt8('\n')
        read(s.sock, UInt8)
    end
    return img
end

# ---------------------------------------------------------------------- #
# Argument formatting and response parsing                               #
# ---------------------------------------------------------------------- #
_arg(x::Bool) = x ? "ON" : "OFF"
_arg(x::Integer) = string(x)
# shortest round-trip form in the value's own precision, e.g. 2.0E9 (Float32 prints 1.0f-7)
_arg(x::AbstractFloat) = uppercase(replace(string(x), 'f' => 'e'))
_arg(x::Real) = _arg(Float64(x))
_arg(x::AbstractString) = String(x)
_arg(x::Symbol) = String(x)
_arg(x::Union{Tuple,AbstractVector}) = join((_arg(a) for a in x), ',')

_quote(::Nothing) = nothing
_quote(x::Union{Tuple,AbstractVector}) = x
function _quote(x)
    x = _arg(x)
    return startswith(x, '"') ? x : string('"', x, '"')
end

"""`head` followed by the non-`nothing` arguments, comma separated."""
function _cmd(head::AbstractString, args...)
    parts = String[_arg(a) for a in args if a !== nothing]
    return isempty(parts) ? String(head) : string(head, ' ', join(parts, ','))
end

function _parse_int(str)
    v = tryparse(Int, str)
    v === nothing || return v
    f = tryparse(Float64, str)
    return f !== nothing && isinteger(f) ? Int(f) : str
end
_parse_float(str) = something(tryparse(Float64, str), str)
function _parse_bool(str)
    u = uppercase(str)
    u in ("ON", "1") && return true
    u in ("OFF", "0") && return false
    return str
end

_query_str(s::SiglentScope, cmd) = scpi_query(s, cmd)
_query_int(s::SiglentScope, cmd) = _parse_int(scpi_query(s, cmd))
_query_float(s::SiglentScope, cmd) = _parse_float(scpi_query(s, cmd))
_query_bool(s::SiglentScope, cmd) = _parse_bool(scpi_query(s, cmd))

# ---------------------------------------------------------------------- #
# Waveform preamble (:WAVeform:PREamble?)                                #
# ---------------------------------------------------------------------- #
const PREAMBLE_LENGTH = 346         # documented size of the WAVEDESC block

"""
Decoded `:WAVeform:PREamble?` descriptor, following Table 1 of the `:WAVeform:PREamble`
section of the guide. Reserved fields are skipped; the full block is kept in `raw`.

Per the guide, volts = code * (vertical_gain / code_per_div) - vertical_offset, with
`vertical_gain`/`vertical_offset` given without probe attenuation (see `probe`).
"""
struct WaveformPreamble
    descriptor_name::String         # "WAVEDESC"
    template_name::String           # "WAVEACE"
    comm_type::Int                  # data width: 0 = BYTE, 1 = WORD
    comm_order::Int                 # data byte order: 0 = LSB first, 1 = MSB first
    wave_desc_length::Int           # bytes in the descriptor block
    wave_array_1::Int               # bytes in the data array (analog channels)
    instrument_name::String
    wave_array_count::Int           # points in the data array (per frame in sequence mode)
    first_point::Int                # same as :WAVeform:STARt
    data_interval::Int              # same as :WAVeform:INTerval
    read_frames::Int                # sequence frames transferred this time
    sum_frames::Int                 # sequence frames acquired
    vertical_gain::Float64          # V/div, without probe attenuation
    vertical_offset::Float64        # V, without probe attenuation
    code_per_div::Float64
    adc_bit::Int
    frame_index::Int                # <value1> of :WAVeform:SEQuence
    horiz_interval::Float64         # s between samples (1 / sample rate)
    horiz_offset::Float64           # s from the trigger to the first sample
    timebase_index::Int             # enumerated time/div; the table differs by model
    coupling::Symbol                # :DC, :AC or :GND
    probe::Float64                  # probe attenuation
    fixed_vgain_index::Int          # enumerated V/div
    bandwidth_limit::Symbol         # :OFF, Symbol("20M") or Symbol("200M")
    wave_source::String             # "C1" ... "C8"
    raw::Vector{UInt8}
end

# 0-based byte offsets, little endian, as in the guide's table
_le(::Type{T}, buf, off0) where {T} = ltoh(reinterpret(T, buf[off0 + 1 : off0 + sizeof(T)])[1])
function _le_str(buf, off0, len)
    bytes = buf[off0 + 1 : off0 + len]
    nul = findfirst(==(0x00), bytes)
    return String(nul === nothing ? bytes : bytes[1:nul-1])
end
_enum(table, i) = 0 <= i < length(table) ? table[i + 1] : Symbol(i)

"""Decode the payload of `:WAVeform:PREamble?` (as from `scpi_query_block`)."""
function parse_preamble(buf::AbstractVector{UInt8})
    length(buf) >= PREAMBLE_LENGTH ||
        error("Preamble is $(length(buf)) bytes, expected at least $PREAMBLE_LENGTH")
    name = _le_str(buf, 0, 16)
    startswith(name, "WAVEDESC") || error("Preamble does not start with WAVEDESC: $(repr(name))")
    WaveformPreamble(
        name,
        _le_str(buf, 16, 16),
        Int(_le(Int16, buf, 32)),
        Int(_le(Int16, buf, 34)),
        Int(_le(Int32, buf, 36)),
        Int(_le(Int32, buf, 60)),
        _le_str(buf, 76, 16),
        Int(_le(Int32, buf, 116)),
        Int(_le(Int32, buf, 132)),
        Int(_le(Int32, buf, 136)),
        Int(_le(Int32, buf, 144)),
        Int(_le(Int32, buf, 148)),
        Float64(_le(Float32, buf, 156)),
        Float64(_le(Float32, buf, 160)),
        Float64(_le(Float32, buf, 164)),
        Int(_le(Int16, buf, 172)),
        Int(_le(Int16, buf, 174)),
        Float64(_le(Float32, buf, 176)),
        _le(Float64, buf, 180),
        Int(_le(Int16, buf, 324)),
        _enum((:DC, :AC, :GND), Int(_le(Int16, buf, 326))),
        Float64(_le(Float32, buf, 328)),
        Int(_le(Int16, buf, 332)),
        _enum((:OFF, Symbol("20M"), Symbol("200M")), Int(_le(Int16, buf, 334))),
        "C$(Int(_le(Int16, buf, 344)) + 1)",
        Vector{UInt8}(buf),
    )
end

_query_preamble(s::SiglentScope, cmd) = parse_preamble(scpi_query_block(s, cmd))

function Base.show(io::IO, ::MIME"text/plain", p::WaveformPreamble)
    println(io, "WaveformPreamble:")
    for f in fieldnames(WaveformPreamble)
        f === :raw && continue
        println(io, "  ", rpad(string(f), 18), " = ", repr(getfield(p, f)))
    end
    print(io, "  ", rpad("raw", 18), " = ", length(p.raw), " bytes")
end


# ---------------------------------------------------------------------- #
# Common commands                                                        #
# ---------------------------------------------------------------------- #

export get_idn, get_opc, rst!

"""
    get_idn(s)

The command query identifies the instrument type and software version. The response consists of four different fields providing information on the manufacturer, the scope model, the serial number and the firmware revision.

`*IDN?` (guide PDF p. 19)

Returns `String`.

Response format:

    Siglent Technologies,<model>,<serial_number>,<firmware>

    <model>:= The model number of the instrument.

    <serial number>:= A 14-character code.

    <firmware>:= The software revision of the instrument
"""
get_idn(s::SiglentScope) =
    _query_str(s, "*IDN?")

"""
    get_opc(s)

The command query places an ASCII "1" in the output queue when all pending device operations have completed. The interface hangs until this query returns.

`*OPC?` (guide PDF p. 20)

Returns `String`.

Response format:

    1
"""
get_opc(s::SiglentScope) =
    _query_str(s, "*OPC?")

"""
    rst!(s)

Resets the oscilloscope to the default configuration, equivalent to the Default button on the front panel.

`*RST` (guide PDF p. 21)
"""
rst!(s::SiglentScope) =
    scpi_write(s, "*RST")

# ---------------------------------------------------------------------- #
# AUToset commands                                                       #
# ---------------------------------------------------------------------- #

export autoset!

"""
    autoset!(s)

This command attempts to automatically adjust the trigger, vertical, and horizontal controls of the oscilloscope to deliver a usable display of the input signal. Autoset is not recommended for use on low frequency events (< 100 Hz).

`:AUToset` (guide PDF p. 23)
"""
autoset!(s::SiglentScope) =
    scpi_write(s, ":AUToset")

# ---------------------------------------------------------------------- #
# PRINt commands                                                         #
# ---------------------------------------------------------------------- #

export get_print

"""
    get_print(s, type)

The query captures the screen and returns the data in specified image format.

`:PRINt? <type>` (guide PDF p. 23)

Returns `Vector{UInt8}` (image file).

    <type>:= {BMP|PNG}
    - BMP selects bitmap format
    - PNG selects Portable Networks Graphics format

Response format:

    <bin>
    Image data in specified image format
"""
get_print(s::SiglentScope, type) =
    _query_image(s, _cmd(":PRINt?", type))

# ---------------------------------------------------------------------- #
# FORMat commands                                                        #
# ---------------------------------------------------------------------- #

export set_format_data!, get_format_data

"""
    set_format_data!(s, option, digit=nothing)

The command sets the returned precision of the command with data in NR1/NR3 format. The current default precision is 3-digits.

`:FORMat:DATA <option>[,<digit>]` (guide PDF p. 24)

    <option>:= {SINGle|DOUBle|CUSTom}
    - SINGle indicates that the single precision type and
    significant digit is 7.
    - DOUBle indicates that the double precision type and
    significant digit is 14.
    - CUSTom is user-defined precision, and <digit> need to be
    set.

    <digit>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1,64].
"""
set_format_data!(s::SiglentScope, option, digit=nothing) =
    scpi_write(s, _cmd(":FORMat:DATA", option, digit))

"""
    get_format_data(s)

The query returns the current precision of the returned data.

`:FORMat:DATA?` (guide PDF p. 24)

Returns `String`.

Response format:

    CUSTom,<digit>

    <digit>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_format_data(s::SiglentScope) =
    _query_str(s, ":FORMat:DATA?")

# ---------------------------------------------------------------------- #
# ACQuire commands                                                       #
# ---------------------------------------------------------------------- #

export set_acquire_amode!, get_acquire_amode, acquire_csweep!, set_acquire_interpolation!,
      get_acquire_interpolation, set_acquire_mmanagement!, get_acquire_mmanagement,
      set_acquire_mode!, get_acquire_mode, set_acquire_mdepth!, get_acquire_mdepth,
      get_acquire_numacq, get_acquire_points, set_acquire_resolution!,
      get_acquire_resolution, set_acquire_sequence!, get_acquire_sequence,
      set_acquire_sequence_count!, get_acquire_sequence_count, set_acquire_srate!,
      get_acquire_srate, set_acquire_type!, get_acquire_type

"""
    set_acquire_amode!(s, rate)

The command sets the rate of waveform capture. This command can provide a high-speed waveform capture rate to help capture signal anomalies.

`:ACQuire:AMODe <rate>` (guide PDF p. 26)

    <rate>:= {FAST|SLOW}
    FAST selects fast waveform capture
    SLOW selects slow waveform capture
"""
set_acquire_amode!(s::SiglentScope, rate) =
    scpi_write(s, _cmd(":ACQuire:AMODe", rate))

"""
    get_acquire_amode(s)

The query returns the current acquisition rate mode.

`:ACQuire:AMODe?` (guide PDF p. 26)

Returns `String`.

Response format:

    <rate>

    <rate>:= {FAST|SLOW}
"""
get_acquire_amode(s::SiglentScope) =
    _query_str(s, ":ACQuire:AMODe?")

"""
    acquire_csweep!(s)

The command clears the sweep and restarts the acquisition. It is equivalent to the Clear Sweeps button on the front panel.

`:ACQuire:CSWeep` (guide PDF p. 27)
"""
acquire_csweep!(s::SiglentScope) =
    scpi_write(s, ":ACQuire:CSWeep")

"""
    set_acquire_interpolation!(s, state)

The command sets the method of interpolation.

`:ACQuire:INTerpolation <state>` (guide PDF p. 28)

    <state>:= {ON|OFF}
    - ON selects sinx/x (sinc) interpolation
    - OFF selects linear interpolation
"""
set_acquire_interpolation!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":ACQuire:INTerpolation", state))

"""
    get_acquire_interpolation(s)

The query returns the current method of interpolation.

`:ACQuire:INTerpolation?` (guide PDF p. 28)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_acquire_interpolation(s::SiglentScope) =
    _query_bool(s, ":ACQuire:INTerpolation?")

"""
    set_acquire_mmanagement!(s, mem_mode)

The command sets the memory mode of the oscilloscope.

`:ACQuire:MMANagement <mem_mode>` (guide PDF p. 29)

    <mem_mode>:= {AUTO|FSRate|FMDepth}
    - AUTO mode maintain the maximum sampling rate, and
    automatically set the memory depth and sampling rate
    according to the time base.
    - FSRate mode is Fixed Samling Rate, maintain the
    specified sampling rate and automatically set the memory
    depth according to the time base.
    - FMDepth mode is Fixed Memory Depth, the oscilloscope
    automatically sets the sampling rate according to the
    storage depth and time base.
"""
set_acquire_mmanagement!(s::SiglentScope, mem_mode) =
    scpi_write(s, _cmd(":ACQuire:MMANagement", mem_mode))

"""
    get_acquire_mmanagement(s)

The query returns the current memory mode of the oscilloscope.

`:ACQuire:MMANagement?` (guide PDF p. 29)

Returns `String`.

Response format:

    <mem_mode>

    < mem_mode>:= {AUTO|FSRate|FMDepth}
"""
get_acquire_mmanagement(s::SiglentScope) =
    _query_str(s, ":ACQuire:MMANagement?")

"""
    set_acquire_mode!(s, mode_type)

The command sets the acquisition mode of the oscilloscope.

`:ACQuire:MODE <mode_type>` (guide PDF p. 30)

    <mode_type>:= {YT|XY|ROLL}
    - YT mode plots amplitude (Y) vs. time (T)
    - XY mode plots channel X vs. channel Y , commonly
    referred to as a Lissajous curve
    - Roll mode plots amplitude (Y) vs. time (T) as in YT mode,
    but begins to write the waveforms from the right-hand side
    of the display. This is similar to a “strip chart” recording and
    is ideal for slow events that happen a few times/second.
"""
set_acquire_mode!(s::SiglentScope, mode_type) =
    scpi_write(s, _cmd(":ACQuire:MODE", mode_type))

"""
    get_acquire_mode(s)

The query returns the current acquisition mode of the oscilloscope.

`:ACQuire:MODE?` (guide PDF p. 30)

Returns `String`.

Response format:

    <mode_type>

    <mode_type>:= {YT|XY|ROLL}
"""
get_acquire_mode(s::SiglentScope) =
    _query_str(s, ":ACQuire:MODE?")

"""
    set_acquire_mdepth!(s, memory_size)

The command sets the maximum memory depth.

`:ACQuire:MDEPth <memory_size>` (guide PDF p. 31)

    <memory_size>:= Varies by model. See the table below for
    details:
    Model <memory_size>
    SDS5000X
    Single Channel
    {250k|1.25M|2.5M|12.5M|25M|125M|
    250M}
    Dual-Channel
    {125k|625k|1.25M|6.25M|12.5M|
    62.5M|125M}
    SDS2000X Plus
    Single Channel
    {20k|200k|2M|20M|200M}
    Dual-Channel
    {10k|100k|1M|10M|100M}
    SDS6000 Pro
    SDS6000A
    1G Model Single Channel
    {1.25k|5k|25k|50k|250k|500k|
    2.5M|5M|12.5M|125M|250M}
    1G Model Dual-Channel
    {1.25k|2.5k|12.5k|25k|125k|250k|
    1.25M|2.5M|12.5M|62.5M|125M}
    2G Model
    {2.5k|5k|25k|50k|250k|500k|
    2.5M|5M|12.5M|25M|50M|125M|250
    M|250M|500M}
    SDS6000L
    {2.5k|5k|25k|50k|250k|500k|
    2.5M|5M|12.5M|25M|50M|125M|250
    M|250M|500M}
    SHS800X
    SHS1000X
    Single Channel
    {12k|120k|1.2M|12M}
    Dual-Channel
    {6k|60k|600k|6M}
    SDS2000X HD
    Single Channel
    {20k|200k|2M|20M|200M}
    ...
"""
set_acquire_mdepth!(s::SiglentScope, memory_size) =
    scpi_write(s, _cmd(":ACQuire:MDEPth", memory_size))

"""
    get_acquire_mdepth(s)

The query returns the maximum memory depth.

`:ACQuire:MDEPth?` (guide PDF p. 31)

Returns `String`.

Response format:

    <memory_size>
"""
get_acquire_mdepth(s::SiglentScope) =
    _query_str(s, ":ACQuire:MDEPth?")

"""
    get_acquire_numacq(s)

The query returns the number of waveform acquisitions that have occurred since starting acquisition. This value is reset to zero when any acquisition,horizontal, or vertical arguments that affect the waveform are changed.

`:ACQuire:NUMAcq?` (guide PDF p. 33)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_acquire_numacq(s::SiglentScope) =
    _query_int(s, ":ACQuire:NUMAcq?")

"""
    get_acquire_points(s)

The query returns the number of sampled points of the current waveform on the screen.

`:ACQuire:POINts?` (guide PDF p. 34)

Returns `Float64`.

Response format:

    <point>

    <point>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_acquire_points(s::SiglentScope) =
    _query_float(s, ":ACQuire:POINts?")

"""
    set_acquire_resolution!(s, bit)

The command sets the ADC resolution for SDS2000X Plus oscilloscope.

`:ACQuire:RESolution <bit>` (guide PDF p. 35)

    <bit>:= {8Bits|10Bits}
"""
set_acquire_resolution!(s::SiglentScope, bit) =
    scpi_write(s, _cmd(":ACQuire:RESolution", bit))

"""
    get_acquire_resolution(s)

The query returns the ADC resolution for SDS2000X Plus oscilloscope.

`:ACQuire:RESolution?` (guide PDF p. 35)

Returns `String`.

Response format:

    <bit>

    <bit>:= {8Bits|10Bits}
"""
get_acquire_resolution(s::SiglentScope) =
    _query_str(s, ":ACQuire:RESolution?")

"""
    set_acquire_sequence!(s, state)

The command enables or disables sequence acquisition mode.

`:ACQuire:SEQuence <state>` (guide PDF p. 36)

    <state>:= {ON|OFF}
"""
set_acquire_sequence!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":ACQuire:SEQuence", state))

"""
    get_acquire_sequence(s)

The query returns whether the current sequence acquisition switch is on or not.

`:ACQuire:SEQuence?` (guide PDF p. 36)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_acquire_sequence(s::SiglentScope) =
    _query_bool(s, ":ACQuire:SEQuence?")

"""
    set_acquire_sequence_count!(s, count)

The command sets the number of memory segments to acquire. The maximum number of segments may be limited by the memory depth of your oscilloscope.

`:ACQuire:SEQuence:COUNt <count>` (guide PDF p. 37)

    <count>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value varies from the
    models and the current timebase, see the user manual for
    details.
"""
set_acquire_sequence_count!(s::SiglentScope, count) =
    scpi_write(s, _cmd(":ACQuire:SEQuence:COUNt", count))

"""
    get_acquire_sequence_count(s)

The query returns the current count setting.

`:ACQuire:SEQuence:COUNt?` (guide PDF p. 37)

Returns `Int`.

Response format:

    <count_value>

    <count_value>:= Value in NR1 format, including an integer and
    no decimal point, like 1.
"""
get_acquire_sequence_count(s::SiglentScope) =
    _query_int(s, ":ACQuire:SEQuence:COUNt?")

"""
    set_acquire_srate!(s, rate)

The command set the sampling rate when in the fixed sampling rare mode.

`:ACQuire:SRATe <rate>` (guide PDF p. 38)

    <type>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. If the set value is greater than the
    settable value, it will automatically match to the settable value.
"""
set_acquire_srate!(s::SiglentScope, rate) =
    scpi_write(s, _cmd(":ACQuire:SRATe", rate))

"""
    get_acquire_srate(s)

The query returns the current sampling rate.

`:ACQuire:SRATe?` (guide PDF p. 38)

Returns `Float64`.

Response format:

    <sample_rate>

    <sample_rate>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.
"""
get_acquire_srate(s::SiglentScope) =
    _query_float(s, ":ACQuire:SRATe?")

"""
    set_acquire_type!(s, type)

The command selects the type of data acquisition that is to take place.

`:ACQuire:TYPE <type>` (guide PDF p. 39)

    <type>:= {NORMal|PEAK|AVERage[,<times>]|ERES[,<bits>]}

    <times>:= {4|16|32|64|128|256|512|1024}

    <bits>:= {0.5|1.0|1.5|2.0|2.5|3.0}
    - NORMal sets the oscilloscope to normal mode.
    - PEAK sets the oscilloscope to peak detect mode.
    - AVERage sets the oscilloscope acquisition to averaging
    mode. You can set the number of averages by sending the
    command followed by a numeric integer value <times>.
    - ERES sets the oscilloscope to the enhanced resolution
    mode. This is essentially a digital boxcar filter and is used
    to reduce noise at slower sweep speeds. You can set the
    enhanced bits by sending the command followed by the
    <bits>.

    Note:
    The AVERage|ERES type is not available when in sequence
    mode (:ACQuire:SEQuence ON).
"""
set_acquire_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":ACQuire:TYPE", type))

"""
    get_acquire_type(s)

The query returns the current acquisition type.

`:ACQuire:TYPE?` (guide PDF p. 39)

Returns `String`.

Response format:

    <type>

    <type>:= {NORMal|PEAK|AVERage[,<times>]|ERES[,<bits>]}

    <times>:= {4|16|32|64|128|256|512|1024}, when <type> is
    AVERage.

    <bits>:= {0.5|1.0|1.5|2.0|2.5|3.0} when <type> is ERES.
"""
get_acquire_type(s::SiglentScope) =
    _query_str(s, ":ACQuire:TYPE?")

# ---------------------------------------------------------------------- #
# CHANnel commands                                                       #
# ---------------------------------------------------------------------- #

export set_channel_reference!, get_channel_reference, set_channel_bwlimit!,
      get_channel_bwlimit, set_channel_coupling!, get_channel_coupling,
      set_channel_impedance!, get_channel_impedance, set_channel_invert!, get_channel_invert,
      set_channel_label!, get_channel_label, set_channel_label_text!, get_channel_label_text,
      set_channel_offset!, get_channel_offset, set_channel_probe!, get_channel_probe,
      set_channel_scale!, get_channel_scale, set_channel_skew!, get_channel_skew,
      set_channel_switch!, get_channel_switch, set_channel_unit!, get_channel_unit,
      set_channel_visible!, get_channel_visible

"""
    set_channel_reference!(s, type)

This command sets the strategy for the offset value change in the vertical direction when the vertical scale is changed.

`:CHANnel:REFerence <type>` (guide PDF p. 41)

    <type>:= {OFFSet|POSition}
    - OFFset means when the vertical scale is changed, the
    vertical offset remains fixed. As the vertical scale is
    changed, the waveform expands/contracts around the
    main X-axis of the display.
    - POSition means when the vertical scale is changed, the
    vertical offset remains fixed to the grid position on the
    display. As the vertical scale is changed, the waveform
    expands/contracts around the position of the vertical
    ground position on the display.
"""
set_channel_reference!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":CHANnel:REFerence", type))

"""
    get_channel_reference(s)

The query returns the current vertical reference strategy.

`:CHANnel:REFerence?` (guide PDF p. 41)

Returns `String`.

Response format:

    <type>

    <type>:= {OFFSet|POSition}
"""
get_channel_reference(s::SiglentScope) =
    _query_str(s, ":CHANnel:REFerence?")

"""
    set_channel_bwlimit!(s, n, bwlimit)

The command enables or disables the bandwidth-limiting low-pass filter. If the bandwidth filter is on, it will filter the signal to reduce noise and other unwanted high frequency components. When the filter is on, the bandwidth of the specified channel is limited to approximately 20 MHz or 200 MHz.

`:CHANnel<n>:BWLimit <bwlimit>` (guide PDF p. 42)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <bwlimit>:= {FULL|20M|200M}
    - FULL sets the oscilloscope bandwidth to full.
    - 20M enables the 20 MHz bandwidth filter.
    - 200M enables the 200 MHz bandwidth filter.
"""
set_channel_bwlimit!(s::SiglentScope, n, bwlimit) =
    scpi_write(s, _cmd(":CHANnel$(n):BWLimit", bwlimit))

"""
    get_channel_bwlimit(s, n)

The query returns the current setting of the low-pass filter.

`:CHANnel<n>:BWLimit?` (guide PDF p. 42)

Returns `String`.

Response format:

    <bwlimit>

    <bwlimit>:= {FULL|20M|200M}
"""
get_channel_bwlimit(s::SiglentScope, n) =
    _query_str(s, ":CHANnel$(n):BWLimit?")

"""
    set_channel_coupling!(s, n, coupling_mode)

The command selects the coupling mode of the specified input channel.

`:CHANnel<n>:COUPling <coupling_mode>` (guide PDF p. 43)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <coupling_mode>:= {DC|AC|GND}
    - DC sets the channel coupling to DC.
    - AC sets the channel coupling to AC.
    - GND sets the channel coupling to Ground.
"""
set_channel_coupling!(s::SiglentScope, n, coupling_mode) =
    scpi_write(s, _cmd(":CHANnel$(n):COUPling", coupling_mode))

"""
    get_channel_coupling(s, n)

The query returns the coupling mode of the specified channel.

`:CHANnel<n>:COUPling?` (guide PDF p. 43)

Returns `String`.

Response format:

    <coupling_mode>

    <coupling_mode>:= {DC|AC|GND}
"""
get_channel_coupling(s::SiglentScope, n) =
    _query_str(s, ":CHANnel$(n):COUPling?")

"""
    set_channel_impedance!(s, n, impedance)

The command sets the input impedance of the selected channel. There are two impedance values available, depending on model. They are 1 MOhm and 50.

`:CHANnel<n>:IMPedance <impedance>` (guide PDF p. 44)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <impedance>:= {ONEMeg|FIFTy}
    - ONEMeg means 1 Mohm.
    - FIFTy means 50 ohm.

    Note:
    When set to FIFTy, the range of legal values set by
    the :CHAN<n>:SCAL commands is limited to less than 1 V.
"""
set_channel_impedance!(s::SiglentScope, n, impedance) =
    scpi_write(s, _cmd(":CHANnel$(n):IMPedance", impedance))

"""
    get_channel_impedance(s, n)

The query returns the current impedance setting of the selected channel.

`:CHANnel<n>:IMPedance?` (guide PDF p. 44)

Returns `String`.

Response format:

    <impedance>

    <impedance>:= {ONEMeg|FIFTy}
"""
get_channel_impedance(s::SiglentScope, n) =
    _query_str(s, ":CHANnel$(n):IMPedance?")

"""
    set_channel_invert!(s, n, state)

The command selects whether or not to mathematically invert the input signal for the specified channel. This is a mathematical operation and does not change the polarity of the input signal with reference to ground.

`:CHANnel<n>:INVert <state>` (guide PDF p. 45)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
    - ON enables channel inversion.
    - Off disables channel inversion.
"""
set_channel_invert!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":CHANnel$(n):INVert", state))

"""
    get_channel_invert(s, n)

The query returns the current state of the channel inversion.

`:CHANnel<n>:INVert?` (guide PDF p. 45)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_channel_invert(s::SiglentScope, n) =
    _query_bool(s, ":CHANnel$(n):INVert?")

"""
    set_channel_label!(s, n, state)

The command is to turn the specified channel label on or off.

`:CHANnel<n>:LABel <state>` (guide PDF p. 46)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
    - ON enables the channel label.
    - OFF disables the channel label.
"""
set_channel_label!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":CHANnel$(n):LABel", state))

"""
    get_channel_label(s, n)

The query returns the label associated with a particular channel.

`:CHANnel<n>:LABel?` (guide PDF p. 46)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_channel_label(s::SiglentScope, n) =
    _query_bool(s, ":CHANnel$(n):LABel?")

"""
    set_channel_label_text!(s, n, qstring)

The command sets the selected channel label to the string that follows. Setting a label for a channel also adds the name to the label list in non-volatile memory (replacing the oldest label in the list)

`:CHANnel<n>:LABel:TEXT <qstring>` (guide PDF p. 47)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <qstring>:= Quoted string of ASCII text. The length of the string
    is limited to 20.

    Note:
    All characters will be automatically converted to uppercase.
"""
set_channel_label_text!(s::SiglentScope, n, qstring) =
    scpi_write(s, _cmd(":CHANnel$(n):LABel:TEXT", _quote(qstring)))

"""
    get_channel_label_text(s, n)

The query returns the current label text of the selected channel.

`:CHANnel<n>:LABel:TEXT?` (guide PDF p. 47)

Returns `String`.

Response format:

    <string>
"""
get_channel_label_text(s::SiglentScope, n) =
    _query_str(s, ":CHANnel$(n):LABel:TEXT?")

"""
    set_channel_offset!(s, n, offset_value)

The command allows adjustment of the vertical offset of the specified input channel. The maximum ranges depend on the fixed sensitivity setting.

`:CHANnel<n>:OFFSet <offset_value>` (guide PDF p. 48)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <offset_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    Note:
    The range of legal values varies with the value set by
    the :CHANnel<n>:SCALe commands.
"""
set_channel_offset!(s::SiglentScope, n, offset_value) =
    scpi_write(s, _cmd(":CHANnel$(n):OFFSet", offset_value))

"""
    get_channel_offset(s, n)

The query returns the offset value of the specified channel.

`:CHANnel<n>:OFFSet?` (guide PDF p. 48)

Returns `Float64`.

Response format:

    <offset_value>

    <offset_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_channel_offset(s::SiglentScope, n) =
    _query_float(s, ":CHANnel$(n):OFFSet?")

"""
    set_channel_probe!(s, n, attenuation, value=nothing)

The command specifies the probe attenuation factor for the selected channel. This command does not change the actual input sensitivity of the oscilloscope. It changes the reference constants for scaling the display factors, for making automatic measurements, and for setting trigger levels.

`:CHANnel<n>:PROBe <attenuation>[,<value>]` (guide PDF p. 49)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <attenuation>:= {DEFault|VALue}
    - DEFault means set to the default value 1X.
    - VALue means set to the <value>.

    <value>:= Probe attenuation ratio in NR3 format when
    <attenuation> is VALue, and the range is [1E-6, 1E6].
"""
set_channel_probe!(s::SiglentScope, n, attenuation, value=nothing) =
    scpi_write(s, _cmd(":CHANnel$(n):PROBe", attenuation, value))

"""
    get_channel_probe(s, n)

The query returns the current probe attenuation factor for the selected channel.

`:CHANnel<n>:PROBe?` (guide PDF p. 49)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_channel_probe(s::SiglentScope, n) =
    _query_float(s, ":CHANnel$(n):PROBe?")

"""
    set_channel_scale!(s, n, scale)

The command sets the vertical sensitivity in Volts/div. If the probe attenuation is changed, the scale value is multiplied by the probe's attenuation factor.

`:CHANnel<n>:SCALe <scale>` (guide PDF p. 50)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The range of value varies from the models and the bandwidth
    of the model. See the data sheet for details.
"""
set_channel_scale!(s::SiglentScope, n, scale) =
    scpi_write(s, _cmd(":CHANnel$(n):SCALe", scale))

"""
    get_channel_scale(s, n)

The query returns the current vertical sensitivity of the specified channel.

`:CHANnel<n>:SCALe?` (guide PDF p. 50)

Returns `Float64`.

Response format:

    <scale>

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The return value is affected by probe.
"""
get_channel_scale(s::SiglentScope, n) =
    _query_float(s, ":CHANnel$(n):SCALe?")

"""
    set_channel_skew!(s, n, skew_value)

The command sets the channel-to-channel skew factor for the specified channel.

`:CHANnel<n>:SKEW <skew_value>` (guide PDF p. 51)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <skew_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2. The range of the value is
    [-1.00E-07, 1.00E-07].
"""
set_channel_skew!(s::SiglentScope, n, skew_value) =
    scpi_write(s, _cmd(":CHANnel$(n):SKEW", skew_value))

"""
    get_channel_skew(s, n)

The query returns the current probe skew setting for the selected channel.

`:CHANnel<n>:SKEW?` (guide PDF p. 51)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_channel_skew(s::SiglentScope, n) =
    _query_float(s, ":CHANnel$(n):SKEW?")

"""
    set_channel_switch!(s, n, state)

The command turns the display of the specified channel on or off.

`:CHANnel<n>:SWITch <state>` (guide PDF p. 52)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
    <state>:= {OFF|ON}
"""
set_channel_switch!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":CHANnel$(n):SWITch", state))

"""
    get_channel_switch(s, n)

The query returns current status of the selected channel.

`:CHANnel<n>:SWITch?` (guide PDF p. 52)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_channel_switch(s::SiglentScope, n) =
    _query_bool(s, ":CHANnel$(n):SWITch?")

"""
    set_channel_unit!(s, n, unit)

The command change the unit of input signal of specified channel. There is voltage (V) and current (A) two choice to choose for each channel.

`:CHANnel<n>:UNIT <unit>` (guide PDF p. 53)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
    <unit>:= {V|A}

    Note:
    The related parameter units are changed to the selected unit
    after processing this command. This also effects measurement
    results, cursors value, channel sensitivity, and trigger level.
"""
set_channel_unit!(s::SiglentScope, n, unit) =
    scpi_write(s, _cmd(":CHANnel$(n):UNIT", unit))

"""
    get_channel_unit(s, n)

The query returns the current unit of the concerned channel.

`:CHANnel<n>:UNIT?` (guide PDF p. 53)

Returns `String`.

Response format:

    <unit>

    <unit>:= {V|A}
"""
get_channel_unit(s::SiglentScope, n) =
    _query_str(s, ":CHANnel$(n):UNIT?")

"""
    set_channel_visible!(s, n, display_state)

The command is used to whether display the waveform of the specified channel or not. Different from the command :CHANnel<n>:SWITch, it sets the state on the display, and the latter sets the physical switch.

`:CHANnel<n>:VISible <display_state>` (guide PDF p. 54)

    <n>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
    <display_state>:= {ON|OFF}
"""
set_channel_visible!(s::SiglentScope, n, display_state) =
    scpi_write(s, _cmd(":CHANnel$(n):VISible", display_state))

"""
    get_channel_visible(s, n)

The query returns whether the waveform display function of the selected channel is on or off.

`:CHANnel<n>:VISible?` (guide PDF p. 54)

Returns `Bool`.

Response format:

    <display_state>

    <display_state>:= {ON|OFF}
"""
get_channel_visible(s::SiglentScope, n) =
    _query_bool(s, ":CHANnel$(n):VISible?")

# ---------------------------------------------------------------------- #
# CURSor commands                                                        #
# ---------------------------------------------------------------------- #

export set_cursor!, get_cursor, set_cursor_tagstyle!, get_cursor_tagstyle,
      get_cursor_ixdelta, set_cursor_mitem!, get_cursor_mitem, set_cursor_mode!,
      get_cursor_mode, set_cursor_source1!, get_cursor_source1, set_cursor_source2!,
      get_cursor_source2, set_cursor_x1!, get_cursor_x1, set_cursor_x2!, get_cursor_x2,
      get_cursor_xdelta, set_cursor_xreference!, get_cursor_xreference, set_cursor_y1!,
      get_cursor_y1, set_cursor_y2!, get_cursor_y2, get_cursor_ydelta,
      set_cursor_yreference!, get_cursor_yreference

"""
    set_cursor!(s, state)

The command chooses whether to open the cursor.

`:CURSor <state>` (guide PDF p. 56)

    <state>:= {ON|OFF}
"""
set_cursor!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":CURSor", state))

"""
    get_cursor(s)

This query returns the current state of the cursor.

`:CURSor?` (guide PDF p. 56)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_cursor(s::SiglentScope) =
    _query_bool(s, ":CURSor?")

"""
    set_cursor_tagstyle!(s, type)

The command selects the tag type of the cursor value.

`:CURSor:TAGStyle <type>` (guide PDF p. 57)

    <type>:= {FIXed|FOLLowing}
"""
set_cursor_tagstyle!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":CURSor:TAGStyle", type))

"""
    get_cursor_tagstyle(s)

The query returns the current tag type of cursor value.

`:CURSor:TAGStyle?` (guide PDF p. 57)

Returns `String`.

Response format:

    <type>

    <type>:= {FIXed|FOLLowing}
"""
get_cursor_tagstyle(s::SiglentScope) =
    _query_str(s, ":CURSor:TAGStyle?")

"""
    get_cursor_ixdelta(s)

The query returns the current value of cursor 1/(X1-X2).

`:CURSor:IXDelta?` (guide PDF p. 58)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_ixdelta(s::SiglentScope) =
    _query_float(s, ":CURSor:IXDelta?")

"""
    set_cursor_mitem!(s, type, source1, source2=nothing)

The command specifies the measure item of the cursors, when the cursor mode is measure.

`:CURSor:MITem <type>,<source1>[,<source2>]` (guide PDF p. 59)

    <type>:= the type of the selected measurement item in advanced
    measurement,see the table for details.
    <source1>:= the source of the selected measurement item in
    advanced measurement. The optional parameters are the same
    as the measurement source.
    <source2>:= when the type is CH Delay type, source2 needs to
    be specified. The optional parameters are the same as the
    measurement source
"""
set_cursor_mitem!(s::SiglentScope, type, source1, source2=nothing) =
    scpi_write(s, _cmd(":CURSor:MITem", type, source1, source2))

"""
    get_cursor_mitem(s)

The query returns the current measure item of cursor.

`:CURSor:MITem?` (guide PDF p. 59)

Returns `String`.

Response format:

    <type>,<source1>[,<source2>]
"""
get_cursor_mitem(s::SiglentScope) =
    _query_str(s, ":CURSor:MITem?")

"""
    set_cursor_mode!(s, type)

The command specifies the mode of cursor, and the type of cursor to be displayed when the cursor mode is manual.

`:CURSor:MODE <type>` (guide PDF p. 60)

    <type>:= {TRACk|MANual[,<mode>]|MEASure}
    <mode>:= {X|Y|XY}
    - MANul means the manual cursors
    - TRACk means the track cursors
    - MEASure means the measure cursors
"""
set_cursor_mode!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":CURSor:MODE", type))

"""
    get_cursor_mode(s)

The query returns the current mode of cursor.

`:CURSor:MODE?` (guide PDF p. 60)

Returns `String`.

Response format:

    <type>

    <type>:= {TRACk|MANual[,<mode>]}
    <mode>:= {X|Y|XY}
"""
get_cursor_mode(s::SiglentScope) =
    _query_str(s, ":CURSor:MODE?")

"""
    set_cursor_source1!(s, source)

This command specifies the source of the cursor source 1.

`:CURSor:SOURce1 <source>` (guide PDF p. 61)

    <source>:=
    {C<x>|F<x>|REFA|REFB|REFC|REFD|DIGital|HISTOGram}
    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    When the cursor mode is a TRACk, the source cannot be set to
    HISTOGram or DIGital.
"""
set_cursor_source1!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":CURSor:SOURce1", source))

"""
    get_cursor_source1(s)

The query returns the current source of the cursor source 1.

`:CURSor:SOURce1?` (guide PDF p. 61)

Returns `String`.

Response format:

    <source>

    <source>:=
    {C<x>|F<x>|REFA|REFB|REFC|REFD|DIGital|HISTOGram}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_cursor_source1(s::SiglentScope) =
    _query_str(s, ":CURSor:SOURce1?")

"""
    set_cursor_source2!(s, source)

This command specifies the source of the cursor source 2.

`:CURSor:SOURce2 <source>` (guide PDF p. 62)

    <source>:=
    {C<x>|F<x>|REFA|REFB|REFC|REFD|DIGital|HISTOGram}
    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    When the cursor mode is a TRACk, the source cannot be set to
    HISTOGram or DIGital.
"""
set_cursor_source2!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":CURSor:SOURce2", source))

"""
    get_cursor_source2(s)

The query returns the current source of the cursor source 2.

`:CURSor:SOURce2?` (guide PDF p. 62)

Returns `String`.

Response format:

    <source>

    <source>:=
    {C<x>|F<x>|REFA|REFB|REFC|REFD|DIGital|HISTOGram}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_cursor_source2(s::SiglentScope) =
    _query_str(s, ":CURSor:SOURce2?")

"""
    set_cursor_x1!(s, value)

This command specifies the position of the cursor X1.

`:CURSor:X1 <value>` (guide PDF p. 63)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-horizontal_grid/2*timebase, horizontal_grid/2*timebase].
"""
set_cursor_x1!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":CURSor:X1", value))

"""
    get_cursor_x1(s)

The query returns the current position of the cursor X1.

`:CURSor:X1?` (guide PDF p. 63)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_x1(s::SiglentScope) =
    _query_float(s, ":CURSor:X1?")

"""
    set_cursor_x2!(s, value)

This command specifies the position of the cursor X2.

`:CURSor:X2 <value>` (guide PDF p. 64)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-horizontal_grid/2*timebase, horizontal_grid/2*timebase].
"""
set_cursor_x2!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":CURSor:X2", value))

"""
    get_cursor_x2(s)

The query returns the current position of the cursor X2.

`:CURSor:X2?` (guide PDF p. 64)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_x2(s::SiglentScope) =
    _query_float(s, ":CURSor:X2?")

"""
    get_cursor_xdelta(s)

The query returns the horizontal difference between cursor X1 and cursor X2.

`:CURSor:XDELta?` (guide PDF p. 65)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_xdelta(s::SiglentScope) =
    _query_float(s, ":CURSor:XDELta?")

"""
    set_cursor_xreference!(s, type)

This command specifies the expansion strategy around the cursor X.

`:CURSor:XREFerence <type>` (guide PDF p. 66)

    <type>:= {DELay|POSition}
    - DELay means that the cursor value is fixed, and the
    on-screen cursor position changes for different timebase
    values.
    - POSition means that the cursor position is fixed, and does
    not change at any time. Timebase changes cause an
    expansion or contraction of the waveforms around the
    cursor position.
"""
set_cursor_xreference!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":CURSor:XREFerence", type))

"""
    get_cursor_xreference(s)

The query returns the expansion strategy of the cursor X.

`:CURSor:XREFerence?` (guide PDF p. 66)

Returns `String`.

Response format:

    <type>

    < type >:= {DELay|POSition}
"""
get_cursor_xreference(s::SiglentScope) =
    _query_str(s, ":CURSor:XREFerence?")

"""
    set_cursor_y1!(s, value)

This command specifies the position of the cursor Y1.

`:CURSor:Y1 <value>` (guide PDF p. 67)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-vertical_grid/2*vertical_scale, vertical_grid/2*vertical_scale].
"""
set_cursor_y1!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":CURSor:Y1", value))

"""
    get_cursor_y1(s)

The query returns the current position of the cursor Y1.

`:CURSor:Y1?` (guide PDF p. 67)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_y1(s::SiglentScope) =
    _query_float(s, ":CURSor:Y1?")

"""
    set_cursor_y2!(s, value)

This command specifies the position of the cursor Y2.

`:CURSor:Y2 <value>` (guide PDF p. 68)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-vertical_grid/2*vertical_scale, vertical_grid/2*vertical_scale]
"""
set_cursor_y2!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":CURSor:Y2", value))

"""
    get_cursor_y2(s)

The query returns the current position of the cursor Y2.

`:CURSor:Y2?` (guide PDF p. 68)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_y2(s::SiglentScope) =
    _query_float(s, ":CURSor:Y2?")

"""
    get_cursor_ydelta(s)

The query returns the vertical difference between the cursor Y1 and cursor Y2.

`:CURSor:YDELta?` (guide PDF p. 69)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_cursor_ydelta(s::SiglentScope) =
    _query_float(s, ":CURSor:YDELta?")

"""
    set_cursor_yreference!(s, type)

This command specifies the expansion strategy of the Y cursor.

`:CURSor:YREFerence <type>` (guide PDF p. 70)

    <type>:= {OFFSet|POSition}
    - OFFSet means that the cursor value is fixed, and the
    cursor position moves with vertical scale changes. The
    cursors expand or contract if the vertical scale changes.
    - POSition means that the cursor position is fixed, and does
    not change at any time.
"""
set_cursor_yreference!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":CURSor:YREFerence", type))

"""
    get_cursor_yreference(s)

The query returns the expansion strategy of the Y cursor.

`:CURSor:YREFerence?` (guide PDF p. 70)

Returns `String`.

Response format:

    <type>

    <type>:= {OFFSet|POSition}
"""
get_cursor_yreference(s::SiglentScope) =
    _query_str(s, ":CURSor:YREFerence?")

# ---------------------------------------------------------------------- #
# DECode commands                                                        #
# ---------------------------------------------------------------------- #

export set_decode!, get_decode, set_decode_list!, get_decode_list, set_decode_list_line!,
      get_decode_list_line, set_decode_list_scroll!, get_decode_list_scroll, set_decode_bus!,
      get_decode_bus, decode_bus_copy!, set_decode_bus_format!, get_decode_bus_format,
      set_decode_bus_protocol!, get_decode_bus_protocol, get_decode_bus_result,
      set_decode_bus_iic_rwbit!, get_decode_bus_iic_rwbit, set_decode_bus_iic_sclsource!,
      get_decode_bus_iic_sclsource, set_decode_bus_iic_sclthreshold!,
      get_decode_bus_iic_sclthreshold, set_decode_bus_iic_sdasource!,
      get_decode_bus_iic_sdasource, set_decode_bus_iic_sdathreshold!,
      get_decode_bus_iic_sdathreshold, set_decode_bus_spi_bitorder!,
      get_decode_bus_spi_bitorder, set_decode_bus_spi_clksource!,
      get_decode_bus_spi_clksource, set_decode_bus_spi_clkthreshold!,
      get_decode_bus_spi_clkthreshold, set_decode_bus_spi_cssource!,
      get_decode_bus_spi_cssource, set_decode_bus_spi_csthreshold!,
      get_decode_bus_spi_csthreshold, set_decode_bus_spi_cstype!, get_decode_bus_spi_cstype,
      set_decode_bus_spi_dlength!, get_decode_bus_spi_dlength, set_decode_bus_spi_latchedge!,
      get_decode_bus_spi_latchedge, set_decode_bus_spi_misosource!,
      get_decode_bus_spi_misosource, set_decode_bus_spi_misothreshold!,
      get_decode_bus_spi_misothreshold, set_decode_bus_spi_mosisource!,
      get_decode_bus_spi_mosisource, set_decode_bus_spi_mosithreshold!,
      get_decode_bus_spi_mosithreshold, set_decode_bus_spi_ncssource!,
      get_decode_bus_spi_ncssource, set_decode_bus_spi_ncsthreshold!,
      get_decode_bus_spi_ncsthreshold, set_decode_bus_uart_baud!, get_decode_bus_uart_baud,
      set_decode_bus_uart_bitorder!, get_decode_bus_uart_bitorder,
      set_decode_bus_uart_dlength!, get_decode_bus_uart_dlength, set_decode_bus_uart_idle!,
      get_decode_bus_uart_idle, set_decode_bus_uart_parity!, get_decode_bus_uart_parity,
      set_decode_bus_uart_rxsource!, get_decode_bus_uart_rxsource,
      set_decode_bus_uart_rxthreshold!, get_decode_bus_uart_rxthreshold,
      set_decode_bus_uart_stop!, get_decode_bus_uart_stop, set_decode_bus_uart_txsource!,
      get_decode_bus_uart_txsource, set_decode_bus_uart_txthreshold!,
      get_decode_bus_uart_txthreshold, set_decode_bus_can_baud!, get_decode_bus_can_baud,
      set_decode_bus_can_source!, get_decode_bus_can_source, set_decode_bus_can_threshold!,
      get_decode_bus_can_threshold, set_decode_bus_lin_baud!, get_decode_bus_lin_baud,
      set_decode_bus_lin_source!, get_decode_bus_lin_source, set_decode_bus_lin_threshold!,
      get_decode_bus_lin_threshold, set_decode_bus_flexray_baud!,
      get_decode_bus_flexray_baud, set_decode_bus_flexray_source!,
      get_decode_bus_flexray_source, set_decode_bus_flexray_threshold!,
      get_decode_bus_flexray_threshold, set_decode_bus_canfd_bauddata!,
      get_decode_bus_canfd_bauddata, set_decode_bus_canfd_baudnominal!,
      get_decode_bus_canfd_baudnominal, set_decode_bus_canfd_source!,
      get_decode_bus_canfd_source, set_decode_bus_canfd_threshold!,
      get_decode_bus_canfd_threshold, set_decode_bus_iis_annotate!,
      get_decode_bus_iis_annotate, set_decode_bus_iis_avariant!, get_decode_bus_iis_avariant,
      set_decode_bus_iis_bclksource!, get_decode_bus_iis_bclksource,
      set_decode_bus_iis_bclkthreshold!, get_decode_bus_iis_bclkthreshold,
      set_decode_bus_iis_bitorder!, get_decode_bus_iis_bitorder, set_decode_bus_iis_dlength!,
      get_decode_bus_iis_dlength, set_decode_bus_iis_dsource!, get_decode_bus_iis_dsource,
      set_decode_bus_iis_dthreshold!, get_decode_bus_iis_dthreshold,
      set_decode_bus_iis_latchedge!, get_decode_bus_iis_latchedge, set_decode_bus_iis_lch!,
      get_decode_bus_iis_lch, set_decode_bus_iis_sbit!, get_decode_bus_iis_sbit,
      set_decode_bus_iis_wssource!, get_decode_bus_iis_wssource,
      set_decode_bus_iis_wsthreshold!, get_decode_bus_iis_wsthreshold,
      set_decode_bus_m1553_lthreshold!, get_decode_bus_m1553_lthreshold,
      set_decode_bus_m1553_source!, get_decode_bus_m1553_source,
      set_decode_bus_m1553_uthreshold!, get_decode_bus_m1553_uthreshold,
      set_decode_bus_sent_source!, get_decode_bus_sent_source,
      set_decode_bus_sent_threshold!, get_decode_bus_sent_threshold,
      set_decode_bus_sent_format!, get_decode_bus_sent_format, set_decode_bus_sent_clock!,
      get_decode_bus_sent_clock, set_decode_bus_sent_tolerance!,
      get_decode_bus_sent_tolerance, set_decode_bus_sent_idle!, get_decode_bus_sent_idle,
      set_decode_bus_sent_length!, get_decode_bus_sent_length, set_decode_bus_sent_crc!,
      get_decode_bus_sent_crc, set_decode_bus_sent_ppulse!, get_decode_bus_sent_ppulse,
      set_decode_bus_manchester_source!, get_decode_bus_manchester_source,
      set_decode_bus_manchester_threshold!, get_decode_bus_manchester_threshold,
      set_decode_bus_manchester_baud!, get_decode_bus_manchester_baud,
      set_decode_bus_manchester_polarity!, get_decode_bus_manchester_polarity,
      set_decode_bus_manchester_idle!, get_decode_bus_manchester_idle,
      set_decode_bus_manchester_ibits!, get_decode_bus_manchester_ibits,
      set_decode_bus_manchester_start!, get_decode_bus_manchester_start,
      set_decode_bus_manchester_ssize!, get_decode_bus_manchester_ssize,
      set_decode_bus_manchester_hsize!, get_decode_bus_manchester_hsize,
      set_decode_bus_manchester_tsize!, get_decode_bus_manchester_tsize,
      set_decode_bus_manchester_wsize!, get_decode_bus_manchester_wsize,
      set_decode_bus_manchester_dsize!, get_decode_bus_manchester_dsize,
      set_decode_bus_manchester_display!, get_decode_bus_manchester_display,
      set_decode_bus_manchester_bitorder!, get_decode_bus_manchester_bitorder

"""
    set_decode!(s, state)

The command sets the state of the decode function.

`:DECode <state>` (guide PDF p. 72)

    <state>:= {ON|OFF}
"""
set_decode!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DECode", state))

"""
    get_decode(s)

This query returns the current status of the decode function.

`:DECode?` (guide PDF p. 72)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_decode(s::SiglentScope) =
    _query_bool(s, ":DECode?")

"""
    set_decode_list!(s, state)

The command enables or disables the list of decode result.

`:DECode:LIST <state>` (guide PDF p. 73)

    <state>:= {OFF|D1|D2}
    - D1 means bus 1
    - D2 means bus 2
"""
set_decode_list!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DECode:LIST", state))

"""
    get_decode_list(s)

This query returns the current switch state of the decode list.

`:DECode:LIST?` (guide PDF p. 73)

Returns `String`.

Response format:

    <state>

    <state>:= {OFF|D1|D2}
"""
get_decode_list(s::SiglentScope) =
    _query_str(s, ":DECode:LIST?")

"""
    set_decode_list_line!(s, value)

The command sets the number of lines displayed in the decoding list on the screen.

`:DECode:LIST:LINE <value>` (guide PDF p. 74)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of value is [1, 7].
"""
set_decode_list_line!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DECode:LIST:LINE", value))

"""
    get_decode_list_line(s)

This query returns the number of lines displayed in the decoding list.

`:DECode:LIST:LINE?` (guide PDF p. 74)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_list_line(s::SiglentScope) =
    _query_int(s, ":DECode:LIST:LINE?")

"""
    set_decode_list_scroll!(s, value)

The command sets the selected line when the decode list is turned on.

`:DECode:LIST:SCRoll <value>` (guide PDF p. 75)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
set_decode_list_scroll!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DECode:LIST:SCRoll", value))

"""
    get_decode_list_scroll(s)

This query returns the selected line of the decode list.

`:DECode:LIST:SCRoll?` (guide PDF p. 75)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_list_scroll(s::SiglentScope) =
    _query_int(s, ":DECode:LIST:SCRoll?")

"""
    set_decode_bus!(s, n, state)

The command sets the status of the decode bus.

`:DECode:BUS<n> <state>` (guide PDF p. 76)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <state>:= {ON|OFF}.
"""
set_decode_bus!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DECode:BUS$(n)", state))

"""
    get_decode_bus(s, n)

This query returns the current status of the decode bus.

`:DECode:BUS<n>?` (guide PDF p. 76)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_decode_bus(s::SiglentScope, n) =
    _query_bool(s, ":DECode:BUS$(n)?")

"""
    decode_bus_copy!(s, n, operation)

The command synchronizes the decoding settings with the trigger settings.

`:DECode:BUS<n>:COPY <operation>` (guide PDF p. 77)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <operation>:= {FROMtrigger|TOTRigger}.
    - FROMtrigger means copy trigger settings to the decoding
    bus.
    - TOTRigger means copy decoding settings to trigger.
"""
decode_bus_copy!(s::SiglentScope, n, operation) =
    scpi_write(s, _cmd(":DECode:BUS$(n):COPY", operation))

"""
    set_decode_bus_format!(s, n, format)

The command selects the display format of the specified decode bus.

`:DECode:BUS<n>:FORMat <format>` (guide PDF p. 78)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <format>:= {BINary|DECimal|HEX|ASCii}
"""
set_decode_bus_format!(s::SiglentScope, n, format) =
    scpi_write(s, _cmd(":DECode:BUS$(n):FORMat", format))

"""
    get_decode_bus_format(s, n)

This query returns the display format of the specified decode bus.

`:DECode:BUS<n>:FORMat?` (guide PDF p. 78)

Returns `String`.

Response format:

    <format>

    <format>:= {BINary|DECimal|HEX|ASCii}
"""
get_decode_bus_format(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):FORMat?")

"""
    set_decode_bus_protocol!(s, n, protocol)

The command selects the protocol of the specified bus.

`:DECode:BUS<n>:PROTocol <protocol>` (guide PDF p. 79)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <protocol>:=
    {IIC|SPI|UART|CAN|LIN|FLEXray|CANFd|IIS|M1553}
"""
set_decode_bus_protocol!(s::SiglentScope, n, protocol) =
    scpi_write(s, _cmd(":DECode:BUS$(n):PROTocol", protocol))

"""
    get_decode_bus_protocol(s, n)

This query returns the protocol of the specified bus.

`:DECode:BUS<n>:PROTocol?` (guide PDF p. 79)

Returns `String`.

Response format:

    <protocol>

    <protocol>:=
    {IIC|SPI|UART|CAN|LIN|FLEXray|CANFd|IIS|M1553}
"""
get_decode_bus_protocol(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):PROTocol?")

"""
    get_decode_bus_result(s, n)

This query returns the protocol of the specified bus.

`:DECode:BUS<n>:RESult?` (guide PDF p. 80)

Returns `String`.

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

Response format:

    <ascii_text>
    The data is separated by commas, and each frame is separated
    by semicolons and automatically wrapped. The data value is
    related to the format set by “:DECode:BUS<n>:FORMat”. The
    first row of data is header description information.
"""
get_decode_bus_result(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):RESult?")

"""
    set_decode_bus_iic_rwbit!(s, n, state)

This command selects whether the decoding result includes read bit and write bit.

`:DECode:BUS<n>:IIC:RWBit <state>` (guide PDF p. 82)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <state>:= {ON|OFF}.
"""
set_decode_bus_iic_rwbit!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIC:RWBit", state))

"""
    get_decode_bus_iic_rwbit(s, n)

This query returns whether the decoding result includes read and write bits.

`:DECode:BUS<n>:IIC:RWBit?` (guide PDF p. 82)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_decode_bus_iic_rwbit(s::SiglentScope, n) =
    _query_bool(s, ":DECode:BUS$(n):IIC:RWBit?")

"""
    set_decode_bus_iic_sclsource!(s, n, source)

The command selects the SCL source of the IIC bus.

`:DECode:BUS<n>:IIC:SCLSource <source>` (guide PDF p. 83)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_iic_sclsource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIC:SCLSource", source))

"""
    get_decode_bus_iic_sclsource(s, n)

This query returns the current SCL source of the IIC bus.

`:DECode:BUS<n>:IIC:SCLSource?` (guide PDF p. 83)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_iic_sclsource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIC:SCLSource?")

"""
    set_decode_bus_iic_sclthreshold!(s, n, value)

The command sets the threshold of the SCL on IIC bus.

`:DECode:BUS<n>:IIC:SCLThreshold <value>` (guide PDF p. 84)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_iic_sclthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIC:SCLThreshold", value))

"""
    get_decode_bus_iic_sclthreshold(s, n)

This query returns the current threshold of the SCL on IIC bus.

`:DECode:BUS<n>:IIC:SCLThreshold?` (guide PDF p. 84)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_iic_sclthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):IIC:SCLThreshold?")

"""
    set_decode_bus_iic_sdasource!(s, n, source)

The command selects the SDA source of the IIC bus.

`:DECode:BUS<n>:IIC:SDASource <source>` (guide PDF p. 85)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_iic_sdasource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIC:SDASource", source))

"""
    get_decode_bus_iic_sdasource(s, n)

This query returns the current SDA source of the IIC bus.

`:DECode:BUS<n>:IIC:SDASource?` (guide PDF p. 85)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_iic_sdasource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIC:SDASource?")

"""
    set_decode_bus_iic_sdathreshold!(s, n, value)

The command sets the threshold of the SDA on IIC bus.

`:DECode:BUS<n>:IIC:SDAThreshold <value>` (guide PDF p. 86)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_iic_sdathreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIC:SDAThreshold", value))

"""
    get_decode_bus_iic_sdathreshold(s, n)

This query returns the current threshold of the SDA on IIC bus.

`:DECode:BUS<n>:IIC:SDAThreshold?` (guide PDF p. 86)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_iic_sdathreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):IIC:SDAThreshold?")

"""
    set_decode_bus_spi_bitorder!(s, n, order)

The command sets the bit order of the SPI bus.

`:DECode:BUS<n>:SPI:BITorder <order>` (guide PDF p. 88)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <order>:= {LSB|MSB}.
"""
set_decode_bus_spi_bitorder!(s::SiglentScope, n, order) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:BITorder", order))

"""
    get_decode_bus_spi_bitorder(s, n)

This query returns the current bit order of the SPI bus.

`:DECode:BUS<n>:SPI:BITorder?` (guide PDF p. 88)

Returns `String`.

Response format:

    <order>

    <order>:= {LSB|MSB}
"""
get_decode_bus_spi_bitorder(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:BITorder?")

"""
    set_decode_bus_spi_clksource!(s, n, source)

The command selects the CLK source of the SPI bus.

`:DECode:BUS<n>:SPI:CLKSource <source>` (guide PDF p. 89)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_spi_clksource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:CLKSource", source))

"""
    get_decode_bus_spi_clksource(s, n)

This query returns the current CLK source of the SPI bus.

`:DECode:BUS<n>:SPI:CLKSource?` (guide PDF p. 89)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_spi_clksource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:CLKSource?")

"""
    set_decode_bus_spi_clkthreshold!(s, n, value)

The command sets the threshold of the CLK on SPI bus.

`:DECode:BUS<n>:SPI:CLKThreshold <value>` (guide PDF p. 90)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_spi_clkthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:CLKThreshold", value))

"""
    get_decode_bus_spi_clkthreshold(s, n)

This query returns the current threshold of the CLK on SPI bus.

`:DECode:BUS<n>:IIC:CLKThreshold?` (guide PDF p. 90)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_clkthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SPI:CLKThreshold?")

"""
    set_decode_bus_spi_cssource!(s, n, source)

The command sets the CS source of the SPI bus.

`:DECode:BUS<n>:SPI:CSSource <source>` (guide PDF p. 91)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_spi_cssource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:CSSource", source))

"""
    get_decode_bus_spi_cssource(s, n)

This query returns the current CS source of the SPI bus.

`:DECode:BUS<n>:SPI:CSSource?` (guide PDF p. 91)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_spi_cssource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:CSSource?")

"""
    set_decode_bus_spi_csthreshold!(s, n, value)

The command sets the threshold of the CS on SPI bus.

`:DECode:BUS<n>:SPI:CSThreshold <value>` (guide PDF p. 92)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_spi_csthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:CSThreshold", value))

"""
    get_decode_bus_spi_csthreshold(s, n)

This query returns the current threshold of the CS on SPI bus.

`:DECode:BUS<n>:SPI:CSThreshold?` (guide PDF p. 92)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_csthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SPI:CSThreshold?")

"""
    set_decode_bus_spi_cstype!(s, n, type)

The command sets the chip selection type of the SPI bus.

`:DECode:BUS<n>:SPI:CSTYpe <type>` (guide PDF p. 93)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <type>:= {NCS|CS|TIMeout[,<time>]}
    - CS means set to chip select state.
    - NCS means set to non-chip select state.
    - TIMeout indicates set to clock timeout status.

    <time>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [1.00E-07,
    5.00E-03].
"""
set_decode_bus_spi_cstype!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:CSTYpe", type))

"""
    get_decode_bus_spi_cstype(s, n)

This query returns the current chip selection type of the SPI bus.

`:DECode:BUS<n>:SPI:CSTYpe?` (guide PDF p. 93)

Returns `String`.

Response format:

    <type>

    <type>:= {NCS|CS|TIMeout[,<time>]}

    <time>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_cstype(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:CSTYpe?")

"""
    set_decode_bus_spi_dlength!(s, n, value)

The command sets the data length of the SPI bus.

`:DECode:BUS<n>:SPI:DLENgth <value>` (guide PDF p. 94)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [4, 32].
"""
set_decode_bus_spi_dlength!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:DLENgth", value))

"""
    get_decode_bus_spi_dlength(s, n)

This query returns the current data length of the SPI bus.

`:DECode:BUS<n>:SPI:DLENgth?` (guide PDF p. 94)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_spi_dlength(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):SPI:DLENgth?")

"""
    set_decode_bus_spi_latchedge!(s, n, slope)

The command selects the sampling edge of CLK on SPI bus.

`:DECode:BUS<n>:SPI:LATChedge <slope>` (guide PDF p. 95)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <slope>:= {RISing|FALLing}
"""
set_decode_bus_spi_latchedge!(s::SiglentScope, n, slope) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:LATChedge", slope))

"""
    get_decode_bus_spi_latchedge(s, n)

This query returns the sampling edge of CLK on SPI bus.

`:DECode:BUS<n>:SPI:LATChedge?` (guide PDF p. 95)

Returns `String`.

Response format:

    <slope>

    <slope>:= {RISing|FALLing}
"""
get_decode_bus_spi_latchedge(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:LATChedge?")

"""
    set_decode_bus_spi_misosource!(s, n, source)

The command selects the MISO source of the SPI bus.

`:DECode:BUS<n>:SPI:MISOSource <source>` (guide PDF p. 96)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1. For example, C1 selects
    analog channel 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1. For example, D1 selects
    digital channel 1.

    - DIS means no source selected.
"""
set_decode_bus_spi_misosource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:MISOSource", source))

"""
    get_decode_bus_spi_misosource(s, n)

This query returns the current MISO source of the SPI bus.

`:DECode:BUS<n>:SPI:MISOSource?` (guide PDF p. 96)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_spi_misosource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:MISOSource?")

"""
    set_decode_bus_spi_misothreshold!(s, n, value)

The command sets the threshold of the MISO on SPI bus.

`:DECode:BUS<n>:SPI:MISOThreshold <value>` (guide PDF p. 97)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_spi_misothreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:MISOThreshold", value))

"""
    get_decode_bus_spi_misothreshold(s, n)

This query returns the current threshold of the MISO.

`:DECode:BUS<n>:SPI:MISOThreshold?` (guide PDF p. 97)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_misothreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SPI:MISOThreshold?")

"""
    set_decode_bus_spi_mosisource!(s, n, source)

The command selects the MOSI source of the SPI bus.

`:DECode:BUS<n>:SPI:MOSISource <source>` (guide PDF p. 98)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    - DIS means no source selected
"""
set_decode_bus_spi_mosisource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:MOSISource", source))

"""
    get_decode_bus_spi_mosisource(s, n)

This query returns the current MOSI source of the SPI bus.

`:DECode:BUS<n>:SPI:MOSISource?` (guide PDF p. 98)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_spi_mosisource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:MOSISource?")

"""
    set_decode_bus_spi_mosithreshold!(s, n, value)

The command sets the threshold of the MOSI.

`:DECode:BUS<n>:SPI:MOSIThreshold <value>` (guide PDF p. 99)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_spi_mosithreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:MOSIThreshold", value))

"""
    get_decode_bus_spi_mosithreshold(s, n)

This query returns the current threshold of the MOSI.

`:DECode:BUS<n>:SPI:MOSIThreshold?` (guide PDF p. 99)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_mosithreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SPI:MOSIThreshold?")

"""
    set_decode_bus_spi_ncssource!(s, n, source)

The command sets the NCS source of the SPI bus.

`:DECode:BUS<n>:SPI:NCSSource <source>` (guide PDF p. 100)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_spi_ncssource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:NCSSource", source))

"""
    get_decode_bus_spi_ncssource(s, n)

This query returns the current NCS source of the SPI bus.

`:DECode:BUS<n>:SPI:NCSSource?` (guide PDF p. 100)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_spi_ncssource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SPI:NCSSource?")

"""
    set_decode_bus_spi_ncsthreshold!(s, n, value)

The command sets the threshold of the NCS on SPI bus.

`:DECode:BUS<n>:SPI:NCSThreshold <value>` (guide PDF p. 101)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_spi_ncsthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SPI:NCSThreshold", value))

"""
    get_decode_bus_spi_ncsthreshold(s, n)

This query returns the current threshold of the NCS on SPI bus.

`:DECode:BUS<n>:SPI:NCSThreshold?` (guide PDF p. 101)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_spi_ncsthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SPI:NCSThreshold?")

"""
    set_decode_bus_uart_baud!(s, n, baud)

The command sets the baud rate of the UART bus.

`:DECode:BUS<n>:UART:BAUD <baud>` (guide PDF p. 103)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|38400
    bps|57600bps|115200bps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [300, 20000000].
"""
set_decode_bus_uart_baud!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:BAUD", baud))

"""
    get_decode_bus_uart_baud(s, n)

This query returns the current baud rate of the UART bus.

`:DECode:BUS<n>:UART:BAUD?` (guide PDF p. 103)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|38400
    bps|57600bps|115200bps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_uart_baud(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:BAUD?")

"""
    set_decode_bus_uart_bitorder!(s, n, order)

The command sets the bit order of the UART bus.

`:DECode:BUS<n>:UART:BITorder <order>` (guide PDF p. 104)

    <order>:= {LSB|MSB}
"""
set_decode_bus_uart_bitorder!(s::SiglentScope, n, order) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:BITorder", order))

"""
    get_decode_bus_uart_bitorder(s, n)

This query returns the current bit order of the UART bus.

`:DECode:BUS<n>:UART:BITorder?` (guide PDF p. 104)

Returns `String`.

Response format:

    <order>

    <order>:= {LSB|MSB}
    - LSB is Least Significant Bit order
    - MSB is Most Significant Bit order
"""
get_decode_bus_uart_bitorder(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:BITorder?")

"""
    set_decode_bus_uart_dlength!(s, n, value)

The command sets the data length of the UART bus.

`:DECode:BUS<n>:UART:DLENgth <value>` (guide PDF p. 105)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of value is [5, 8].
"""
set_decode_bus_uart_dlength!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:DLENgth", value))

"""
    get_decode_bus_uart_dlength(s, n)

This query returns the current data length of the UART bus.

`:DECode:BUS<n>:UART:DLENgth?` (guide PDF p. 105)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_uart_dlength(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):UART:DLENgth?")

"""
    set_decode_bus_uart_idle!(s, n, idle)

The command sets the idle level of the UART bus.

`:DECode:BUS<n>:UART:IDLE <idle>` (guide PDF p. 106)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <idle>:= {LOW|HIGH}
"""
set_decode_bus_uart_idle!(s::SiglentScope, n, idle) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:IDLE", idle))

"""
    get_decode_bus_uart_idle(s, n)

This query returns the current idle level of the UART bus.

`:DECode:BUS<n>:UART:IDLE?` (guide PDF p. 106)

Returns `String`.

Response format:

    <idle>

    <idle>:= {LOW|HIGH}
    - LOW means that the idle voltage value is low
    - HIGH means that the idle voltage value is high
"""
get_decode_bus_uart_idle(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:IDLE?")

"""
    set_decode_bus_uart_parity!(s, n, parity)

The command sets the parity check of the UART bus.

`:DECode:BUS<n>:UART:PARity <parity>` (guide PDF p. 107)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <parity>:= {NONE|ODD|EVEN|MARK|SPACe}
"""
set_decode_bus_uart_parity!(s::SiglentScope, n, parity) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:PARity", parity))

"""
    get_decode_bus_uart_parity(s, n)

This query returns the current parity check of the UART bus.

`:DECode:BUS<n>:UART:PARity?` (guide PDF p. 107)

Returns `String`.

Response format:

    <parity>

    <parity>:= {NONE|ODD|EVEN|MARK|SPACe}
"""
get_decode_bus_uart_parity(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:PARity?")

"""
    set_decode_bus_uart_rxsource!(s, n, source)

The command sets the RX source of the UART bus.

`:DECode:BUS<n>:UART:RXSource <source>` (guide PDF p. 108)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    - DIS means no source selected
"""
set_decode_bus_uart_rxsource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:RXSource", source))

"""
    get_decode_bus_uart_rxsource(s, n)

This query returns the current RX source of the UART bus.

`:DECode:BUS<n>:UART:RXSource?` (guide PDF p. 108)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_uart_rxsource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:RXSource?")

"""
    set_decode_bus_uart_rxthreshold!(s, n, value)

The command sets the threshold of RX on UART bus.

`:DECode:BUS<n>:UART:RXThreshold <value>` (guide PDF p. 109)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_uart_rxthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:RXThreshold", value))

"""
    get_decode_bus_uart_rxthreshold(s, n)

This query returns the current threshold of RX on UART bus.

`:DECode:BUS<n>:UART:RXThreshold?` (guide PDF p. 109)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_uart_rxthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):UART:RXThreshold?")

"""
    set_decode_bus_uart_stop!(s, n, bit)

The command sets the length of the stop bit on UART bus.

`:DECode:BUS<n>:UART:STOP <bit>` (guide PDF p. 110)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <bit>:= {1|1.5|2}
"""
set_decode_bus_uart_stop!(s::SiglentScope, n, bit) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:STOP", bit))

"""
    get_decode_bus_uart_stop(s, n)

This query returns the current length of the stop bit on UART bus.

`:DECode:BUS<n>:UART:STOP?` (guide PDF p. 110)

Returns `String`.

Response format:

    <bit>

    <bit>:= {1|1.5|2}
"""
get_decode_bus_uart_stop(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:STOP?")

"""
    set_decode_bus_uart_txsource!(s, n, source)

The command sets the TX source of the UART bus.

`:DECode:BUS<n>:UART:TXSource <source>` (guide PDF p. 111)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
    - DIS means no source selected
"""
set_decode_bus_uart_txsource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:TXSource", source))

"""
    get_decode_bus_uart_txsource(s, n)

This query returns the current TX source of the UART bus.

`:DECode:BUS<n>:UART:TXSource?` (guide PDF p. 111)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>|DIS}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_uart_txsource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):UART:TXSource?")

"""
    set_decode_bus_uart_txthreshold!(s, n, value)

The command sets the threshold of TX on UART bus.

`:DECode:BUS<n>:UART:TXThreshold <value>` (guide PDF p. 112)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_uart_txthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):UART:TXThreshold", value))

"""
    get_decode_bus_uart_txthreshold(s, n)

This query returns the current threshold of TX on UART bus.

`:DECode:BUS<n>:UART:TXThreshold?` (guide PDF p. 112)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_uart_txthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):UART:TXThreshold?")

"""
    set_decode_bus_can_baud!(s, n, baud)

The command sets the baud rate of the CAN bus.

`:DECode:BUS<n>:CAN:BAUD <baud>` (guide PDF p. 114)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:=
    {5kbps|10kbps|20kbps|50kbps|100kbps|125kbps|250kbps|500
    kbps|800kbps|1Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [5000, 1000000].
"""
set_decode_bus_can_baud!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CAN:BAUD", baud))

"""
    get_decode_bus_can_baud(s, n)

This query returns the current baud rate of the CAN bus.

`:DECode:BUS<n>:CAN:BAUD?` (guide PDF p. 114)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {5kbps|10kbps|20kbps|50kbps|100kbps|125kbps|250kbps|500
    kbps|800kbps|1Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_can_baud(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):CAN:BAUD?")

"""
    set_decode_bus_can_source!(s, n, source)

The command selects the source of the CAN bus.

`:DECode:BUS<n>:CAN:SOURce <source>` (guide PDF p. 115)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_can_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CAN:SOURce", source))

"""
    get_decode_bus_can_source(s, n)

This query returns the current source of the CAN bus.

`:DECode:BUS<n>:CAN:SOURce?` (guide PDF p. 115)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_can_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):CAN:SOURce?")

"""
    set_decode_bus_can_threshold!(s, n, value)

The command sets the threshold of the source on CAN bus.

`:DECode:BUS<n>:CAN:THReshold <value>` (guide PDF p. 116)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_can_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CAN:THReshold", value))

"""
    get_decode_bus_can_threshold(s, n)

This query returns the current threshold of the source on CAN bus.

`:DECode:BUS<n>:CAN:THReshold?` (guide PDF p. 116)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_can_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):CAN:THReshold?")

"""
    set_decode_bus_lin_baud!(s, n, baud)

The command sets the baud rate for the LIN bus.

`:DECode:BUS<n>:LIN:BAUD <baud>` (guide PDF p. 118)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|CUST
    om[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [300, 20000000].
"""
set_decode_bus_lin_baud!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):LIN:BAUD", baud))

"""
    get_decode_bus_lin_baud(s, n)

This query returns the current baud rate for the LIN bus.

`:DECode:BUS<n>:LIN:BAUD?` (guide PDF p. 118)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|CUST
    om[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_lin_baud(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):LIN:BAUD?")

"""
    set_decode_bus_lin_source!(s, n, source)

The command selects the source of the LIN bus.

`:DECode:BUS<n>:LIN:SOURce <source>` (guide PDF p. 119)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}
    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_lin_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):LIN:SOURce", source))

"""
    get_decode_bus_lin_source(s, n)

This query returns the current source of the LIN bus.

`:DECode:BUS<n>:LIN:SOURce?` (guide PDF p. 119)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_lin_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):LIN:SOURce?")

"""
    set_decode_bus_lin_threshold!(s, n, value)

The command sets the threshold of the source on LIN bus.

`:DECode:BUS<n>:LIN:THReshold <value>` (guide PDF p. 120)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_lin_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):LIN:THReshold", value))

"""
    get_decode_bus_lin_threshold(s, n)

This query returns the current threshold of the source on LIN bus.

`:DECode:BUS<n>:LIN:THReshold?` (guide PDF p. 120)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_lin_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):LIN:THReshold?")

"""
    set_decode_bus_flexray_baud!(s, n, baud)

The command sets the baud rate of the Flexray bus.

`:DECode:BUS<n>:FLEXray:BAUD <baud>` (guide PDF p. 122)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:= {2500kbps|5Mbps|10Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1000000,
    20000000]
"""
set_decode_bus_flexray_baud!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):FLEXray:BAUD", baud))

"""
    get_decode_bus_flexray_baud(s, n)

This query returns the current baud rate of the Flexray bus.

`:DECode:BUS<n>:FLEXray:BAUD?` (guide PDF p. 122)

Returns `String`.

Response format:

    <baud>

    <baud>:= {2500kbps|5Mbps|10Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_flexray_baud(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):FLEXray:BAUD?")

"""
    set_decode_bus_flexray_source!(s, n, source)

The command selects the source of the Flexray bus.

`:DECode:BUS<n>:FLEXray:SOURce <source>` (guide PDF p. 123)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_flexray_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):FLEXray:SOURce", source))

"""
    get_decode_bus_flexray_source(s, n)

This query returns the current source of the Flexray bus.

`:DECode:BUS<n>:FLEXray:SOURce?` (guide PDF p. 123)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_flexray_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):FLEXray:SOURce?")

"""
    set_decode_bus_flexray_threshold!(s, n, value)

The command sets the threshold of the source on Flexray bus.

`:DECode:BUS<n>:FLEXray:THReshold <value>` (guide PDF p. 124)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_flexray_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):FLEXray:THReshold", value))

"""
    get_decode_bus_flexray_threshold(s, n)

This query returns the current threshold of the source on Flexray bus.

`:DECode:BUS<n>:FLEXray:THReshold?` (guide PDF p. 124)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_flexray_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):FLEXray:THReshold?")

"""
    set_decode_bus_canfd_bauddata!(s, n, baud)

The command sets the data baud rate of the CAN FD bus.

`:DECode:BUS<n>:CANFd:BAUDData <baud>` (guide PDF p. 126)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:=
    {500kbps|1Mbps|2Mbps|5Mbps|8Mbps|10Mbps|CUSTom[,<val
    ue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [100000,
    10000000]
"""
set_decode_bus_canfd_bauddata!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CANFd:BAUDData", baud))

"""
    get_decode_bus_canfd_bauddata(s, n)

This query returns the current data baud rate of the CAN FD bus.

`:DECode:BUS<n>:CANFd:BAUDData?` (guide PDF p. 126)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {500kbps|1Mbps|2Mbps|5Mbps|8Mbps|10Mbps|CUSTom[,<val
    ue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_canfd_bauddata(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):CANFd:BAUDData?")

"""
    set_decode_bus_canfd_baudnominal!(s, n, baud)

The command sets the nominal baud rate of the CAN FD bus.

`:DECode:BUS<n>:CANFd:BAUDNominal <baud>` (guide PDF p. 127)

    <n>:= {1|2} is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <baud>:=
    {10kbps|25kbps|50kbps|100kbps|250kbps|1Mbps|CUSTom[,<v
    alue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [10000,
    1000000]
"""
set_decode_bus_canfd_baudnominal!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CANFd:BAUDNominal", baud))

"""
    get_decode_bus_canfd_baudnominal(s, n)

This query returns the current nominal baud rate of the CAN FD bus.

`:DECode:BUS<n>:CANFd:BAUDNominal?` (guide PDF p. 127)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {10kbps|25kbps|50kbps|100kbps|250kbps|1Mbps|CUSTom[,<v
    alue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_canfd_baudnominal(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):CANFd:BAUDNominal?")

"""
    set_decode_bus_canfd_source!(s, n, source)

The command selects the source of the CAN FD bus.

`:DECode:BUS<n>:CANFd:SOURce <source>` (guide PDF p. 128)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_canfd_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CANFd:SOURce", source))

"""
    get_decode_bus_canfd_source(s, n)

This query returns the current source of the CAN FD bus.

`:DECode:BUS<n>:CANFd:SOURce?` (guide PDF p. 128)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_canfd_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):CANFd:SOURce?")

"""
    set_decode_bus_canfd_threshold!(s, n, value)

The command sets the threshold of the source on CAN FD bus.

`:DECode:BUS<n>:CANFd:THReshold <value>` (guide PDF p. 129)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_canfd_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):CANFd:THReshold", value))

"""
    get_decode_bus_canfd_threshold(s, n)

This query returns the current threshold of the source on CAN FD bus.

`:DECode:BUS<n>:CANFd:THReshold?` (guide PDF p. 129)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_canfd_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):CANFd:THReshold?")

"""
    set_decode_bus_iis_annotate!(s, n, type)

The command specifies the channel for IIS bus to be annotated.

`:DECode:BUS<n>:IIS:ANNotate <type>` (guide PDF p. 131)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <type>:= {ALL|LEFT|RIGHt}
"""
set_decode_bus_iis_annotate!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:ANNotate", type))

"""
    get_decode_bus_iis_annotate(s, n)

This query returns the current annotated channel of IIS bus.

`:DECode:BUS<n>:IIS:ANNotate?` (guide PDF p. 131)

Returns `String`.

Response format:

    <type>

    <type>:= {ALL|LEFT|RIGHt}
"""
get_decode_bus_iis_annotate(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:ANNotate?")

"""
    set_decode_bus_iis_avariant!(s, n, type)

The command selects the audio variant for IIS bus.

`:DECode:BUS<n>:IIS:AVARiant <type>` (guide PDF p. 132)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <type>:= {I2S|LJ|RJ}
    - I2S justified.
    - LJ is left justified.
    - RL is right justified.
"""
set_decode_bus_iis_avariant!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:AVARiant", type))

"""
    get_decode_bus_iis_avariant(s, n)

This query returns the current audio variant for IIS bus.

`:DECode:BUS<n>:IIS:AVARiant?` (guide PDF p. 132)

Returns `String`.

Response format:

    <type>

    <type>:= {I2S|LJ|RJ}
"""
get_decode_bus_iis_avariant(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:AVARiant?")

"""
    set_decode_bus_iis_bclksource!(s, n, source)

The command selects the BCLK source of the IIS bus.

`:DECode:BUS<n>:IIS:BCLKSource <source>` (guide PDF p. 133)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_iis_bclksource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:BCLKSource", source))

"""
    get_decode_bus_iis_bclksource(s, n)

This query returns the current BCLK source of the IIS bus.

`:DECode:BUS<n>:IIS:BCLKSource?` (guide PDF p. 133)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_iis_bclksource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:BCLKSource?")

"""
    set_decode_bus_iis_bclkthreshold!(s, n, value)

The command sets the threshold of the BCLK on IIS bus.

`:DECode:BUS<n>:IIS:BCLKThreshold <value>` (guide PDF p. 134)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_iis_bclkthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:BCLKThreshold", value))

"""
    get_decode_bus_iis_bclkthreshold(s, n)

This query returns the current threshold of the BCLK on IIS bus.

`:DECode:BUS<n>:IIS:BCLKThreshold?` (guide PDF p. 134)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_iis_bclkthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):IIS:BCLKThreshold?")

"""
    set_decode_bus_iis_bitorder!(s, n, order)

The command sets the bit order for the IIS bus.

`:DECode:BUS<n>:IIS:BITorder <order>` (guide PDF p. 135)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <order>:= {LSB|MSB}
    - LSB is Least Significant Bit.
    - MSB is Most Significant Bit.
"""
set_decode_bus_iis_bitorder!(s::SiglentScope, n, order) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:BITorder", order))

"""
    get_decode_bus_iis_bitorder(s, n)

This query returns the current bit order for the IIS bus.

`:DECode:BUS<n>:IIS:BITorder?` (guide PDF p. 135)

Returns `String`.

Response format:

    <order>

    <order>:= {LSB|MSB}
"""
get_decode_bus_iis_bitorder(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:BITorder?")

"""
    set_decode_bus_iis_dlength!(s, n, value)

The command sets the data bits for the IIS bus.

`:DECode:BUS<n>:IIS:DLENgth <value>` (guide PDF p. 136)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 32].
"""
set_decode_bus_iis_dlength!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:DLENgth", value))

"""
    get_decode_bus_iis_dlength(s, n)

This query returns the current data bits for the IIS bus.

`:DECode:BUS<n>:IIS:DLENgth?` (guide PDF p. 136)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_iis_dlength(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):IIS:DLENgth?")

"""
    set_decode_bus_iis_dsource!(s, n, source)

The command selects the data source of the IIS bus.

`:DECode:BUS<n>:IIS:DSource <source>` (guide PDF p. 137)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_iis_dsource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:DSource", source))

"""
    get_decode_bus_iis_dsource(s, n)

This query returns the current data source of the IIS bus.

`:DECode:BUS<n>:IIS:DSource?` (guide PDF p. 137)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_iis_dsource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:DSource?")

"""
    set_decode_bus_iis_dthreshold!(s, n, value)

The command sets the threshold of the data source on IIS bus.

`:DECode:BUS<n>:IIS:DTHReshold <value>` (guide PDF p. 138)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_iis_dthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:DTHReshold", value))

"""
    get_decode_bus_iis_dthreshold(s, n)

This query returns the current threshold of the data source on IIS bus.

`:DECode:BUS<n>:IIS:DTHReshold?` (guide PDF p. 138)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_iis_dthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):IIS:DTHReshold?")

"""
    set_decode_bus_iis_latchedge!(s, n, slope)

The command selects the sampling edge of BCLK on IIS bus.

`:DECode:BUS<n>:IIS:LATChedge <slope>` (guide PDF p. 139)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <slope>:= {RISing|FALLing}
"""
set_decode_bus_iis_latchedge!(s::SiglentScope, n, slope) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:LATChedge", slope))

"""
    get_decode_bus_iis_latchedge(s, n)

This query returns the sampling edge of BCLK on IIS bus.

`:DECode:BUS<n>:IIS:LATChedge?` (guide PDF p. 139)

Returns `String`.

Response format:

    <slope>

    <slope>:= {RISing|FALLing}
    - RISing selects the rising edge.
    - FALLing selects the falling edge.
"""
get_decode_bus_iis_latchedge(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:LATChedge?")

"""
    set_decode_bus_iis_lch!(s, n, left)

The command selects the level of the left channel.

`:DECode:BUS<n>:IIS:LCH <left>` (guide PDF p. 140)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <left>:= {LOW|HIGH}
"""
set_decode_bus_iis_lch!(s::SiglentScope, n, left) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:LCH", left))

"""
    get_decode_bus_iis_lch(s, n)

This query returns the current level of the left channel.

`:DECode:BUS<n>:IIS:LCH?` (guide PDF p. 140)

Returns `String`.

Response format:

    <left>

    <left>:= {LOW|HIGH}
"""
get_decode_bus_iis_lch(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:LCH?")

"""
    set_decode_bus_iis_sbit!(s, n, value)

The command sets the start bit of the data.

`:DECode:BUS<n>:IIS:SBIT <value>` (guide PDF p. 141)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 31].
"""
set_decode_bus_iis_sbit!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:SBIT", value))

"""
    get_decode_bus_iis_sbit(s, n)

This query returns the start bit of the data.

`:DECode:BUS<n>:IIS:SBIT?` (guide PDF p. 141)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_iis_sbit(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):IIS:SBIT?")

"""
    set_decode_bus_iis_wssource!(s, n, source)

The command selects the WS source of the IIS bus.

`:DECode:BUS<n>:IIS:WSSource <source>` (guide PDF p. 142)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_iis_wssource!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:WSSource", source))

"""
    get_decode_bus_iis_wssource(s, n)

This query returns the current WS source of the IIS bus.

`:DECode:BUS<n>:IIS:WSSource?` (guide PDF p. 142)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_iis_wssource(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):IIS:WSSource?")

"""
    set_decode_bus_iis_wsthreshold!(s, n, value)

The command sets the threshold of the WS on IIS bus.

`:DECode:BUS<n>:IIS:WSTHreshold <value>` (guide PDF p. 143)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_iis_wsthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):IIS:WSTHreshold", value))

"""
    get_decode_bus_iis_wsthreshold(s, n)

This query returns the current threshold of the WS on IIS bus.

`:DECode:BUS<n>:IIS:WSTHreshold?` (guide PDF p. 143)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_iis_wsthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):IIS:WSTHreshold?")

"""
    set_decode_bus_m1553_lthreshold!(s, n, value)

The command sets the lower threshold of the M1553 source.

`:DECode:BUS<n>:M1553:LTHReshold <value>` (guide PDF p. 145)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]

    Note:
    The lower threshold value cannot be greater than the upper
    threshold value set by the command
    :DECode:BUS<n>:M1553:UTHReshold.
"""
set_decode_bus_m1553_lthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):M1553:LTHReshold", value))

"""
    get_decode_bus_m1553_lthreshold(s, n)

This query returns the current lower threshold of the M1553 source.

`:DECode:BUS<n>:M1553:LTHReshold?` (guide PDF p. 145)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_m1553_lthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):M1553:LTHReshold?")

"""
    set_decode_bus_m1553_source!(s, n, source)

The command selects the source of the M1553 bus.

`:DECode:BUS<n>:M1553:SOURce <source>` (guide PDF p. 146)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_m1553_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):M1553:SOURce", source))

"""
    get_decode_bus_m1553_source(s, n)

This query returns the current source of the M1553 bus.

`:DECode:BUS<n>:M1553:SOURce?` (guide PDF p. 146)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_m1553_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):M1553:SOURce?")

"""
    set_decode_bus_m1553_uthreshold!(s, n, value)

The command sets the upper threshold of the M1553 source.

`:DECode:BUS<n>:M1553:UTHReshold <value>` (guide PDF p. 147)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The upper threshold value cannot be less than the lower
    threshold value set by the command
    :DECode:BUS<n>:M1553:LTHReshold.
"""
set_decode_bus_m1553_uthreshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):M1553:UTHReshold", value))

"""
    get_decode_bus_m1553_uthreshold(s, n)

This query returns the current upper threshold of the M1553 source.

`:DECode:BUS<n>:M1553:UTHReshold?` (guide PDF p. 147)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_m1553_uthreshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):M1553:UTHReshold?")

"""
    set_decode_bus_sent_source!(s, n, source)

The command selects the source of the SENT bus.

`:DECode:BUS<n>:SENT:SOURce <source>` (guide PDF p. 149)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_sent_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:SOURce", source))

"""
    get_decode_bus_sent_source(s, n)

This query returns the current source of the SENT bus.

`:DECode:BUS<n>:SENT:SOURce?` (guide PDF p. 149)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_sent_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SENT:SOURce?")

"""
    set_decode_bus_sent_threshold!(s, n, value)

The command sets the threshold of the source on SENT bus.

`:DECode:BUS<n>:SENT:THReshold <value>` (guide PDF p. 150)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_sent_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:THReshold", value))

"""
    get_decode_bus_sent_threshold(s, n)

This query returns the current threshold of the source on SENT bus.

`:DECode:BUS<n>:SENT:THReshold?` (guide PDF p. 150)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_sent_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SENT:THReshold?")

"""
    set_decode_bus_sent_format!(s, n, format)

The command selects the message format of the SENT bus.

`:DECode:BUS<n>:SENT:FORMat <format>` (guide PDF p. 151)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <format>:= {NIBBles|FSIGnal|SSERial|ESERial}
"""
set_decode_bus_sent_format!(s::SiglentScope, n, format) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:FORMat", format))

"""
    get_decode_bus_sent_format(s, n)

This query returns the message format of the SENT bus.

`:DECode:BUS<n>:SENT:FORMat?` (guide PDF p. 151)

Returns `String`.

Response format:

    <format>

    <format>:= {NIBBles|FSIGnal|SSERial|ESERial}
"""
get_decode_bus_sent_format(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SENT:FORMat?")

"""
    set_decode_bus_sent_clock!(s, n, value)

The command sets the clock period (tick) time of the SENT bus.

`:DECode:BUS<n>:SENT:CLOCk <value>` (guide PDF p. 152)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [500E-09,
    300E-06]
"""
set_decode_bus_sent_clock!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:CLOCk", value))

"""
    get_decode_bus_sent_clock(s, n)

This query returns the current clock period of the SENT bus.

`:DECode:BUS<n>:SENT:CLOCk?` (guide PDF p. 152)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_sent_clock(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):SENT:CLOCk?")

"""
    set_decode_bus_sent_tolerance!(s, n, value)

The command sets the clock percent tolerance of the SENT bus.

`:DECode:BUS<n>:SENT:TOLerance <value>` (guide PDF p. 153)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 25].
"""
set_decode_bus_sent_tolerance!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:TOLerance", value))

"""
    get_decode_bus_sent_tolerance(s, n)

This query returns the current clock tolerance of the SENT bus.

`:DECode:BUS<n>:SENT:TOLerance?` (guide PDF p. 153)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_sent_tolerance(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):SENT:TOLerance?")

"""
    set_decode_bus_sent_idle!(s, n, idle)

The command sets the idle level of the SENT bus.

`:DECode:BUS<n>:SENT:IDLE <idle>` (guide PDF p. 154)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <idle>:= {LOW|HIGH}
"""
set_decode_bus_sent_idle!(s::SiglentScope, n, idle) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:IDLE", idle))

"""
    get_decode_bus_sent_idle(s, n)

The query returns the current idle level of the SENT bus.

`:DECode:BUS<n>:SENT:IDLE?` (guide PDF p. 154)

Returns `String`.

Response format:

    <idle>

    <idle>:= {LOW|HIGH}
"""
get_decode_bus_sent_idle(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):SENT:IDLE?")

"""
    set_decode_bus_sent_length!(s, n, value)

The command sets the number of nibbles of the SENT bus.

`:DECode:BUS<n>:SENT:LENGth <value>` (guide PDF p. 155)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [3, 8].
"""
set_decode_bus_sent_length!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:LENGth", value))

"""
    get_decode_bus_sent_length(s, n)

This query returns the current number of nibbles of the SENT bus.

`:DECode:BUS<n>:SENT:LENGth?` (guide PDF p. 155)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_sent_length(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):SENT:LENGth?")

"""
    set_decode_bus_sent_crc!(s, n, state)

The command sets the CRC format of the SENT bus.

`:DECode:BUS<n>:SENT:CRC <state>` (guide PDF p. 156)

    <state>:= {OFF|ON}
    ON sets to 2010 CRC format.
    OFF sets to 2008 CRC format.
"""
set_decode_bus_sent_crc!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:CRC", state))

"""
    get_decode_bus_sent_crc(s, n)

The query returns the CRC format of the SENT bus.

`:DECode:BUS<n>:SENT:CRC?` (guide PDF p. 156)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_decode_bus_sent_crc(s::SiglentScope, n) =
    _query_bool(s, ":DECode:BUS$(n):SENT:CRC?")

"""
    set_decode_bus_sent_ppulse!(s, n, state)

The command sets the state of pause pulse of the SENT bus.

`:DECode:BUS<n>:SENT:PPULse <state>` (guide PDF p. 157)

    <state>:= {OFF|ON}
"""
set_decode_bus_sent_ppulse!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DECode:BUS$(n):SENT:PPULse", state))

"""
    get_decode_bus_sent_ppulse(s, n)

The query returns the current state of pause pulse of the SENT bus.

`:DECode:BUS<n>:SENT:PPULse?` (guide PDF p. 157)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_decode_bus_sent_ppulse(s::SiglentScope, n) =
    _query_bool(s, ":DECode:BUS$(n):SENT:PPULse?")

"""
    set_decode_bus_manchester_source!(s, n, source)

The command selects the source of the Manchester bus.

`:DECode:BUS<n>:MANChester:SOURce <source>` (guide PDF p. 159)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_decode_bus_manchester_source!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:SOURce", source))

"""
    get_decode_bus_manchester_source(s, n)

This query returns the current source of the Manchester bus.

`:DECode:BUS<n>:MANChester:SOURce?` (guide PDF p. 159)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<m>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_decode_bus_manchester_source(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:SOURce?")

"""
    set_decode_bus_manchester_threshold!(s, n, value)

The command sets the threshold of the source on Manchester bus.

`:DECode:BUS<n>:MANChester:THReshold <value>` (guide PDF p. 160)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_decode_bus_manchester_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:THReshold", value))

"""
    get_decode_bus_manchester_threshold(s, n)

This query returns the current threshold of the source on Manchester bus.

`:DECode:BUS<n>:MANChester:THReshold?` (guide PDF p. 160)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_decode_bus_manchester_threshold(s::SiglentScope, n) =
    _query_float(s, ":DECode:BUS$(n):MANChester:THReshold?")

"""
    set_decode_bus_manchester_baud!(s, n, baud)

The command sets the baud rate for the Manchester bus.

`:DECode:BUS<n>:MANChester:BAUD <baud>` (guide PDF p. 161)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [500, 5000000].
"""
set_decode_bus_manchester_baud!(s::SiglentScope, n, baud) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:BAUD", baud))

"""
    get_decode_bus_manchester_baud(s, n)

This query returns the current baud rate for the Manchester bus.

`:DECode:BUS<n>:MANChester:BAUD?` (guide PDF p. 161)

Returns `String`.

Response format:

    <baud>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_baud(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:BAUD?")

"""
    set_decode_bus_manchester_polarity!(s, n, polar)

The command sets the signal's logic type of the Manchester bus.

`:DECode:BUS<n>:MANChester:PULarity <polar>` (guide PDF p. 162)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <polar>:= {RISing|FALLing}
    - RISing indicates that rising edge is used to encode a bit
    value of logic 1.
    - FALLing indicates that falling edge is used to encode a bit
    value of logic 1.
"""
set_decode_bus_manchester_polarity!(s::SiglentScope, n, polar) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:POLarity", polar))

"""
    get_decode_bus_manchester_polarity(s, n)

The query returns the current polarity of the Manchester bus.

`:DECode:BUS<n>:MANChester:POLarity?` (guide PDF p. 162)

Returns `String`.

Response format:

    <polar>

    <polar>:= {RISing|FALLing}
"""
get_decode_bus_manchester_polarity(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:POLarity?")

"""
    set_decode_bus_manchester_idle!(s, n, idle)

The command sets the idle level of the Manchester bus.

`:DECode:BUS<n>:MANChester:IDLE <idle>` (guide PDF p. 163)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <idle>:= {LOW|HIGH}
"""
set_decode_bus_manchester_idle!(s::SiglentScope, n, idle) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:IDLE", idle))

"""
    get_decode_bus_manchester_idle(s, n)

The query returns the current idle level of the Manchester bus.

`:DECode:BUS<n>:MANChester:IDLE?` (guide PDF p. 163)

Returns `String`.

Response format:

    <idle>

    <idle>:= {LOW|HIGH}
"""
get_decode_bus_manchester_idle(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:IDLE?")

"""
    set_decode_bus_manchester_ibits!(s, n, value)

The command sets the idle bits of the Manchester bus.

`:DECode:BUS<n>:MANChester:IBITs <value>` (guide PDF p. 164)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [2, 32].
"""
set_decode_bus_manchester_ibits!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:IBITs", value))

"""
    get_decode_bus_manchester_ibits(s, n)

This query returns the current idle bits of the Manchester bus.

`:DECode:BUS<n>:MANChester:IBITs?` (guide PDF p. 164)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_ibits(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:IBITs?")

"""
    set_decode_bus_manchester_start!(s, n, value)

The command sets the start edge of the Manchester bus.

`:DECode:BUS<n>:MANChester:STARt <value>` (guide PDF p. 165)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 32].
"""
set_decode_bus_manchester_start!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:STARt", value))

"""
    get_decode_bus_manchester_start(s, n)

This query returns the current start edge of the Manchester bus.

`:DECode:BUS<n>:MANChester:STARt?` (guide PDF p. 165)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_start(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:STARt?")

"""
    set_decode_bus_manchester_ssize!(s, n, value)

The command sets the sync size of the Manchester bus.

`:DECode:BUS<n>:MANChester:SSIZe <value>` (guide PDF p. 166)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 32].
"""
set_decode_bus_manchester_ssize!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:SSIZe", value))

"""
    get_decode_bus_manchester_ssize(s, n)

This query returns the current sync size of the Manchester bus.

`:DECode:BUS<n>:MANChester:SSIZe?` (guide PDF p. 166)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_ssize(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:SSIZe?")

"""
    set_decode_bus_manchester_hsize!(s, n, value)

The command sets the header size of the Manchester bus.

`:DECode:BUS<n>:MANChester:HSIZe <value>` (guide PDF p. 167)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 32].
"""
set_decode_bus_manchester_hsize!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:HSIZe", value))

"""
    get_decode_bus_manchester_hsize(s, n)

This query returns the current header size of the Manchester bus.

`:DECode:BUS<n>:MANChester:HSIZe?` (guide PDF p. 167)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_hsize(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:HSIZe?")

"""
    set_decode_bus_manchester_tsize!(s, n, value)

The command sets the trailer size of the Manchester bus.

`:DECode:BUS<n>:MANChester:TSIZe <value>` (guide PDF p. 168)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 32].
"""
set_decode_bus_manchester_tsize!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:TSIZe", value))

"""
    get_decode_bus_manchester_tsize(s, n)

This query returns the current trailer size of the Manchester bus.

`:DECode:BUS<n>:MANChester:TSIZe?` (guide PDF p. 168)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_tsize(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:TSIZe?")

"""
    set_decode_bus_manchester_wsize!(s, n, value)

The command sets the word size of the Manchester bus.

`:DECode:BUS<n>:MANChester:WSIZe <value>` (guide PDF p. 169)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [2, 8].
"""
set_decode_bus_manchester_wsize!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:WSIZe", value))

"""
    get_decode_bus_manchester_wsize(s, n)

This query returns the current word size of the Manchester bus.

`:DECode:BUS<n>:MANChester:WSIZe?` (guide PDF p. 169)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_wsize(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:WSIZe?")

"""
    set_decode_bus_manchester_dsize!(s, n, value)

The command sets the data word length of the Manchester bus.

`:DECode:BUS<n>:MANChester:DSIZe <value>` (guide PDF p. 170)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 255].
"""
set_decode_bus_manchester_dsize!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:DSIZe", value))

"""
    get_decode_bus_manchester_dsize(s, n)

This query returns the current data word length of the Manchester bus.

`:DECode:BUS<n>:MANChester:DSIZe?` (guide PDF p. 170)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_decode_bus_manchester_dsize(s::SiglentScope, n) =
    _query_int(s, ":DECode:BUS$(n):MANChester:DSIZe?")

"""
    set_decode_bus_manchester_display!(s, n, format)

The command sets the display format of the Manchester bus.

`:DECode:BUS<n>:MANChester:DISPlay <format>` (guide PDF p. 171)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <format>:= {WORD|BIT}
"""
set_decode_bus_manchester_display!(s::SiglentScope, n, format) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:DISPlay", format))

"""
    get_decode_bus_manchester_display(s, n)

The query returns the current display format of the Manchester bus.

`:DECode:BUS<n>:MANChester:DISPlay?` (guide PDF p. 171)

Returns `String`.

Response format:

    <format>

    <format>:= {WORD|BIT}
"""
get_decode_bus_manchester_display(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:DISPlay?")

"""
    set_decode_bus_manchester_bitorder!(s, n, order)

The command sets the bit order of the Manchester bus.

`:DECode:BUS<n>:MANChester:BITorder <order>` (guide PDF p. 172)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command

    <order>:= {LSB|MSB}
"""
set_decode_bus_manchester_bitorder!(s::SiglentScope, n, order) =
    scpi_write(s, _cmd(":DECode:BUS$(n):MANChester:BITorder", order))

"""
    get_decode_bus_manchester_bitorder(s, n)

The query returns the current bit order of the Manchester bus.

`:DECode:BUS<n>:MANChester:BITorder?` (guide PDF p. 172)

Returns `String`.

Response format:

    <order>

    <order>:= {LSB|MSB}
"""
get_decode_bus_manchester_bitorder(s::SiglentScope, n) =
    _query_str(s, ":DECode:BUS$(n):MANChester:BITorder?")

# ---------------------------------------------------------------------- #
# DIGital commands                                                       #
# ---------------------------------------------------------------------- #

export set_digital!, get_digital, set_digital_active!, get_digital_active,
      digital_bus_default!, set_digital_bus_display!, get_digital_bus_display,
      set_digital_bus_format!, get_digital_bus_format, set_digital_bus_map!,
      get_digital_bus_map, set_digital_d!, get_digital_d, set_digital_height!,
      get_digital_height, set_digital_label!, get_digital_label, get_digital_points,
      set_digital_position!, get_digital_position, set_digital_skew!, get_digital_skew,
      get_digital_srate, set_digital_threshold!, get_digital_threshold

"""
    set_digital!(s, state)

The command set the switch of the digital.

`:DIGital <state>` (guide PDF p. 174)

    <state>:= {ON|OFF}
    - ON enables the channel.
    - OFF disables the channel.
"""
set_digital!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DIGital", state))

"""
    get_digital(s)

This query returns the current state of the digital.

`:DIGital?` (guide PDF p. 174)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_digital(s::SiglentScope) =
    _query_bool(s, ":DIGital?")

"""
    set_digital_active!(s, digital)

This command activates the specified digital channel.

`:DIGital:ACTive <digital>` (guide PDF p. 175)

    <digital>:= {D<x>}

    <x>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_digital_active!(s::SiglentScope, digital) =
    scpi_write(s, _cmd(":DIGital:ACTive", digital))

"""
    get_digital_active(s)

This query returns the active digital channel.

`:DIGital:ACTive?` (guide PDF p. 175)

Returns `String`.

Response format:

    <digital>

    <digital>:= {D<x>}

    <x>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_digital_active(s::SiglentScope) =
    _query_str(s, ":DIGital:ACTive?")

"""
    digital_bus_default!(s, n)

This command resets the digital channel bus bit order

`:DIGital:BUS<n>:DEFault` (guide PDF p. 176)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.
"""
digital_bus_default!(s::SiglentScope, n) =
    scpi_write(s, ":DIGital:BUS$(n):DEFault")

"""
    set_digital_bus_display!(s, n, state)

The command sets the display of the specified digital bus.

`:DIGital:BUS<n>:DISPlay <state>` (guide PDF p. 177)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <state>:= {ON|OFF}
"""
set_digital_bus_display!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DIGital:BUS$(n):DISPlay", state))

"""
    get_digital_bus_display(s, n)

This query returns the current display of the specified digital bus.

`:DIGital:BUS<n>:DISPlay?` (guide PDF p. 177)

Returns `String`.

Response format:

    <state>

    <state>:= {ON|OFF}
    - ON displays the selected bus.
    - OFF removes the selected bus from the display.
"""
get_digital_bus_display(s::SiglentScope, n) =
    _query_str(s, ":DIGital:BUS$(n):DISPlay?")

"""
    set_digital_bus_format!(s, n, format)

The command selects the display format of the specified digital bus.

`:DIGital:BUS<n>:FORMat <format>` (guide PDF p. 178)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <format>:= {BINary|DECimal|HEX|ASCii}
    - BINary presents the decoded data in binary format
    - DECimal presents the decoded data in decimal format
    - HEX presents the decoded data in hexadecimal format
    - ASCii presents the decoded data in ASCII format
"""
set_digital_bus_format!(s::SiglentScope, n, format) =
    scpi_write(s, _cmd(":DIGital:BUS$(n):FORMat", format))

"""
    get_digital_bus_format(s, n)

This query returns the current display format of the specified digital bus.

`:DIGital:BUS<n>:FORMat?` (guide PDF p. 178)

Returns `String`.

Response format:

    <format>

    <format>:= {BINary|DECimal|HEX|ASCii}
"""
get_digital_bus_format(s::SiglentScope, n) =
    _query_str(s, ":DIGital:BUS$(n):FORMat?")

"""
    set_digital_bus_map!(s, n, source...)

The command sets the bit order of each digital channel in the digital bus and the bit width of the digital bus.

`:DIGital:BUS<n>:MAP <source>[...[,<source>]]` (guide PDF p. 179)

    <n>:= {1|2}, is attached as a suffix to BUS and defines the bus
    that is affected by the command.

    <source>:= {D<x>}

    <x>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    • It will synchronously set the bit width of the digital bus,
    which is determined by the number of parameters.
    • Use the command :DIGital:BUS<n>:DEFault to reset the
    bit sequence to d0-d15 according to the current digital bus
    bit width.
"""
set_digital_bus_map!(s::SiglentScope, n, source...) =
    scpi_write(s, _cmd(":DIGital:BUS$(n):MAP", source...))

"""
    get_digital_bus_map(s, n)

The query returns the current digital bus data composition in the LSB order.

`:DIGital:BUS<n>:MAP?` (guide PDF p. 179)

Returns `String`.

Response format:

    <source>[...[,<source>]]

    <source>:= {D<x>}

    <x>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_digital_bus_map(s::SiglentScope, n) =
    _query_str(s, ":DIGital:BUS$(n):MAP?")

"""
    set_digital_d!(s, n, state)

This command enables or disables the specified digital channel.

`:DIGital:D<n> <state>` (guide PDF p. 180)

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
    - ON enables the specified digital channel.
    - OFF disables the specified digital channel.
"""
set_digital_d!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":DIGital:D$(n)", state))

"""
    get_digital_d(s, n)

This query returns the switch of the specified digital channel.

`:DIGital:D<n>?` (guide PDF p. 180)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_digital_d(s::SiglentScope, n) =
    _query_bool(s, ":DIGital:D$(n)?")

"""
    set_digital_height!(s, value)

This command sets the height of digital channel waveform display.

`:DIGital:HEIGht <value>` (guide PDF p. 181)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. This value indicates the number of
    divisions occupied by the digital waveform in the vertical
    direction when the waveform area is not compressed.

    The range of the value is [4.00E+00, 8.00E+00].
"""
set_digital_height!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DIGital:HEIGht", value))

"""
    get_digital_height(s)

This query returns the height of digital channel waveform display.

`:DIGital:HEIGht?` (guide PDF p. 181)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_height(s::SiglentScope) =
    _query_float(s, ":DIGital:HEIGht?")

"""
    set_digital_label!(s, n, text)

This command sets the label text of the selected digital channel.

`:DIGital:LABel<n> <string>` (guide PDF p. 182)

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    <string>:= Quoted string of ASCII text. The length of the string
    is limited to 7.
"""
set_digital_label!(s::SiglentScope, n, text) =
    scpi_write(s, _cmd(":DIGital:LABel$(n)", _quote(text)))

"""
    get_digital_label(s, n)

This query returns the current label text of the selected digital channel.

`:DIGital:LABel?` (guide PDF p. 182)

Returns `String`.

Response format:

    <string>
"""
get_digital_label(s::SiglentScope, n) =
    _query_str(s, ":DIGital:LABel$(n)?")

"""
    get_digital_points(s)

This query returns the number of sampling points of the digital channel.

`:DIGital:POINts?` (guide PDF p. 183)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_points(s::SiglentScope) =
    _query_float(s, ":DIGital:POINts?")

"""
    set_digital_position!(s, value)

The command sets the position of the digital channel waveform display.

`:DIGital:POSition <value>` (guide PDF p. 184)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. This value indicates the number of
    divisions the digital waveform moves from top to bottom of the
    waveform area when the waveform area is not compressed

    Note:
    The range of legal values varies with the number of digital
    channels displayed.
"""
set_digital_position!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DIGital:POSition", value))

"""
    get_digital_position(s)

The query returns the position of the digital channel waveform display.

`:DIGital:POSition?` (guide PDF p. 184)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_position(s::SiglentScope) =
    _query_float(s, ":DIGital:POSition?")

"""
    set_digital_skew!(s, value)

This command sets the skew of the digital channel.

`:DIGital:SKEW <value>` (guide PDF p. 185)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value is [-1.00E-07, 1.00E-07].
"""
set_digital_skew!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DIGital:SKEW", value))

"""
    get_digital_skew(s)

This query returns the current skew of the digital channel.

`:DIGital:SKEW?` (guide PDF p. 185)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_skew(s::SiglentScope) =
    _query_float(s, ":DIGital:SKEW?")

"""
    get_digital_srate(s)

This command query returns the sampling rate of the digital channel.

`:DIGital:SRATe?` (guide PDF p. 186)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_srate(s::SiglentScope) =
    _query_float(s, ":DIGital:SRATe?")

"""
    set_digital_threshold!(s, n, type)

This command sets the threshold value of the digital channel group.

`:DIGital:THReshold<n> <type>` (guide PDF p. 187)

    <n>:= {1|2}
    - 1 means D0-D7
    - 2 means D8-D15

    <type>:=
    {TTL|CMOS|LVCMOS33|LVCMOS25|CUSTom[,<value>]}

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value is [-1.00E+01, 1.00E+01]
"""
set_digital_threshold!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":DIGital:THReshold$(n)", type))

"""
    get_digital_threshold(s, n)

This query returns the threshold value of the digital channel group.

`:DIGital:THReshold<n>?` (guide PDF p. 187)

Returns `String`.

Response format:

    <type>

    <type>:=
    {TTL|CMOS|LVCMOS33|LVCMOS25|CUSTom[,<value>]}

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_digital_threshold(s::SiglentScope, n) =
    _query_str(s, ":DIGital:THReshold$(n)?")

# ---------------------------------------------------------------------- #
# DISPlay commands                                                       #
# ---------------------------------------------------------------------- #

export set_display_axis!, get_display_axis, set_display_axis_mode!, get_display_axis_mode,
      set_display_backlight!, get_display_backlight, display_clear!, set_display_color!,
      get_display_color, set_display_graticule!, get_display_graticule,
      set_display_gridstyle!, get_display_gridstyle, set_display_intensity!,
      get_display_intensity, set_display_menu!, get_display_menu, set_display_persistence!,
      get_display_persistence, set_display_transparence!, get_display_transparence,
      set_display_type!, get_display_type

"""
    set_display_axis!(s, state)

The command sets the display of the axis label.

`:DISPlay:AXIS <state>` (guide PDF p. 189)

    <state>:= {ON|OFF}
"""
set_display_axis!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DISPlay:AXIS", state))

"""
    get_display_axis(s)

The query returns the current status of the axis label.

`:DISPlay:AXIS?` (guide PDF p. 189)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_display_axis(s::SiglentScope) =
    _query_bool(s, ":DISPlay:AXIS?")

"""
    set_display_axis_mode!(s, mode)

The command selects the mode of the axis label.

`:DISPlay:AXIS:MODE <mode>` (guide PDF p. 190)

    <mode>:= {FIXed|MOVing}
    - FIXed means that position of the axes remain fixed, while
    the coordinates update as the waveform is moving.
    - MOVing means when moving the waveform, the position of
    the axes moves with the waveform, while the coordinates
    remain fixed.
"""
set_display_axis_mode!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":DISPlay:AXIS:MODE", mode))

"""
    get_display_axis_mode(s)

The query returns the current mdoe of the axis label.

`:DISPlay:AXIS:MODE?` (guide PDF p. 190)

Returns `String`.

Response format:

    <mode>

    <mode>:= {FIXed|MOVing}
"""
get_display_axis_mode(s::SiglentScope) =
    _query_str(s, ":DISPlay:AXIS:MODE?")

"""
    set_display_backlight!(s, value)

This command sets the backlight level of the screen.

`:DISPlay:BACKlight <value>` (guide PDF p. 191)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 100]. 0 is the
    least bright and 100 is the brightest.
"""
set_display_backlight!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DISPlay:BACKlight", value))

"""
    get_display_backlight(s)

The query returns the current backlight level of the screen.

`:DISPlay:BACKlight?` (guide PDF p. 191)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_display_backlight(s::SiglentScope) =
    _query_int(s, ":DISPlay:BACKlight?")

"""
    display_clear!(s)

The command clears the waveform displayed on the screen.

`:DISPlay:CLEar` (guide PDF p. 192)
"""
display_clear!(s::SiglentScope) =
    scpi_write(s, ":DISPlay:CLEar")

"""
    set_display_color!(s, state)

The command sets the state of the color grade.

`:DISPlay:COLor <state>` (guide PDF p. 193)

    <state>:= {ON|OFF}
"""
set_display_color!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DISPlay:COLor", state))

"""
    get_display_color(s)

The query returns the state of the current color grade.

`:DISPlay:COLor?` (guide PDF p. 193)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_display_color(s::SiglentScope) =
    _query_bool(s, ":DISPlay:COLor?")

"""
    set_display_graticule!(s, value)

The command sets the brightness level of the grid.

`:DISPlay:GRATicule <value>` (guide PDF p. 194)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 100]. 0 is the
    least bright and 100 is the brightest.
"""
set_display_graticule!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DISPlay:GRATicule", value))

"""
    get_display_graticule(s)

The query returns the current brightness level of the grid.

`:DISPlay:GRATicule?` (guide PDF p. 194)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_display_graticule(s::SiglentScope) =
    _query_int(s, ":DISPlay:GRATicule?")

"""
    set_display_gridstyle!(s, type)

This command selects the type of grid to display.

`:DISPlay:GRIDstyle <type>` (guide PDF p. 195)

    <type>:= {FULL|LIGHt|NONE}
"""
set_display_gridstyle!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":DISPlay:GRIDstyle", type))

"""
    get_display_gridstyle(s)

The query returns the current type of grid to display.

`:DISPlay:GRIDstyle?` (guide PDF p. 195)

Returns `String`.

Response format:

    <type>

    <type>:= {FULL|LIGHt|NONE}
"""
get_display_gridstyle(s::SiglentScope) =
    _query_str(s, ":DISPlay:GRIDstyle?")

"""
    set_display_intensity!(s, value)

The command sets the intensity level of the waveform.

`:DISPlay:INTensity <value>` (guide PDF p. 196)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 100]. 0 is the
    least bright and 100 is the brightest.
"""
set_display_intensity!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DISPlay:INTensity", value))

"""
    get_display_intensity(s)

The query returns the current intensity level of the waveform.

`:DISPlay:INTensity?` (guide PDF p. 196)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_display_intensity(s::SiglentScope) =
    _query_int(s, ":DISPlay:INTensity?")

"""
    set_display_menu!(s, type)

This command selects the style of menu to display.

`:DISPlay:MENU <type>` (guide PDF p. 197)

    <type>:= {EMBedded|FLOating}
"""
set_display_menu!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":DISPlay:MENU", type))

"""
    get_display_menu(s)

The query returns the style of menu to display.

`:DISPlay:MENU?` (guide PDF p. 197)

Returns `String`.

Response format:

    <type>

    <type>:= {EMBedded|FLOating}
"""
get_display_menu(s::SiglentScope) =
    _query_str(s, ":DISPlay:MENU?")

"""
    set_display_persistence!(s, time)

The command selects the persistence duration of the display, in seconds, in persistence mode.

`:DISPlay:PERSistence <time>` (guide PDF p. 198)

    <time>:= {OFF|INFinite|1S|5S|10S|30S}
"""
set_display_persistence!(s::SiglentScope, time) =
    scpi_write(s, _cmd(":DISPlay:PERSistence", time))

"""
    get_display_persistence(s)

The query returns the current status of the persistence setting.

`:DISPlay:PERSistence?` (guide PDF p. 198)

Returns `String`.

Response format:

    <time>

    <time>:= {OFF|INFinite|1S|5S|10S|30S}
"""
get_display_persistence(s::SiglentScope) =
    _query_str(s, ":DISPlay:PERSistence?")

"""
    set_display_transparence!(s, value)

This command sets the transparency level of the information bar.

`:DISPlay:TRANsparence <value>` (guide PDF p. 199)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 100]. 0 is the
    least transparent and 100 is the most transparent.
"""
set_display_transparence!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":DISPlay:TRANsparence", value))

"""
    get_display_transparence(s)

The query returns the transparency level of the current information bar.

`:DISPlay:TRANsparence?` (guide PDF p. 199)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_display_transparence(s::SiglentScope) =
    _query_int(s, ":DISPlay:TRANsparence?")

"""
    set_display_type!(s, type)

The command sets the interpolation lines between data points.

`:DISPlay:TYPE <type>` (guide PDF p. 200)

    <type>:= {VECTor|DOT}
    VECTor is the default mode and draws lines between points.
    DOT mode displays data more quickly than vector mode but
    does not draw lines between sample points.
"""
set_display_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":DISPlay:TYPE", type))

"""
    get_display_type(s)

The query returns the interpolation lines between data points.

`:DISPlay:TYPE?` (guide PDF p. 200)

Returns `String`.

Response format:

    <type>

    <type>:= {VECTor|DOT}
"""
get_display_type(s::SiglentScope) =
    _query_str(s, ":DISPlay:TYPE?")

# ---------------------------------------------------------------------- #
# DVM commands                                                           #
# ---------------------------------------------------------------------- #

export set_dvm!, get_dvm, set_dvm_alarm!, get_dvm_alarm, set_dvm_arange!, get_dvm_arange,
      get_dvm_current, set_dvm_hold!, get_dvm_hold, set_dvm_mode!, get_dvm_mode,
      set_dvm_source!, get_dvm_source

"""
    set_dvm!(s, state)

This command sets the switch of the dvm function.

`:DVM <state>` (guide PDF p. 202)

    <state>:= {ON|OFF}
"""
set_dvm!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DVM", state))

"""
    get_dvm(s)

The query returns the current state of the dvm.

`:DVM?` (guide PDF p. 202)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_dvm(s::SiglentScope) =
    _query_bool(s, ":DVM?")

"""
    set_dvm_alarm!(s, state)

This command sets the switch of the overload alarm. When enabled, an alarm will be given if the signal amplitude exceeds the screen range.

`:DVM:ALARm <state>` (guide PDF p. 203)

    <state>:= {ON|OFF}
"""
set_dvm_alarm!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DVM:ALARm", state))

"""
    get_dvm_alarm(s)

The query returns the switch of the overload arm.

`:DVM:ALARm?` (guide PDF p. 203)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}.
"""
get_dvm_alarm(s::SiglentScope) =
    _query_bool(s, ":DVM:ALARm?")

"""
    set_dvm_arange!(s, state)

This command sets the auto range state for the dvm.

`:DVM:ARANge <state>` (guide PDF p. 204)

    <state>:= {ON|OFF}
"""
set_dvm_arange!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DVM:ARANge", state))

"""
    get_dvm_arange(s)

The query returns the auto range state for the dvm.

`:DVM:ARANge?` (guide PDF p. 204)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_dvm_arange(s::SiglentScope) =
    _query_bool(s, ":DVM:ARANge?")

"""
    get_dvm_current(s)

The query returns the displayed 3-digit DVM value based on the current mode.

`:DVM:CURRent?` (guide PDF p. 205)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_dvm_current(s::SiglentScope) =
    _query_float(s, ":DVM:CURRent?")

"""
    set_dvm_hold!(s, state)

This command sets the hold switch of dvm. When enabled, the measured display value will remain unchanged.

`:DVM:HOLD <state>` (guide PDF p. 206)

    <state>:= {ON|OFF}
"""
set_dvm_hold!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":DVM:HOLD", state))

"""
    get_dvm_hold(s)

The query returns the current hold switch of dvm.

`:DVM:HOLD?` (guide PDF p. 206)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_dvm_hold(s::SiglentScope) =
    _query_bool(s, ":DVM:HOLD?")

"""
    set_dvm_mode!(s, mode)

This command sets the digital voltmeter (DVM) mode.

`:DVM:MODE <mode>` (guide PDF p. 207)

    <mode>:= {DCavg|DCRMs|ACRMs|PKPK|AMPLitude}
    - DCavg displays the DC value of the acquired data.
    - DCRMs displays the root-mean-square value of the
    acquired data.
    - ACRMs displays the root-mean-square value of the
    acquired data, with the DC component removed.
    - PKPK displays the difference between maximum and
    minimum data values
    - AMPLitude displays difference between top and base in a
    bimodal waveform. If not bimodal, displays difference
    between max and min
"""
set_dvm_mode!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":DVM:MODE", mode))

"""
    get_dvm_mode(s)

The query returns the current digital voltmeter (DVM) mode:.

`:DVM:MODE?` (guide PDF p. 207)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DCavg|DCRMs|ACRMs|PKPK|AMPLitude}
"""
get_dvm_mode(s::SiglentScope) =
    _query_str(s, ":DVM:MODE?")

"""
    set_dvm_source!(s, source)

This command sets the select the analog channel on which digital voltmeter (DVM) measurements are made.

`:DVM:SOURce <source>` (guide PDF p. 208)

    <source>:= {C<x>}
    - C is analog channel <x>

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_dvm_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":DVM:SOURce", source))

"""
    get_dvm_source(s)

The query returns the current source of dvm.

`:DVM:SOURce?` (guide PDF p. 208)

Returns `String`.

Response format:

    <source>

    <source>:= {Cx}
"""
get_dvm_source(s::SiglentScope) =
    _query_str(s, ":DVM:SOURce?")

# ---------------------------------------------------------------------- #
# FUNCtion commands                                                      #
# ---------------------------------------------------------------------- #

export set_function_fftdisplay!, get_function_fftdisplay, set_function_gvalue!,
      get_function_gvalue, set_function!, get_function, set_function_average_num!,
      get_function_average_num, set_function_diff_dx!, get_function_diff_dx,
      set_function_eres_bits!, get_function_eres_bits, function_fft_autoset!,
      set_function_fft_hcenter!, get_function_fft_hcenter, get_function_fft_hscale,
      get_function_fft_span, set_function_fft_load!, get_function_fft_load,
      set_function_fft_mode!, get_function_fft_mode, set_function_fft_points!,
      get_function_fft_points, function_fft_reset!, set_function_fft_rlevel!,
      get_function_fft_rlevel, set_function_fft_scale!, get_function_fft_scale,
      set_function_fft_search!, get_function_fft_search, set_function_fft_search_excursion!,
      get_function_fft_search_excursion, get_function_fft_search_result,
      set_function_fft_search_threshold!, get_function_fft_search_threshold,
      set_function_fft_unit!, get_function_fft_unit, set_function_fft_window!,
      get_function_fft_window, set_function_filter_type!, get_function_filter_type,
      set_function_filter_hfrequency!, get_function_filter_hfrequency,
      set_function_filter_lfrequency!, get_function_filter_lfrequency,
      set_function_integrate_gate!, get_function_integrate_gate,
      set_function_integrate_offset!, get_function_integrate_offset,
      set_function_interpolate_coef!, get_function_interpolate_coef, set_function_invert!,
      get_function_invert, set_function_label!, get_function_label, set_function_label_text!,
      get_function_label_text, set_function_maxhold_sweeps!, get_function_maxhold_sweeps,
      set_function_minhold_sweeps!, get_function_minhold_sweeps, set_function_operation!,
      get_function_operation, set_function_position!, get_function_position,
      set_function_scale!, get_function_scale, set_function_source1!, get_function_source1,
      set_function_source2!, get_function_source2

"""
    set_function_fftdisplay!(s, mode)

This command sets the display mode of the FFT waveform.

`:FUNCtion:FFTDisplay <mode>` (guide PDF p. 211)

    <mode>:= {SPLit|FULL|EXCLusive}
    - SPLit means that the channel waveform and the FFT
    waveform are displayed on the screen separately.
    - FULL means a full-screen display of the FFT waveform.
    - EXCLusive means that only the FFT waveform is
    displayed on the screen.
"""
set_function_fftdisplay!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":FUNCtion:FFTDisplay", mode))

"""
    get_function_fftdisplay(s)

This query returns the current display mode of the FFT waveform.

`:FUNCtion:FFTDisplay?` (guide PDF p. 211)

Returns `String`.

Response format:

    <mode>

    <mode>:= {SPLit|FULL|EXCLusive}
"""
get_function_fftdisplay(s::SiglentScope) =
    _query_str(s, ":FUNCtion:FFTDisplay?")

"""
    set_function_gvalue!(s, valuea, valueb)

The command sets the integration threshold value of gate A and gate B.

`:FUNCtion:GVALue <valueA>,<valueB>` (guide PDF p. 212)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-horizontal_grid/2*timebase, horizontal_grid/2*timebase].

    Note:
    The value of GA cannot be greater than that of GB. If you set
    the value greater than GB, it will automatically be set to the
    same value as GB.
"""
set_function_gvalue!(s::SiglentScope, valuea, valueb) =
    scpi_write(s, _cmd(":FUNCtion:GVALue", valuea, valueb))

"""
    get_function_gvalue(s)

The query returns the current integration threshold values.

`:FUNCtion:GVALue?` (guide PDF p. 212)

Returns `String`.

Response format:

    <valueA>,<valueB>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_gvalue(s::SiglentScope) =
    _query_str(s, ":FUNCtion:GVALue?")

"""
    set_function!(s, n, state)

This command set the switch of the math function.

`:FUNCtion<n> <state>` (guide PDF p. 213)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <state>:= {ON|OFF}
"""
set_function!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":FUNCtion$(n)", state))

"""
    get_function(s, n)

This query returns the current state of the math function.

`:FUNCtion<n>?` (guide PDF p. 213)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_function(s::SiglentScope, n) =
    _query_bool(s, ":FUNCtion$(n)?")

"""
    set_function_average_num!(s, n, num)

This command sets the average number for the average operation.

`:FUNCtion<n>:AVERage:NUM <num>` (guide PDF p. 214)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <num>:= vary from models, see the table below for details.
    Model <num>
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    {4|16|32|64|128|256|512|1024|2048
    |4096|8192}
    SDS2000X Plus {4|16|32|64|128|256|512|1024}
"""
set_function_average_num!(s::SiglentScope, n, num) =
    scpi_write(s, _cmd(":FUNCtion$(n):AVERage:NUM", num))

"""
    get_function_average_num(s, n)

This query returns the current average number for the average operation.

`:FUNCtion<n>:AVERage:NUM?` (guide PDF p. 214)

Returns `String`.

Response format:

    <num>
"""
get_function_average_num(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):AVERage:NUM?")

"""
    set_function_diff_dx!(s, n, dx)

This command sets the step size of the differential operation.

`:FUNCtion<n>:DIFF:DX <dx>` (guide PDF p. 215)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command

    <dx>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
set_function_diff_dx!(s::SiglentScope, n, dx) =
    scpi_write(s, _cmd(":FUNCtion$(n):DIFF:DX", dx))

"""
    get_function_diff_dx(s, n)

This query returns the current step size of the differential operation.

`:FUNCtion<n>:DIFF:DX?` (guide PDF p. 215)

Returns `Int`.

Response format:

    <dx>

    <dx>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_function_diff_dx(s::SiglentScope, n) =
    _query_int(s, ":FUNCtion$(n):DIFF:DX?")

"""
    set_function_eres_bits!(s, n, bits)

This command sets the eres bits for the eres operation.

`:FUNCtion<n>:ERES:BITS <bits>` (guide PDF p. 216)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <bits>:= {0.5|1.0|1.5|2.0|2.5|3.0}
"""
set_function_eres_bits!(s::SiglentScope, n, bits) =
    scpi_write(s, _cmd(":FUNCtion$(n):ERES:BITS", bits))

"""
    get_function_eres_bits(s, n)

This query returns the current eres bits for the eres operation.

`:FUNCtion<n>:ERES:BITS?` (guide PDF p. 216)

Returns `String`.

Response format:

    <bits>:= {0.5|1.0|1.5|2.0|2.5|3.0}
"""
get_function_eres_bits(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):ERES:BITS?")

"""
    function_fft_autoset!(s, n, mode)

This command causes the FFT waveform to be displayed at the best position on the screen.

`:FUNCtion<n>:FFT:AUToset <mode>` (guide PDF p. 217)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to on FUNCtion and defines the math that is affected by
    the command.

    <mode>:= {SPAN|PEAK|NORMal}
    - SPAN – full span.
    - PEAK – center to peak.
    - NORMal –center set to the fundamental frequency and the
    span is set to one-half of the fft sampling rate
"""
function_fft_autoset!(s::SiglentScope, n, mode) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:AUToset", mode))

"""
    set_function_fft_hcenter!(s, n, center)

This command sets the center frequency of FFT.

`:FUNCtion<n>:FFT:HCENter <center>` (guide PDF p. 218)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <center>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The range of legal values varies with the value set by the
    command :TIMebase:SCALe.
"""
set_function_fft_hcenter!(s::SiglentScope, n, center) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:HCENter", center))

"""
    get_function_fft_hcenter(s, n)

This query returns the current center frequency of FFT.

`:FUNCtion<n>:FFT:HCENter?` (guide PDF p. 218)

Returns `Float64`.

Response format:

    <center>

    <center>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_hcenter(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:HCENter?")

"""
    get_function_fft_hscale(s, n)

This query returns the current horizontal scale of FFT.

`:FUNCtion<n>:FFT:HSCale?` (guide PDF p. 219)

Returns `Float64`.

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

Response format:

    <scale>

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_hscale(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:HSCale?")

"""
    get_function_fft_span(s, n)

This query returns the current horizontal span of FFT.

`:FUNCtion<n>:FFT:SPAN?` (guide PDF p. 220)

Returns `Float64`.

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

Response format:

    <span>

    <span>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_span(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:SPAN?")

"""
    set_function_fft_load!(s, n, load)

This command sets the external load of the FFT.

`:FUNCtion<n>:FFT:LOAD <load>` (guide PDF p. 221)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <load>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 1000000]

    Note:
    The load can be set only when the FFT unit is dBm.
"""
set_function_fft_load!(s::SiglentScope, n, load) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:LOAD", load))

"""
    get_function_fft_load(s, n)

This query returns the current external load of FFT.

`:FUNCtion<n>:FFT:LOAD?` (guide PDF p. 221)

Returns `Int`.

Response format:

    <load>

    <load>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_function_fft_load(s::SiglentScope, n) =
    _query_int(s, ":FUNCtion$(n):FFT:LOAD?")

"""
    set_function_fft_mode!(s, n, mode)

This command selects the acquisition mode of the FFT operation.

`:FUNCtion<n>:FFT:MODE <mode>` (guide PDF p. 222)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <mode>:= {NORMal|MAXHold|AVERage[,<num>]}
    - NORMal sets the FFT in the normal mode.
    - MAXHold sets the FFT in the max detect mode.
    - AVERage sets the FFT in the averaging mode.

    <num>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    The range of the value is [4, 1024].
"""
set_function_fft_mode!(s::SiglentScope, n, mode) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:MODE", mode))

"""
    get_function_fft_mode(s, n)

This query returns the current acquisition mode of the FFT operation.

`:FUNCtion<n>:FFT:MODE?` (guide PDF p. 222)

Returns `String`.

Response format:

    <mode>

    <mode>:= {NORMal|MAXHold|AVERage[,<num>]}

    <num>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_function_fft_mode(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:MODE?")

"""
    set_function_fft_points!(s, n, point)

This command sets the maximum number of points for the FFT operation.

`:FUNCtion<n>:FFT:POINts <point>` (guide PDF p. 223)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <point>:= Vary from models, see the table below for details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    {1k|2k|4k|8k|16k|32k|64k|128k|256k
    |512k|1M|2M|4M|8M}
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    {1k|2k|4k|8k|16k|32k|64k|128k|256k
    |512k|1M|2M}
    SHS800X
    SHS1000X
    {1k|2k|4k|8k|16k|32k|64k|128k|256k
    |512k|1M}
"""
set_function_fft_points!(s::SiglentScope, n, point) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:POINts", point))

"""
    get_function_fft_points(s, n)

This query returns the current maximum number of points for the FFT operation.

`:FUNCtion<n>:FFT:POINts?` (guide PDF p. 223)

Returns `String`.

Response format:

    <point>
"""
get_function_fft_points(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:POINts?")

"""
    function_fft_reset!(s, n)

This command restarts counting when the acquisition mode is average.

`:FUNCtion<n>:FFT:RESET` (guide PDF p. 224)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.
"""
function_fft_reset!(s::SiglentScope, n) =
    scpi_write(s, ":FUNCtion$(n):FFT:RESET")

"""
    set_function_fft_rlevel!(s, n, level)

The command sets the reference level of the FFT operation.

`:FUNCtion<n>:FFT:RLEVel <level>` (guide PDF p. 225)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <level>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the values is related to the probe of the FFT
    source.
    Probe dBVrms Vrms dBm
    1E6 X [-40,200] [1E-2,1E10] [-27,213]
    1E5 X [-60,180] [1E-3,1E9] [-47,193]
    1E4 X [-80,160] [1E-4,1E8] [-67,173]
    1000X [-100,140] [1E-5,1E7] [-87,153]
    100X [-120,120] [1E-6,1E6] [-107,133]
    10X [-140,100] [1E-7,1E5] [-127,113]
    1 [-160,80] [1E-8,1E4] [-147,93]
    0.1X [-180,60] [1E-9,1E3] [-167,73]
    0.01X [-200,40] [1E-10,1E2] [-187,53]
    1E-3 X [-220,20] [1E-11,10] [-207,33]
    1E-4 X [-240,0] [1E-12,1] [-227,13]
    1E-5 X [-260,-20] [1E-13,1E-1] [-247,-7]
    1E-6 X [-280,-40] [1E-14,1E-2] [-267,-27]

    Note:
    The smaller the :FUNCtion<n>:FFT:SCALe, the greater the
    accuracy of the level value.
"""
set_function_fft_rlevel!(s::SiglentScope, n, level) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:RLEVel", level))

"""
    get_function_fft_rlevel(s, n)

The query returns the current reference level of the FFT operation.

`:FUNCtion<n>:FFT:RLEVel?` (guide PDF p. 225)

Returns `Float64`.

Response format:

    <level>

    <level>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_rlevel(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:RLEVel?")

"""
    set_function_fft_scale!(s, n, scale)

The command sets the vertical scale of the FFT.

`:FUNCtion<n>:FFT:SCALe <scale>` (guide PDF p. 227)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the values is related to the vertical unit.
    Unit Range
    dBVrms [1.00E-01, 2.00E+01]
    Vrms [1.00E-03, 1.00E+01]
    dBm [1.00E-01, 2.00E+01]
"""
set_function_fft_scale!(s::SiglentScope, n, scale) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:SCALe", scale))

"""
    get_function_fft_scale(s, n)

The query returns the current vertical scale of FFT.

`:FUNCtion<n>:FFT:SCALe?` (guide PDF p. 227)

Returns `Float64`.

Response format:

    <scale>

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_scale(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:SCALe?")

"""
    set_function_fft_search!(s, n, type)

This command selects the search tools type of the FFT operation.

`:FUNCtion<n>:FFT:SEARch <type>` (guide PDF p. 228)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <type>:= {OFF|PEAK|MARKer}
"""
set_function_fft_search!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:SEARch", type))

"""
    get_function_fft_search(s, n)

This query returns the current search tools type of the FFT operation.

`:FUNCtion<n>:FFT:SEARch?` (guide PDF p. 228)

Returns `String`.

Response format:

    <type>

    <type>:= {OFF|PEAK|MARKer}
"""
get_function_fft_search(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:SEARch?")

"""
    set_function_fft_search_excursion!(s, n, value)

This command sets the search excursion of the search tool (marker or peak) for the FFT operation.

`:FUNCtion<n>:FFT:SEARch:EXCursion <value>` (guide PDF p. 229)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the values is [0, 1.60E+02] when the FFT unit is
    dBVrms. The value range varies with the corresponding unit.

    Note:
    The range of values varies with the value set by
    the :CHANnel<n>:PROBe commands.
"""
set_function_fft_search_excursion!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:SEARch:EXCursion", value))

"""
    get_function_fft_search_excursion(s, n)

This query returns the current search excursion of the search tool for the FFT operation.

`:FUNCtion<n>:FFT:SEARch:EXCursion?` (guide PDF p. 229)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_search_excursion(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:SEARch:EXCursion?")

"""
    get_function_fft_search_result(s, n)

The query returns the current search list result for the FFT operation. It only contains search number, frequency and amplitude information.

`:FUNCtion<n>:FFT:SEARch:RESult?` (guide PDF p. 230)

Returns `String`.

Response format:

    <type>,<no>,<freq>,<ampl>;

    <type>:={Markers|Peaks}
    <no>:= Value in NR1 format, indicates the peak number or
    marker number
    <freq>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
    <ampl>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The unit is the same as FFT vertical
    unit
"""
get_function_fft_search_result(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:SEARch:RESult?")

"""
    set_function_fft_search_threshold!(s, n, value)

The command sets the search threshold of the search tool (marker or peak) for the FFT operation.

`:FUNCtion<n>:FFT:SEARch:THReshold <value>` (guide PDF p. 231)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the values is [-1.60E+02, 8.00E+01], when FFT
    unit is dBVrms. The value changes to match the set Units
    value.
"""
set_function_fft_search_threshold!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:SEARch:THReshold", value))

"""
    get_function_fft_search_threshold(s, n)

The query returns the current search threshold of the search tool for the FFT operation.

`:FUNCtion<n>:FFT:SEARch:THReshold?` (guide PDF p. 231)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_fft_search_threshold(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FFT:SEARch:THReshold?")

"""
    set_function_fft_unit!(s, n, unit)

This command sets the unit type of the FFT operation.

`:FUNCtion<n>:FFT:UNIT <unit>` (guide PDF p. 232)

    <n>:= 1 to (# math functions) in NR1 format is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <unit>:= {DBVrms|Vrms|DBm}
"""
set_function_fft_unit!(s::SiglentScope, n, unit) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:UNIT", unit))

"""
    get_function_fft_unit(s, n)

This query returns the current unit type of the FFT operation.

`:FUNCtion<n>:FFT:UNIT?` (guide PDF p. 232)

Returns `String`.

Response format:

    <unit>

    <unit>:= {DBVrms|Vrms|DBm}
"""
get_function_fft_unit(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:UNIT?")

"""
    set_function_fft_window!(s, n, window)

This command selects the window type of the FFT operation.

`:FUNCtion<n>:FFT:WINDow <window>` (guide PDF p. 233)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a suffix
    to FUNCtion and defines the math that is affected by the
    command.

    <window>:=
    {RECTangle|BLACkman|HANNing|HAMMing|FLATtop}
    - RECTangle is useful for transient signals, and signals where
    there are an integral number of cycles in the time record.
    - BLACkman reduces time resolution compared to the
    rectangular window, but it improves the capacity to detect
    smaller impulses due to lower secondary lobes (provides
    minimal spectral leakage).
    - HANNing is useful for frequency resolution and
    general-purpose use. It is good for resolving two frequencies
    that are close together, or for making frequency
    measurements.
    - HAMMing means Hamming.
    - FLATtop is the best for making accurate amplitude
    measurements of frequency peaks.
"""
set_function_fft_window!(s::SiglentScope, n, window) =
    scpi_write(s, _cmd(":FUNCtion$(n):FFT:WINDow", window))

"""
    get_function_fft_window(s, n)

This query returns the current window type of the FFT operation.

`:FUNCtion<n>:FFT:WINDow?` (guide PDF p. 233)

Returns `String`.

Response format:

    <window>

    <window>:=
    {RECTangle|BLACkman|HANNing|HAMMing|FLATtop}
"""
get_function_fft_window(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FFT:WINDow?")

"""
    set_function_filter_type!(s, n, type)

This command selects the filter type of the filter operation.

`:FUNCtion<n>:FILTer:TYPe <type>` (guide PDF p. 234)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a suffix
    to FUNCtion and defines the math that is affected by the
    command.

    <type>:= {LPASs|HPASs|BPASs|BREJect}
    - LPASs - Low pass filter.
    - HPASs - High pass filter.
    - BPASs - Band pass filter.
    - BREJect - Band reject filter.
"""
set_function_filter_type!(s::SiglentScope, n, type) =
    scpi_write(s, _cmd(":FUNCtion$(n):FILTer:TYPe", type))

"""
    get_function_filter_type(s, n)

This query returns the current filter type of the filter operation.

`:FUNCtion<n>:FILTer:TYPe?` (guide PDF p. 234)

Returns `String`.

Response format:

    <type>

    <type>:=  {LPASs|HPASs|BPASs|BREJect}
"""
get_function_filter_type(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):FILTer:TYPe?")

"""
    set_function_filter_hfrequency!(s, n, value)

This command sets the upper frequency of the filter.

The command/query is available only when the filter type is BPASs or BREJect.

`:FUNCtion<n>:FILTer:HFRequency <value>` (guide PDF p. 235)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_function_filter_hfrequency!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):FILTer:HFRequency", value))

"""
    get_function_filter_hfrequency(s, n)

This query returns the current upper frequency of the filter.

`:FUNCtion<n>:FILTer:HFRequency?` (guide PDF p. 235)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_filter_hfrequency(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FILTer:HFRequency?")

"""
    set_function_filter_lfrequency!(s, n, value)

This command sets the lower frequency of the filter.

`:FUNCtion<n>:FILTer:LFRequency <value>` (guide PDF p. 236)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_function_filter_lfrequency!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):FILTer:LFRequency", value))

"""
    get_function_filter_lfrequency(s, n)

This query returns the current lower frequency of the filter.

`:FUNCtion<n>:FILTer:LFRequency?` (guide PDF p. 236)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_filter_lfrequency(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):FILTer:LFRequency?")

"""
    set_function_integrate_gate!(s, n, state)

This command selects whether to enable the threshold of the integral operation.

`:FUNCtion<n>:INTegrate:GATE <state>` (guide PDF p. 237)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <state>:= {ON|OFF}
"""
set_function_integrate_gate!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":FUNCtion$(n):INTegrate:GATE", state))

"""
    get_function_integrate_gate(s, n)

This query returns the threshold status of the integral operation.

`:FUNCtion<n>:INTegrate:GATE?` (guide PDF p. 237)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_function_integrate_gate(s::SiglentScope, n) =
    _query_bool(s, ":FUNCtion$(n):INTegrate:GATE?")

"""
    set_function_integrate_offset!(s, n, offset)

The command sets the dc offset of the integrate operation.

`:FUNCtion<n>:INTegrate:OFFSet <offset>` (guide PDF p. 238)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value is [-1.67E+00, 1.67E+00].
"""
set_function_integrate_offset!(s::SiglentScope, n, offset) =
    scpi_write(s, _cmd(":FUNCtion$(n):INTegrate:OFFSet", offset))

"""
    get_function_integrate_offset(s, n)

The query returns the current dc offset of the integrate operation.

`:FUNCtion<n>:INTegrate:OFFSet?` (guide PDF p. 238)

Returns `Float64`.

Response format:

    <offset>

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_integrate_offset(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):INTegrate:OFFSet?")

"""
    set_function_interpolate_coef!(s, n, coef)

This command sets the upsample coef for the interpolate operation.

`:FUNCtion<n>:INTErpolate:COEF <coef>` (guide PDF p. 239)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <coef>:= {2|5|10|20}
"""
set_function_interpolate_coef!(s::SiglentScope, n, coef) =
    scpi_write(s, _cmd(":FUNCtion$(n):INTErpolate:COEF", coef))

"""
    get_function_interpolate_coef(s, n)

This query returns the current upsample coef for the interpolate operation.

`:FUNCtion<n>:INTErpolate:COEF?` (guide PDF p. 239)

Returns `String`.

Response format:

    <coef>:= {2|5|10|20}
"""
get_function_interpolate_coef(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):INTErpolate:COEF?")

"""
    set_function_invert!(s, n, state)

This command inverts the math waveform.

`:FUNCtion<n>:INVert <state>` (guide PDF p. 240)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <state>:= {ON|OFF}
"""
set_function_invert!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":FUNCtion$(n):INVert", state))

"""
    get_function_invert(s, n)

This query returns whether the math waveform is inverted or not.

`:FUNCtion<n>:INVert?` (guide PDF p. 240)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_function_invert(s::SiglentScope, n) =
    _query_bool(s, ":FUNCtion$(n):INVert?")

"""
    set_function_label!(s, n, state)

This command is to turn the specified math label on or off.

`:FUNCtion<n>:LABel <state>` (guide PDF p. 241)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <state>:= {ON|OFF}
"""
set_function_label!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":FUNCtion$(n):LABel", state))

"""
    get_function_label(s, n)

This query returns the label associated with a particular math function.

`:FUNCtion<n>:LABel?` (guide PDF p. 241)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_function_label(s::SiglentScope, n) =
    _query_bool(s, ":FUNCtion$(n):LABel?")

"""
    set_function_label_text!(s, n, text)

This command sets the selected math label to the string that follows. Setting a label for a math function also adds the name to the label list in non-volatile memory (replacing the oldest label in the list)

`:FUNCtion<n>:LABel:TEXT <string>` (guide PDF p. 242)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <string>:= Quoted string of ASCII text. The length of the string
    is limited to 20.
"""
set_function_label_text!(s::SiglentScope, n, text) =
    scpi_write(s, _cmd(":FUNCtion$(n):LABel:TEXT", _quote(text)))

"""
    get_function_label_text(s, n)

This query returns the current label text of the selected math.

`:FUNCtion<n>:LABel:TEXT?` (guide PDF p. 242)

Returns `String`.

Response format:

    <string>
"""
get_function_label_text(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):LABel:TEXT?")

"""
    set_function_maxhold_sweeps!(s, n, value)

This command sets the sweeps limit for the maxhold operation.

`:FUNCtion<n>:MAXHold:Sweeps <value>` (guide PDF p. 243)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 2147483646].
"""
set_function_maxhold_sweeps!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):MAXHold:SWeeps", value))

"""
    get_function_maxhold_sweeps(s, n)

This query returns the current sweeps limit for the maxhold operation.

`:FUNCtion<n>:MAXHold:Sweeps?` (guide PDF p. 243)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 forma, including an integer and no
    decimal point, like 1.
"""
get_function_maxhold_sweeps(s::SiglentScope, n) =
    _query_int(s, ":FUNCtion$(n):MAXHold:SWeeps?")

"""
    set_function_minhold_sweeps!(s, n, value)

This command sets the sweeps limit for the minhold operation.

`:FUNCtion<n>:MINHold:SWeeps <value>` (guide PDF p. 244)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 2147483646].
"""
set_function_minhold_sweeps!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":FUNCtion$(n):MINHold:SWeeps", value))

"""
    get_function_minhold_sweeps(s, n)

This query returns the current sweeps limit for the minhold operation.

`:FUNCtion<n>:MINHold:Sweeps?` (guide PDF p. 244)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 forma, including an integer and no
    decimal point, like 1.
"""
get_function_minhold_sweeps(s::SiglentScope, n) =
    _query_int(s, ":FUNCtion$(n):MINHold:SWeeps?")

"""
    set_function_operation!(s, n, operation)

This command sets the desired waveform math operation.

`:FUNCtion<n>:OPERation <operation>` (guide PDF p. 245)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <operation>:=
    {ADD|SUBTract|MULTiply|DIVision|INTegrate|DIFF|FFT|SQRT|
    ERES|AVERage|ABSolute|SIGN|IDENtity|NEGation|EXP|TEN|
    LN|LOG|INTErpolate|MAXHold|MINHold|FILTer}
"""
set_function_operation!(s::SiglentScope, n, operation) =
    scpi_write(s, _cmd(":FUNCtion$(n):OPERation", operation))

"""
    get_function_operation(s, n)

This query returns the current operation for the selected function.

`:FUNCtion<n>:OPERation?` (guide PDF p. 245)

Returns `String`.

Response format:

    <operation>

    <operation>:=
    {ADD|SUBTract|MULTiply|DIVision|INTegrate|DIFF|FFT|SQRT|
    ERES|AVERage|ABSolute|SIGN|IDENtity|NEGation|EXP|TEN|
    LN|LOG|INTErpolate|MAXHold|MINHold|FILTer}
"""
get_function_operation(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):OPERation?")

"""
    set_function_position!(s, n, offset)

This command sets the vertical position of the selected math operation (arithmetic and algebra operation).

`:FUNCtion<n>:POSition <offset>` (guide PDF p. 246)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The range of values is uniform and related to an operation.
"""
set_function_position!(s::SiglentScope, n, offset) =
    scpi_write(s, _cmd(":FUNCtion$(n):POSition", offset))

"""
    get_function_position(s, n)

This query returns the current position value for the selected operation.

`:FUNCtion<n>:POSition?` (guide PDF p. 246)

Returns `Float64`.

Response format:

    <offset>

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_position(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):POSition?")

"""
    set_function_scale!(s, n, scale)

The command sets the vertical scale of the selected math operation (arithmetic and algebra operation).

`:FUNCtion<n>:SCALe <scale>` (guide PDF p. 247)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    • The range of the function scale is related to the scale of
    the function source.
    • When the operation is INTegrate and DIFF, the scale range
    is related to the timebase.
"""
set_function_scale!(s::SiglentScope, n, scale) =
    scpi_write(s, _cmd(":FUNCtion$(n):SCALe", scale))

"""
    get_function_scale(s, n)

The query returns the current scale value for the selected operation.

`:FUNCtion<n>:SCALe?` (guide PDF p. 247)

Returns `Float64`.

Response format:

    <scale>

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_function_scale(s::SiglentScope, n) =
    _query_float(s, ":FUNCtion$(n):SCALe?")

"""
    set_function_source1!(s, n, source)

This command sets the source1 of the math operation.

`:FUNCtion<n>:SOURce1 <source>` (guide PDF p. 248)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <source>:= {C<x>|Z<x>|F<x>}
    - C is analog channel <x>
    - Z is zoom channel <x>
    - F is math function <x>, for math-on-math operations

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    • Z<x> is optional only when Zoom is on.
    • FUNCtion<n> cannot set itself as the source.
"""
set_function_source1!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":FUNCtion$(n):SOURce1", source))

"""
    get_function_source1(s, n)

This query returns the current source1 of the math operation.

`:FUNCtion<n>:SOURce1?` (guide PDF p. 248)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|Z<x>|F<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_function_source1(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):SOURce1?")

"""
    set_function_source2!(s, n, source)

This command sets the source2 of the math operation.

`:FUNCtion<n>:SOURce2 <source>` (guide PDF p. 249)

    <n>:= 1 to (# math functions) in NR1 format, is attached as a
    suffix to FUNCtion and defines the math that is affected by the
    command.

    <source>:= {C<x>|Z<x>|F<x>}
    - C is analog channel <x>
    - Z is zoom channel <x>
    - F is math function <x>, for math-on-math operations

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    • Z<x> is optional only when Zoom is on.
    • FUNCtion<n> cannot set itself as the source.
"""
set_function_source2!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":FUNCtion$(n):SOURce2", source))

"""
    get_function_source2(s, n)

This query returns the current source2 of the math operation.

`:FUNCtion<n>:SOURce2?` (guide PDF p. 249)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|Z<x>|F<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_function_source2(s::SiglentScope, n) =
    _query_str(s, ":FUNCtion$(n):SOURce2?")

# ---------------------------------------------------------------------- #
# HISTORy commands                                                       #
# ---------------------------------------------------------------------- #

export set_history!, get_history, set_history_frame!, get_history_frame,
      set_history_interval!, get_history_interval, set_history_list!, get_history_list,
      set_history_play!, get_history_play, get_history_time

"""
    set_history!(s, state)

The command sets the mode of the history function.

`:HISTORy <state>` (guide PDF p. 251)

    <state>:= {ON|OFF}
"""
set_history!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":HISTORy", state))

"""
    get_history(s)

This query returns the current status of the history function.

`:HISTORy?` (guide PDF p. 251)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_history(s::SiglentScope) =
    _query_bool(s, ":HISTORy?")

"""
    set_history_frame!(s, value)

This command sets the number of the history frame.

`:HISTORy:FRAMe <value>` (guide PDF p. 252)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    The maximum number of frames is related to the number of
    samples set for the acquisition (memory depth). More
    points/frame means less total frames available. Fewer
    points/frame equals more frames available.
"""
set_history_frame!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":HISTORy:FRAMe", value))

"""
    get_history_frame(s)

This query returns the current number of history frames.

`:HISTORy:FRAMe?` (guide PDF p. 252)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_history_frame(s::SiglentScope) =
    _query_int(s, ":HISTORy:FRAMe?")

"""
    set_history_interval!(s, value)

This command sets the play interval of the history frame.

`:HISTORy:INTERval <value>` (guide PDF p. 253)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [1.00E-06,1].
"""
set_history_interval!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":HISTORy:INTERval", value))

"""
    get_history_interval(s)

This query returns the current play interval of the history frame.

`:HISTORy:INTERval?` (guide PDF p. 253)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_history_interval(s::SiglentScope) =
    _query_float(s, ":HISTORy:INTERval?")

"""
    set_history_list!(s, state)

This command sets the state of the history list.

`:HISTORy:LIST <state>` (guide PDF p. 254)

    <state>:= {OFF|ON[,<type>]}

    <type>:= {TIME|DELTa}
    - TIME indicates that the time column is displayed by
    sampling time
    - DELTa indicates that the time column is displayed by the
    sampling interval.
"""
set_history_list!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":HISTORy:LIST", state))

"""
    get_history_list(s)

This query returns the current state of the history list.

`:HISTORy:LIST?` (guide PDF p. 254)

Returns `String`.

Response format:

    <state>

    <state>:= {OFF|ON[,<type>]}

    <type>:= {TIME|DELTa}
"""
get_history_list(s::SiglentScope) =
    _query_str(s, ":HISTORy:LIST?")

"""
    set_history_play!(s, state)

This command sets the play state of the history waveform.

`:HISTORy:PLAY <state>` (guide PDF p. 255)

    <state>:= {BACKWards|PAUSe|FORWards}
    - BACKWards indicates that the frame number is played
    from highest frame number to lowest (last-to-first,
    chronologically).
    - FORWards indicates that the frame number is played from
    the lowest frame number to the highest (first-to-last,
    chronologically).
    - PAUSe will pause playback.
"""
set_history_play!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":HISTORy:PLAY", state))

"""
    get_history_play(s)

This query returns the current play state of the history waveform.

`:HISTORy:PLAY?` (guide PDF p. 255)

Returns `String`.

Response format:

    <state>

    <state>:= {BACKWards|PAUSe|FORWards}
"""
get_history_play(s::SiglentScope) =
    _query_str(s, ":HISTORy:PLAY?")

"""
    get_history_time(s)

The query returns the acquire timestamp of the current frame.

`:HISTORy:TIME?` (guide PDF p. 256)

Returns `Int`.

Response format:

    <time>

    <time>:= hours:minutes:seconds.microseconds in NR1 format,
    including an integer and no decimal point, like 1.
"""
get_history_time(s::SiglentScope) =
    _query_int(s, ":HISTORy:TIME?")

# ---------------------------------------------------------------------- #
# MEASure commands                                                       #
# ---------------------------------------------------------------------- #

export set_measure!, get_measure, set_measure_advanced_linenumber!,
      get_measure_advanced_linenumber, set_measure_advanced_p!, get_measure_advanced_p,
      set_measure_advanced_p_source1!, get_measure_advanced_p_source1,
      set_measure_advanced_p_source2!, get_measure_advanced_p_source2,
      get_measure_advanced_p_statistics, set_measure_advanced_p_type!,
      get_measure_advanced_p_type, get_measure_advanced_p_value,
      set_measure_advanced_statistics!, get_measure_advanced_statistics,
      set_measure_advanced_statistics_histogram!, get_measure_advanced_statistics_histogram,
      set_measure_advanced_statistics_maxcount!, get_measure_advanced_statistics_maxcount,
      measure_advanced_statistics_reset!, set_measure_advanced_style!,
      get_measure_advanced_style, set_measure_gate!, get_measure_gate, set_measure_gate_ga!,
      get_measure_gate_ga, set_measure_gate_gb!, get_measure_gate_gb, set_measure_mode!,
      get_measure_mode, set_measure_simple_item!, set_measure_simple_source!,
      get_measure_simple_source, get_measure_simple_value, set_measure_threshold_source!,
      get_measure_threshold_source, set_measure_threshold_type!, get_measure_threshold_type,
      set_measure_threshold_absolute!, get_measure_threshold_absolute,
      set_measure_threshold_percent!, get_measure_threshold_percent

"""
    set_measure!(s, state)

The command sets the state of the measurement function.

`:MEASure <state>` (guide PDF p. 258)

    <state>:= {ON|OFF}
"""
set_measure!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MEASure", state))

"""
    get_measure(s)

This query returns the current state of the measurement function.

`:MEASure?` (guide PDF p. 258)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_measure(s::SiglentScope) =
    _query_bool(s, ":MEASure?")

"""
    set_measure_advanced_linenumber!(s, value)

The command sets the total number of advanced measurement items displayed.

`:MEASure:ADVanced:LINenumber <value>` (guide PDF p. 259)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 12].
"""
set_measure_advanced_linenumber!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":MEASure:ADVanced:LINenumber", value))

"""
    get_measure_advanced_linenumber(s)

The query returns the current total number of advanced measurement items displayed.

`:MEASure:ADVanced:LINenumber?` (guide PDF p. 259)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_measure_advanced_linenumber(s::SiglentScope) =
    _query_int(s, ":MEASure:ADVanced:LINenumber?")

"""
    set_measure_advanced_p!(s, n, state)

This command sets the state of the specified measurement item.

`:MEASure:ADVanced:P<n> <state>` (guide PDF p. 260)

    <n>:= 1 to 12

    <state>:= {ON|OFF}
"""
set_measure_advanced_p!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":MEASure:ADVanced:P$(n)", state))

"""
    get_measure_advanced_p(s, n)

This query returns the current state of the measurement item.

`:MEASure:ADVanced:P<n>?` (guide PDF p. 260)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_measure_advanced_p(s::SiglentScope, n) =
    _query_bool(s, ":MEASure:ADVanced:P$(n)?")

"""
    set_measure_advanced_p_source1!(s, n, source)

This command sets the source1 of the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:SOURce1 <source>` (guide PDF p. 261)

    <n>:= 1 to 12

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}
    - C denotes an analog input channel.
    - Z denotes a zoomed input.
    - F denotes a math function.
    - D denotes a digital input channel.
    - ZD denotes a zoomed digital input channel.
    - REF denotes a reference waveform.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    • Z<x> and ZD<m> are optional only when Zoom is on.
    • The source can only be set to C<x> when the type is delay
    measurement.
"""
set_measure_advanced_p_source1!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":MEASure:ADVanced:P$(n):SOURce1", source))

"""
    get_measure_advanced_p_source1(s, n)

This query returns the current source1 of the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:SOURce1?` (guide PDF p. 261)

Returns `String`.

Response format:

    <source>

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_measure_advanced_p_source1(s::SiglentScope, n) =
    _query_str(s, ":MEASure:ADVanced:P$(n):SOURce1?")

"""
    set_measure_advanced_p_source2!(s, n, source)

This command sets the source2 of the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:SOURce2 <source>` (guide PDF p. 263)

    <n>:= 1 to 12

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - Z denotes a zoomed waveform. For example, Z1 is zoom
    waveform 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.
    - ZD denotes a zoomed digital input.
    - REF denotes a reference waveform.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    • Z<x> and ZD<m> are optional only when Zoom is on.
    • The source can only be set to C<x> when the type is delay
    measurement.
"""
set_measure_advanced_p_source2!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":MEASure:ADVanced:P$(n):SOURce2", source))

"""
    get_measure_advanced_p_source2(s, n)

This query returns the source2 of the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:SOURce2?` (guide PDF p. 263)

Returns `String`.

Response format:

    <source>

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_measure_advanced_p_source2(s::SiglentScope, n) =
    _query_str(s, ":MEASure:ADVanced:P$(n):SOURce2?")

"""
    get_measure_advanced_p_statistics(s, n, type)

This query returns statistics for the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:STATistics? <type>` (guide PDF p. 265)

Returns `Float64`.

    <n>:= 1 to 12

    <type>:=
    {ALL|CURRent|MEAN|MAXimum|MINimum|STDev|COUNt}
    - ALL returns all the statistics
    - CURRent returns the current value of the statistics
    - MEAN returns the mean value of the statistics
    - MAXimum returns the maximum value of the statistics
    - MINimum returns the minimum value of the statistics
    - STDev returns the standard deviation of the statistics
    - COUNt returns the current number of counts used to
    calculate the statistical data

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    When measurement statistics are off, it returns OFF.
"""
get_measure_advanced_p_statistics(s::SiglentScope, n, type) =
    _query_float(s, _cmd(":MEASure:ADVanced:P$(n):STATistics?", type))

"""
    set_measure_advanced_p_type!(s, n, parameter)

This command sets the type for the specified measurement item.

`:MEASure:ADVanced:P<n>:TYPE <parameter>` (guide PDF p. 266)

    <n>:= 1 to 12

    <parameter>:=
    {PKPK|MAX|MIN|AMPL|TOP|BASE|LEVELX|CMEAN|MEAN|S
    TDEV|VSTD|RMS|CRMS|MEDIAN|CMEDIAN|OVSN|FPRE|O
    VSP|RPRE|PER|FREQ|TMAX|TMIN|PWID|NWID|DUTY|NDU
    TY|WID|NBWID|DELAY|TIMEL|RISE|FALL|RISE10T90|FALL9
    0T10|CCJ|PAREA|NAREA|AREA|ABSAREA|CYCLES|REDGE
    S|FEDGES|EDGES|PPULSES|NPULSES|PHA|SKEW|FRR|F
    RF|FFR|FFF|LRR|LRF|LFR|LFF|PACArea|NACArea|ACArea|A
    BSACArea|PSLOPE|NSLOPE|TSR|TSF|THR|THF}

    Description of Parameters
    Parameter Description
    PKPK Difference between maximum and
    minimum data values
    MAX Highest value in waveform
    MIN Lowest value in waveform
    AMPL
    Difference between top and base in a
    bimodal waveform. If not bimodal,
    difference between max and min
    TOP Value of most probable higher state in a
    bimodal waveform
    BASE Value of most probable lower state in a
    bimodal waveform
    LEVELX Level measured at trigger position
    CMEAN Average value of the first cycle
    MEAN Average of data values
    STDEV Standard deviation of the data
    VSTD Standard deviation of the first cycle
    RMS Root mean square of the data
    CRMS Root mean square of the first cycle
    MEDIAN Value at which 50% of the measurement
    are above and 50% are below
    CMEDIAN Median of the first cycle
    OVSN Overshoot following a falling edge; 100%*
    (base-min)/amplitude
    FPRE Overshoot before a falling edge;
    100%*(max-top)/amplitude
    ...
"""
set_measure_advanced_p_type!(s::SiglentScope, n, parameter) =
    scpi_write(s, _cmd(":MEASure:ADVanced:P$(n):TYPE", parameter))

"""
    get_measure_advanced_p_type(s, n)

This query returns the type for the specified measurement item.

`:MEASure:ADVanced:P<n>:TYPE?` (guide PDF p. 266)

Returns `String`.

Response format:

    <parameter>
"""
get_measure_advanced_p_type(s::SiglentScope, n) =
    _query_str(s, ":MEASure:ADVanced:P$(n):TYPE?")

"""
    get_measure_advanced_p_value(s, n)

The query returns the value of the specified advanced measurement item.

`:MEASure:ADVanced:P<n>:VALue?` (guide PDF p. 269)

Returns `Float64`.

    <n>:= 1 to 12

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_measure_advanced_p_value(s::SiglentScope, n) =
    _query_float(s, ":MEASure:ADVanced:P$(n):VALue?")

"""
    set_measure_advanced_statistics!(s, state)

The command sets the state of the measurement statistics.

`:MEASure:ADVanced:STATistics <state>` (guide PDF p. 270)

    <state>:= {ON|OFF}
"""
set_measure_advanced_statistics!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MEASure:ADVanced:STATistics", state))

"""
    get_measure_advanced_statistics(s)

This query returns the current state of the measurement statistics function.

`:MEASure:ADVanced:STATistics?` (guide PDF p. 270)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_measure_advanced_statistics(s::SiglentScope) =
    _query_bool(s, ":MEASure:ADVanced:STATistics?")

"""
    set_measure_advanced_statistics_histogram!(s, state)

The command sets the state of the histogram function.

`:MEASure:ADVanced:STATistics:HISTOGram <state>` (guide PDF p. 271)

    <state>:= {ON|OFF}
"""
set_measure_advanced_statistics_histogram!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MEASure:ADVanced:STATistics:HISTOGram", state))

"""
    get_measure_advanced_statistics_histogram(s)

This query returns the current state of the histogram function.

`:MEASure:ADVanced:STATistics:HISTOGram?` (guide PDF p. 271)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_measure_advanced_statistics_histogram(s::SiglentScope) =
    _query_bool(s, ":MEASure:ADVanced:STATistics:HISTOGram?")

"""
    set_measure_advanced_statistics_maxcount!(s, value)

This command sets the maximum value of the statistics count.

`:MEASure:ADVanced:STATistics:MAXCount <value>` (guide PDF p. 272)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 1024].

    Note:
    When the value is set to 0, it means unlimited statistics.
"""
set_measure_advanced_statistics_maxcount!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":MEASure:ADVanced:STATistics:MAXCount", value))

"""
    get_measure_advanced_statistics_maxcount(s)

The query returns the current value of statistics count.

`:MEASure:ADVanced:STATistics:MAXCount?` (guide PDF p. 272)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_measure_advanced_statistics_maxcount(s::SiglentScope) =
    _query_int(s, ":MEASure:ADVanced:STATistics:MAXCount?")

"""
    measure_advanced_statistics_reset!(s)

The command resets the measurement statistics.

`:MEASure:ADVanced:STATistics:RESet` (guide PDF p. 273)
"""
measure_advanced_statistics_reset!(s::SiglentScope) =
    scpi_write(s, ":MEASure:ADVanced:STATistics:RESet")

"""
    set_measure_advanced_style!(s, type)

The command selects the display mode of the advanced measurements.

`:MEASure:ADVanced:STYLe <type>` (guide PDF p. 274)

    <type>:= {M1|M2}
    - M1 lists a measurement, corresponding statistics, and
    histogram vertically on the display.
    - M2 lists a measurement and corresponding statistics
    horizontally on the display. No histogram is available with
    M2.
"""
set_measure_advanced_style!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":MEASure:ADVanced:STYLe", type))

"""
    get_measure_advanced_style(s)

This query returns the current display mode of the advanced measurement.

`:MEASure:ADVanced:STYLe?` (guide PDF p. 274)

Returns `String`.

Response format:

    <type>

    <type>:= {M1|M2}
"""
get_measure_advanced_style(s::SiglentScope) =
    _query_str(s, ":MEASure:ADVanced:STYLe?")

"""
    set_measure_gate!(s, state)

This command sets the state of the measurement gate.

`:MEASure:GATE <state>` (guide PDF p. 275)

    <state>:= {ON|OFF}
"""
set_measure_gate!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MEASure:GATE", state))

"""
    get_measure_gate(s)

This query returns the current state of the measurement gate.

`:MEASure:GATE?` (guide PDF p. 275)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_measure_gate(s::SiglentScope) =
    _query_bool(s, ":MEASure:GATE?")

"""
    set_measure_gate_ga!(s, value)

This command sets the position of gate A.

`:MEASure:GATE:GA <value>` (guide PDF p. 276)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-horizontal_grid/2*timebase, horizontal_grid/2*timebase].

    Note:
    The value of GA cannot be greater than that of GB. If you set the
    value greater than GB, it will automatically be set to the same
    value as GB.
"""
set_measure_gate_ga!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":MEASure:GATE:GA", value))

"""
    get_measure_gate_ga(s)

This query returns the current position of gate A.

`:MEASure:GATE:GA?` (guide PDF p. 276)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_measure_gate_ga(s::SiglentScope) =
    _query_float(s, ":MEASure:GATE:GA?")

"""
    set_measure_gate_gb!(s, value)

This command sets the position of gate B.

This command returns the current position of gate B.

`:MEASure:GATE:GB <value>` (guide PDF p. 277)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is
    [-horizontal_grid/2*timebase, horizontal_grid/2*timebase].

    Note:
    The value of GB cannot be less than that of GA. If you set the
    value less than GA, it will automatically be set to the same
    value as GA.
"""
set_measure_gate_gb!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":MEASure:GATE:GB", value))

"""
    get_measure_gate_gb(s)

This command sets the position of gate B.

This command returns the current position of gate B.

`:MEASure:GATE:GB?` (guide PDF p. 277)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_measure_gate_gb(s::SiglentScope) =
    _query_float(s, ":MEASure:GATE:GB?")

"""
    set_measure_mode!(s, type)

The command specifies the mode of measurement.

`:MEASure:MODE <type>` (guide PDF p. 278)

    <type>:= {SIMPle|ADVanced}
    - SIMPle shows measurements only
    - ADVanced shows measurements and includes selections
    for statistics, view mode (M1, M2), histogram, and
    trending.
"""
set_measure_mode!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":MEASure:MODE", type))

"""
    get_measure_mode(s)

The query returns the current mode of measurement.

`:MEASure:MODE?` (guide PDF p. 278)

Returns `String`.

Response format:

    <type>

    <type>:= {SIMPle|ADVanced}
"""
get_measure_mode(s::SiglentScope) =
    _query_str(s, ":MEASure:MODE?")

"""
    set_measure_simple_item!(s, parameter, state)

This command sets the type of simple measurement.

`:MEASure:SIMPle:ITEM <parameter>,<state>` (guide PDF p. 279)

    <parameter>:=
    {PKPK|MAX|MIN|AMPL|TOP|BASE|LEVELX|CMEAN|MEAN|S
    TDEV|VSTD|RMS|CRMS|MEDIAN|CMEDIAN|OVSN|FPRE|O
    VSP|RPRE|PER|FREQ|TMAX|TMIN|PWID|NWID|DUTY|NDU
    TY|WID|NBWID|DELAY|TIMEL|RISE|FALL|RISE20T80|FALL8
    0T20|CCJ|PAREA|NAREA|AREA|ABSAREA|CYCLES|REDGE
    S|FEDGES|EDGES|PPULSES|NPULSES|PACArea|NACArea|
    ACArea|ABSACArea}

    <state>:= {ON|OFF}

    Note:
    See the table for details.
"""
set_measure_simple_item!(s::SiglentScope, parameter, state) =
    scpi_write(s, _cmd(":MEASure:SIMPle:ITEM", parameter, state))

"""
    set_measure_simple_source!(s, source)

This command sets the source of the simple measurement.

`:MEASure:SIMPle:SOURce <source>` (guide PDF p. 280)

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - Z denotes a zoomed waveform. For example, Z1 is zoom
    waveform 1.
    - F denotes a math function. For example, F1 is math function
    1.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.
    - REF denotes a reference waveform.

    <x>:= 1 to (# analog channels) in NR1 format, including an integer
    and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    Z<x> and ZD<m> are optional only when Zoom is on.
"""
set_measure_simple_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":MEASure:SIMPle:SOURce", source))

"""
    get_measure_simple_source(s)

This query returns the current source of the simple measurement.

`:MEASure:SIMPle:SOURce?` (guide PDF p. 280)

Returns `String`.

Response format:

    <source>

    <source>:=
    {C<x>|Z<x>|F<x>|D<m>|ZD<m>|REFA|REFB|REFC|REFD}

    <x>:= 1 to (# analog channels) in NR1 format, including an integer
    and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_measure_simple_source(s::SiglentScope) =
    _query_str(s, ":MEASure:SIMPle:SOURce?")

"""
    get_measure_simple_value(s, type)

This query returns the specified measurement value that appears on the simple measurement.

`:MEASure:SIMPle:VALue? <type>` (guide PDF p. 281)

Returns `Float64`.

    <type>:=
    {PKPK|MAX|MIN|AMPL|TOP|BASE|LEVELX|CMEAN|MEAN|S
    TDEV|VSTD|RMS|CRMS|MEDIAN|CMEDIAN|OVSN|FPRE|O
    VSP|RPRE|PER|FREQ|TMAX|TMIN|PWID|NWID|DUTY|NDU
    TY|WID|NBWID|DELAY|TIMEL|RISE|FALL|RISE20T80|FALL8
    0T20|CCJ|PAREA|NAREA|AREA|ABSAREA|CYCLES|REDGE
    S|FEDGES|EDGES|PPULSES|NPULSES|PACArea|NACArea|
    ACArea|ABSACArea|ALL}

    Note:
    • See the table for details.
    • ALL is only valid for queries, and it returns all
    measurement values of all measurement types except for
    delay measurements.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_measure_simple_value(s::SiglentScope, type) =
    _query_float(s, _cmd(":MEASure:SIMPle:VALue?", type))

"""
    set_measure_threshold_source!(s, source)

This command sets the measurement threshold source.

`:MEASure:THReshold:SOURce <source>` (guide PDF p. 282)

    <source>:= {C<x>|Z<x>|F<x>|REFA|REFB|REFC|REFD}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - Z denotes a zoomed waveform. For example, Z1 is zoom
    waveform 1.
    - F denotes a math function. For example, F1 is math function
    1.
    - REF denotes a reference waveform.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    Z<x> and ZD<m> are optional only when Zoom is on.
"""
set_measure_threshold_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":MEASure:THReshold:SOURce", source))

"""
    get_measure_threshold_source(s)

This query returns the current measurement threshold source.

`:MEASure:THReshold:SOURce?` (guide PDF p. 282)

Returns `String`.

Response format:

    <source>:= {C<x>|Z<x>|F<x>|REFA|REFB|REFC|REFD}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_measure_threshold_source(s::SiglentScope) =
    _query_str(s, ":MEASure:THReshold:SOURce?")

"""
    set_measure_threshold_type!(s, type)

This command sets the measurement threshold type.

`:MEASure:THReshold:TYPE <type>` (guide PDF p. 283)

    <type>:= {PERCent|ABSolute}
"""
set_measure_threshold_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":MEASure:THReshold:TYPE", type))

"""
    get_measure_threshold_type(s)

This query returns the current measurement threshold type.

`:MEASure:THReshold:TYPE?` (guide PDF p. 283)

Returns `String`.

Response format:

    <type>

    <type>:= {PERCent|ABSolute}
"""
get_measure_threshold_type(s::SiglentScope) =
    _query_str(s, ":MEASure:THReshold:TYPE?")

"""
    set_measure_threshold_absolute!(s, high, mid, low)

This command specifies the reference level when :MEASure:THReshold:TYPE is set to ABSolute.This command affects the results of some measurements.

`:MEASure:THReshold:ABSolute <high>,<mid>,<low>` (guide PDF p. 284)

    <low>:= Value in NR3 format, including a
    decimal point and exponent, like 1.23E+2.
"""
set_measure_threshold_absolute!(s::SiglentScope, high, mid, low) =
    scpi_write(s, _cmd(":MEASure:THReshold:ABSolute", high, mid, low))

"""
    get_measure_threshold_absolute(s)

This query returns the reference level of the source

`:MEASure:THReshold:ABSolute?` (guide PDF p. 284)

Returns `String`.

Response format:

    <high>,<mid>,<low>

    <high>,<mid>,<low>:= Value in NR3 format, including a
    decimal point and exponent, like 1.23E+2.
"""
get_measure_threshold_absolute(s::SiglentScope) =
    _query_str(s, ":MEASure:THReshold:ABSolute?")

"""
    set_measure_threshold_percent!(s, high, mid, low)

This command specifies the percent used to calculate the reference level when :MEASure:THReshold:TYPE is set to PERCent. This command affects the results of some measurements.

`:MEASure:THReshold:PERCent <high>,<mid>,<low>` (guide PDF p. 285)

    <low>:= Value in NR1 format, including an
    integer and no decimal point, like 10
"""
set_measure_threshold_percent!(s::SiglentScope, high, mid, low) =
    scpi_write(s, _cmd(":MEASure:THReshold:PERCent", high, mid, low))

"""
    get_measure_threshold_percent(s)

This command specifies the percent used to calculate the reference level when :MEASure:THReshold:TYPE is set to PERCent. This command affects the results of some measurements.

`:MEASure:THReshold:PERCent?` (guide PDF p. 285)

Returns `String`.

Response format:

    <high>,<mid>,<low>

    <high>,<mid>,<low>:= Value in NR1 format, including an
    integer and no decimal point, like 10
"""
get_measure_threshold_percent(s::SiglentScope) =
    _query_str(s, ":MEASure:THReshold:PERCent?")

# ---------------------------------------------------------------------- #
# MEMory commands                                                        #
# ---------------------------------------------------------------------- #

export set_memory_horizontal_position!, get_memory_horizontal_position,
      set_memory_horizontal_scale!, get_memory_horizontal_scale, set_memory_horizontal_sync!,
      get_memory_horizontal_sync, memory_import!, set_memory_label!, get_memory_label,
      set_memory_label_text!, get_memory_label_text, set_memory_switch!, get_memory_switch,
      set_memory_vertical_position!, get_memory_vertical_position,
      set_memory_vertical_scale!, get_memory_vertical_scale

"""
    set_memory_horizontal_position!(s, n, val)

The command specifies the horizontal position of the memory waveform.

`:MEMory<n>:HORizontal:POSition <val>` (guide PDF p. 287)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <val>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_memory_horizontal_position!(s::SiglentScope, n, val) =
    scpi_write(s, _cmd(":MEMory$(n):HORizontal:POSition", val))

"""
    get_memory_horizontal_position(s, n)

The query returns the current horizontal position of the memory.

`:MEMory<n>:HORizontal:POSition?` (guide PDF p. 287)

Returns `Float64`.

Response format:

    <val>

    <val>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_memory_horizontal_position(s::SiglentScope, n) =
    _query_float(s, ":MEMory$(n):HORizontal:POSition?")

"""
    set_memory_horizontal_scale!(s, n, value)

The command sets the horizontal scale per division for the memory waveform.

`:MEMory<n>:HORizontal:SCALe <value>` (guide PDF p. 288)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_memory_horizontal_scale!(s::SiglentScope, n, value) =
    scpi_write(s, _cmd(":MEMory$(n):HORizontal:SCALe", value))

"""
    get_memory_horizontal_scale(s, n)

The query returns the current horizontal scale setting in seconds per division for the memory.

`:MEMory<n>:HORizontal:SCALe?` (guide PDF p. 288)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_memory_horizontal_scale(s::SiglentScope, n) =
    _query_float(s, ":MEMory$(n):HORizontal:SCALe?")

"""
    set_memory_horizontal_sync!(s, n, state)

The command turns on and off the horizontal parameter synchronization switch. When enabled, modify the horizontal parameters of the imported source, and the parameters of its memory waveform will also be modified synchronously.

`:MEMory<n>:HORizontal:SYNC <state>` (guide PDF p. 289)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
"""
set_memory_horizontal_sync!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":MEMory$(n):HORizontal:SYNC", state))

"""
    get_memory_horizontal_sync(s, n)

This query returns the current state of the horizontal parameter synchronization switch.

`:MEMory<n>:HORizontal:SYNC?` (guide PDF p. 289)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_memory_horizontal_sync(s::SiglentScope, n) =
    _query_bool(s, ":MEMory$(n):HORizontal:SYNC?")

"""
    memory_import!(s, n, source)

The command import the source to the memory waveform.

`:MEMory<n>:IMPort <source>` (guide PDF p. 290)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <source>:= {C<x>|Z<x>|F<x>|M<x>|<path>}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - Z denotes a zoomed waveform. For example, Z1 is zoom
    waveform 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - M denotes a memory waveform. For example, M1 denotes
    Memory 1.
    - <path>:= Quoted string of path with an extension “.bin”,
    denotes a waveform file.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
memory_import!(s::SiglentScope, n, source) =
    scpi_write(s, _cmd(":MEMory$(n):IMPort", source))

"""
    set_memory_label!(s, n, state)

The command is to turn the specified memory label on or off.

`:MEMory<n>:LABel <state>` (guide PDF p. 291)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
"""
set_memory_label!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":MEMory$(n):LABel", state))

"""
    get_memory_label(s, n)

This query returns the label associated with a particular memory function.

`:MEMory<n>:LABel?` (guide PDF p. 291)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_memory_label(s::SiglentScope, n) =
    _query_bool(s, ":MEMory$(n):LABel?")

"""
    set_memory_label_text!(s, n, text)

The command sets the selected memory label to the string that follows. Setting a label for a memory waveform also adds the name to the label list in non-volatile memory (replacing the oldest label in the list)

`:MEMory<n>:LABel:TEXT <string>` (guide PDF p. 292)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <string>:= Quoted string of ASCII text. The length of the string
    is limited to 20.
"""
set_memory_label_text!(s::SiglentScope, n, text) =
    scpi_write(s, _cmd(":MEMory$(n):LABel:TEXT", _quote(text)))

"""
    get_memory_label_text(s, n)

The query returns the current label text of the selected memory waveform.

`:MEMory<n>:LABel:TEXT?` (guide PDF p. 292)

Returns `String`.

Response format:

    <string>
"""
get_memory_label_text(s::SiglentScope, n) =
    _query_str(s, ":MEMory$(n):LABel:TEXT?")

"""
    set_memory_switch!(s, n, state)

The command sets the display of the memory waveform.

`:MEMory<n>:SWITch <state>` (guide PDF p. 293)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <state>:= {ON|OFF}
"""
set_memory_switch!(s::SiglentScope, n, state) =
    scpi_write(s, _cmd(":MEMory$(n):SWITch", state))

"""
    get_memory_switch(s, n)

This query returns the current display of the memory waveform.

`:MEMory<n>:SWITch?` (guide PDF p. 293)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_memory_switch(s::SiglentScope, n) =
    _query_bool(s, ":MEMory$(n):SWITch?")

"""
    set_memory_vertical_position!(s, n, offset)

The command the vertical position of the selected memory waveform.

`:MEMory<n>:VERTical:POSition <offset>` (guide PDF p. 294)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_memory_vertical_position!(s::SiglentScope, n, offset) =
    scpi_write(s, _cmd(":MEMory$(n):VERTical:POSition", offset))

"""
    get_memory_vertical_position(s, n)

This query returns the current position value for the selected memory.

`:MEMory<n>:VERTical:POSition?` (guide PDF p. 294)

Returns `Float64`.

Response format:

    <offset>

    <offset>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_memory_vertical_position(s::SiglentScope, n) =
    _query_float(s, ":MEMory$(n):VERTical:POSition?")

"""
    set_memory_vertical_scale!(s, n, scale)

The command sets the vertical scale of the selected memory waveform.

`:MEMory<n>:VERTical:SCALe <scale>` (guide PDF p. 295)

    <n>:= 1 to (# memory waveforms) in NR1 format, including an
    integer and no decimal point, like 1.

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
set_memory_vertical_scale!(s::SiglentScope, n, scale) =
    scpi_write(s, _cmd(":MEMory$(n):VERTical:SCALe", scale))

"""
    get_memory_vertical_scale(s, n)

The query returns the current scale value for the selected memory waveform.

`:MEMory<n>:VERTical:SCALe?` (guide PDF p. 295)

Returns `Float64`.

Response format:

    <scale>

    <scale>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_memory_vertical_scale(s::SiglentScope, n) =
    _query_float(s, ":MEMory$(n):VERTical:SCALe?")

# ---------------------------------------------------------------------- #
# MTESt commands                                                         #
# ---------------------------------------------------------------------- #

export set_mtest!, get_mtest, get_mtest_count, set_mtest_function_buzzer!,
      get_mtest_function_buzzer, set_mtest_function_cof!, get_mtest_function_cof,
      set_mtest_function_fth!, get_mtest_function_fth, set_mtest_function_sof!,
      get_mtest_function_sof, set_mtest_idisplay!, get_mtest_idisplay, mtest_mask_create!,
      mtest_mask_load!, set_mtest_operate!, get_mtest_operate, mtest_reset!,
      set_mtest_source!, get_mtest_source, set_mtest_type!, get_mtest_type

"""
    set_mtest!(s, state)

The command sets the state of the mask test.

`:MTESt <state>` (guide PDF p. 297)

    <state>:= {ON|OFF}
"""
set_mtest!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt", state))

"""
    get_mtest(s)

This query returns the current state of the mask test.

`:MTESt?` (guide PDF p. 297)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest(s::SiglentScope) =
    _query_bool(s, ":MTESt?")

"""
    get_mtest_count(s)

The query returns the result of the mask test.

`:MTESt:COUNt?` (guide PDF p. 298)

Returns `String`.

Response format:

    FAIL,<num>,PASS,<num>,TOTAL,<num>

    <num>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_mtest_count(s::SiglentScope) =
    _query_str(s, ":MTESt:COUNt?")

"""
    set_mtest_function_buzzer!(s, state)

This command sets the state of the buzzer when failure frames are detected.

This command query returns the status of the buzzer.

`:MTESt:FUNCtion:BUZZer <state>` (guide PDF p. 299)

    <state>:= {ON|OFF}
"""
set_mtest_function_buzzer!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:FUNCtion:BUZZer", state))

"""
    get_mtest_function_buzzer(s)

This command sets the state of the buzzer when failure frames are detected.

This command query returns the status of the buzzer.

`:MTESt:FUNCtion:BUZZer?` (guide PDF p. 299)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest_function_buzzer(s::SiglentScope) =
    _query_bool(s, ":MTESt:FUNCtion:BUZZer?")

"""
    set_mtest_function_cof!(s, state)

This command sets the state of the mask test function "Capture on Fail". When this function is enabled, the default path to save the image of failing frames is “SIGLENT/”.

This command query returns the status of “Capture on Fail”.

`:MTESt:FUNCtion:COF <state>` (guide PDF p. 300)

    <state>:= {OFF|ON}
"""
set_mtest_function_cof!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:FUNCtion:COF", state))

"""
    get_mtest_function_cof(s)

This command sets the state of the mask test function "Capture on Fail". When this function is enabled, the default path to save the image of failing frames is “SIGLENT/”.

This command query returns the status of “Capture on Fail”.

`:MTESt:FUNCtion:COF?` (guide PDF p. 300)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_mtest_function_cof(s::SiglentScope) =
    _query_bool(s, ":MTESt:FUNCtion:COF?")

"""
    set_mtest_function_fth!(s, state)

This command sets the state of the mask test function "Failure to History".

This command query returns the status of “Failure to History”.

`:MTESt:FUNCtion:FTH <state>` (guide PDF p. 301)

    <state>:= {ON|OFF}
"""
set_mtest_function_fth!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:FUNCtion:FTH", state))

"""
    get_mtest_function_fth(s)

This command sets the state of the mask test function "Failure to History".

This command query returns the status of “Failure to History”.

`:MTESt:FUNCtion:FTH?` (guide PDF p. 301)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest_function_fth(s::SiglentScope) =
    _query_bool(s, ":MTESt:FUNCtion:FTH?")

"""
    set_mtest_function_sof!(s, state)

This command sets the state of the mask test function “Stop-on-Fail”.

This command query returns the status of “Stop- on-Fail”.

`:MTESt:FUNCtion:SOF <state>` (guide PDF p. 302)

    <state>:= {ON|OFF}
"""
set_mtest_function_sof!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:FUNCtion:SOF", state))

"""
    get_mtest_function_sof(s)

This command sets the state of the mask test function “Stop-on-Fail”.

This command query returns the status of “Stop- on-Fail”.

`:MTESt:FUNCtion:SOF?` (guide PDF p. 302)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest_function_sof(s::SiglentScope) =
    _query_bool(s, ":MTESt:FUNCtion:SOF?")

"""
    set_mtest_idisplay!(s, state)

This command sets the state of the mask test result display.

This command query returns the status of the mask test result display.

`:MTESt:IDISplay <state>` (guide PDF p. 303)

    <state>:= {ON|OFF}
"""
set_mtest_idisplay!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:IDISplay", state))

"""
    get_mtest_idisplay(s)

This command sets the state of the mask test result display.

This command query returns the status of the mask test result display.

`:MTESt:IDISplay?` (guide PDF p. 303)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest_idisplay(s::SiglentScope) =
    _query_bool(s, ":MTESt:IDISplay?")

"""
    mtest_mask_create!(s, xmargin, ymargin)

This command sets the mask X and mask Y of mask test.

`:MTESt:MASK:CREate <XMARgin>,<YMARgin>` (guide PDF p. 304)

    <XMARgin>:= Value in NR2 format. The range of the value is
    [0.08, 4.00]

    <YMARgin>:= Value in NR2 format. The range of the value is
    [0.08, 4.00]
"""
mtest_mask_create!(s::SiglentScope, xmargin, ymargin) =
    scpi_write(s, _cmd(":MTESt:MASK:CREate", xmargin, ymargin))

"""
    mtest_mask_load!(s, location)

The command recalls the mask from internal or external memory locations.

`:MTESt:MASK:LOAD <location>` (guide PDF p. 305)

    <location>:= {INTernal,<num>|EXTernal,<path>}

    <num>:= {1|2|3|4}

    <path>:= Quoted string of path name with an extension “.msk”
    or “.smsk”

    Note:
    The file format is not automatically determined by the file name
    extension. You need to choose a file name with an extension
    which is consistent with the selected file format.
"""
mtest_mask_load!(s::SiglentScope, location) =
    scpi_write(s, _cmd(":MTESt:MASK:LOAD", location))

"""
    set_mtest_operate!(s, state)

This command sets the state of the mask test operation.

This command query returns the status of the mask test operation.

`:MTESt:OPERate <state>` (guide PDF p. 306)

    <state>:= {ON|OFF}
"""
set_mtest_operate!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":MTESt:OPERate", state))

"""
    get_mtest_operate(s)

This command sets the state of the mask test operation.

This command query returns the status of the mask test operation.

`:MTESt:OPERate?` (guide PDF p. 306)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_mtest_operate(s::SiglentScope) =
    _query_bool(s, ":MTESt:OPERate?")

"""
    mtest_reset!(s)

This command resets the mask test.

`:MTESt:RESet` (guide PDF p. 307)
"""
mtest_reset!(s::SiglentScope) =
    scpi_write(s, ":MTESt:RESet")

"""
    set_mtest_source!(s, source)

This command specifies the source of the mask test.

`:MTESt:SOURce <source>` (guide PDF p. 308)

    <source>:= {C<x>|Z<x>}
    - C denotes an analog input. C1 is analog input channel 1,
    for example.
    - Z denotes a zoomed input. Z1 denotes zoom 1.

    - <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    Note:
    Only Z<x> can be selected when Zoom is on.
"""
set_mtest_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":MTESt:SOURce", source))

"""
    get_mtest_source(s)

The query returns the current source of the mask test.

`:MTESt:SOURce?` (guide PDF p. 308)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|Z<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_mtest_source(s::SiglentScope) =
    _query_str(s, ":MTESt:SOURce?")

"""
    set_mtest_type!(s, type)

This command specifies the type of mask test.

`:MTESt:TYPE <type>` (guide PDF p. 309)

    <type>:= {ALL_IN|ALL_OUT|ANY_IN|ANY_OUT}
    - ALL_IN means that all of the waveform elements must fall
    within the mask area.
    - ALL_OUT means that all of the waveform elements are all
    outside of the mask area.
    - ANY_IN means that the waveform is partially within the
    mask area.
    - ANY_OUT means that the waveform is partially outside
    the mask area.
"""
set_mtest_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":MTESt:TYPE", type))

"""
    get_mtest_type(s)

The query returns the current type of mask test.

`:MTESt:TYPE` (guide PDF p. 309)

Returns `String`.

Response format:

    <type

    <type>:= {ALL_IN|ALL_OUT|ANY_IN|ANY_OUT}
"""
get_mtest_type(s::SiglentScope) =
    _query_str(s, ":MTESt:TYPE?")

# ---------------------------------------------------------------------- #
# RECall commands                                                        #
# ---------------------------------------------------------------------- #

export recall_fdefault!, recall_reference!, recall_serase!, recall_setup!

"""
    recall_fdefault!(s)

This command recalls the factory settings.

`:RECall:FDEFault` (guide PDF p. 311)
"""
recall_fdefault!(s::SiglentScope) =
    scpi_write(s, ":RECall:FDEFault")

"""
    recall_reference!(s, location, path)

This command recalls the specified waveform file from an external USB memory device and copies it to the selected reference waveform.

`:RECall:REFerence <location>,<path>` (guide PDF p. 312)

    <location>:= {REFA|REFB|REFC|REFD}
    - REF is the reference waveform name

    <path>:= Quoted string of path with an extension “.ref”
    Users can recall from local, net storage or U-disk according to
    requirements.
    Path type Such as
    local “local/SIGLENT/test.ref”
    net storage “net_storage/SIGLENT/test.ref”
    U-disk “U-disk0/SIGLENT/test.ref”
    “U-disk1/SIGLENT/test.ref”

    Note:
    The file format is not automatically determined by the file name
    extension. You need to choose a file name with an extension
    which is consistent with the selected file format.
"""
recall_reference!(s::SiglentScope, location, path) =
    scpi_write(s, _cmd(":RECall:REFerence", location, _quote(path)))

"""
    recall_serase!(s)

This command deletes user defined files stored inside the oscilloscope, includes reference waveforms, internal setups, internal mask files, custom default setups, the waveform files copied from analog trace to AWG.

`:RECall:SERase` (guide PDF p. 313)
"""
recall_serase!(s::SiglentScope) =
    scpi_write(s, ":RECall:SERase")

"""
    recall_setup!(s, state)

This command will recall the saved settings file from internal or external sources.

`:RECall:SETup <state>` (guide PDF p. 314)

    <state>:= {INTernal,<num>|EXTernal,<path>}

    <num>:= Value in NR1 format, including an integer and no
    decimal point, like 1.The range of the value is [1,10].

    <path>:= Quoted string of path with an extension “.xml”. Users
    can recall from local, net storage or U-disk according to
    requirements.
    Path type Such as
    local “local/SIGLENT/default.xml”
    net storage “net_storage/SIGLENT/default.xml”
    U-disk “U-disk0/SIGLENT/default.xml”
    “U-disk1/SIGLENT/default.xml”

    Note:
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
    • If the storage path type is not specified, it is recall from
    U-disk0 by default
"""
recall_setup!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":RECall:SETup", state))

# ---------------------------------------------------------------------- #
# REF commands                                                           #
# ---------------------------------------------------------------------- #

export set_ref_label!, get_ref_label, set_ref_label_text!, get_ref_label_text, ref_data!,
      get_ref_data_source, set_ref_data_scale!, get_ref_data_scale, set_ref_data_position!,
      get_ref_data_position

"""
    set_ref_label!(s, r, state)

The command is to turn the specified reference label on or off.

`:REF<r>:LABel <state>` (guide PDF p. 316)

    <r>:= {A|B|C|D}
    - Reference waveform name

    <state>:= {ON|OFF}
"""
set_ref_label!(s::SiglentScope, r, state) =
    scpi_write(s, _cmd(":REF$(r):LABel", state))

"""
    get_ref_label(s, r)

The query returns the state of the label associated with the specified reference.

`:REF<r>:LABel?` (guide PDF p. 316)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_ref_label(s::SiglentScope, r) =
    _query_bool(s, ":REF$(r):LABel?")

"""
    set_ref_label_text!(s, r, text)

The command sets the selected REF label to the string that follows. Setting a label for a REF also adds the name to the label list in non-volatile memory (replacing the oldest label in the list).

`:REF<r>:LABel:TEXT <string>` (guide PDF p. 317)

    <r>:= {A|B|C|D}
    - Reference waveform name

    <string>:= Quoted string of ASCII text. The length of the string
    is limited to 20 characters.
"""
set_ref_label_text!(s::SiglentScope, r, text) =
    scpi_write(s, _cmd(":REF$(r):LABel:TEXT", _quote(text)))

"""
    get_ref_label_text(s, r)

The query returns the current label text of the selected reference waveform.

`:REF<r>:LABel:TEXT?` (guide PDF p. 317)

Returns `String`.

Response format:

    <string>
"""
get_ref_label_text(s::SiglentScope, r) =
    _query_str(s, ":REF$(r):LABel:TEXT?")

"""
    ref_data!(s, r, operation)

The command controls the display and saving of reference waveforms.

`:REF<r>:DATA <operation>` (guide PDF p. 318)

    <r>:= {A|B|C|D}
    - Reference waveform name

    <operation>:= {LOAD|UNLoad|SAVE,<source>}
    - LOAD means to call up the reference waveform display.
    - UNLoad means to turn off the reference waveform display.
    - SAVE means to save the waveform to the reference
    waveform.

    <source>:= {C<x>|F<x>|D<n>}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
ref_data!(s::SiglentScope, r, operation) =
    scpi_write(s, _cmd(":REF$(r):DATA", operation))

"""
    get_ref_data_source(s, r)

This query returns the source of the current reference channel.

`:REF<r>:DATA:SOURce?` (guide PDF p. 319)

Returns `String`.

    <r>:= {A|B|C|D}

Response format:

    <source>

    <source>:= {C<x>|F<x>|D<n>}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math function
    1.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_ref_data_source(s::SiglentScope, r) =
    _query_str(s, ":REF$(r):DATA:SOURce?")

"""
    set_ref_data_scale!(s, r, value)

The command sets the vertical scale of the current reference channel. This command is only used when the current reference channel has been stored, and the display state is on.

`:REF<r>:DATA:SCALe <value>` (guide PDF p. 320)

    <r>:= {A|B|C|D}
    - Reference waveform name

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The scale range of the reference waveform is the same as that of
    the reference source.
"""
set_ref_data_scale!(s::SiglentScope, r, value) =
    scpi_write(s, _cmd(":REF$(r):DATA:SCALe", value))

"""
    get_ref_data_scale(s, r)

The query returns the vertical scale of the current reference channel.

`:REF<r>:DATA:SCALe?` (guide PDF p. 320)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_ref_data_scale(s::SiglentScope, r) =
    _query_float(s, ":REF$(r):DATA:SCALe?")

"""
    set_ref_data_position!(s, r, value)

The command sets the vertical offset of the current reference channel. This command is only used when the current reference channel has been saved, and the display state is on.

`:REF<r>:DATA:POSition <value>` (guide PDF p. 321)

    <r>:= {A|B|C|D}
    - Reference channel name

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The position range of the reference waveform is the same as that
    of the reference source.
"""
set_ref_data_position!(s::SiglentScope, r, value) =
    scpi_write(s, _cmd(":REF$(r):DATA:POSition", value))

"""
    get_ref_data_position(s, r)

This query returns the vertical offset of the current reference channel.

`:REF<r>:DATA:POSition?` (guide PDF p. 321)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_ref_data_position(s::SiglentScope, r) =
    _query_float(s, ":REF$(r):DATA:POSition?")

# ---------------------------------------------------------------------- #
# SAVE commands                                                          #
# ---------------------------------------------------------------------- #

export save_binary!, save_csv!, save_default!, save_image!, save_matlab!, save_reference!,
      save_setup!

"""
    save_binary!(s, path, src)

This command saves the binary data of the channel displayed on the screen to an external USB memory device.

`:SAVE:BINary <path>,<src>` (guide PDF p. 323)

    <path>:= Quoted string of path with an extension “.bin”
    Users can save to local, net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/test.bin”
    net storage “net_storage/test.bin”
    U-disk “U-disk0/test.bin”

    <src>:= {C<x>|F<x>|M<x>|D0_D15}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - M denotes a memory trace. For example, M1 is memory 1.
    - D0_D15 denotes a digital waveform. Data display by bit.

    Note:
    • When save to internal, the default path is local.
    • When save to external, if the path type is not set, it is
    stored to u-disk0 by default
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
    • If the parameter <src> is not specified, the command is
    invalid.
"""
save_binary!(s::SiglentScope, path, src) =
    scpi_write(s, _cmd(":SAVE:BINary", _quote(path), src))

"""
    save_csv!(s, path, source, state)

This command saves the waveform data of the specified channel to an external U disk/USB memory device in CSV format.

`:SAVE:CSV <path>,<source>,<state>` (guide PDF p. 324)

    <path>:= Quoted string of path with an extension “.csv”.
    Users can save to local, net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/test.csv”
    net storage “net_storage/test.csv”
    U-disk “U-disk0/test.csv”

    <source>:= {C<x>|F<x>|M<x>|D0_D15|DIGital}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - M denotes a memory trace. For example, M1 is memory 1.
    - D0_D15 denotes a digital waveform. Data display by bit.
    - DIGital denotes a digital waveform. Data display by bus.

    <state>:= {OFF|ON}
    - ON enables parameter save. This adds vertical scale
    values, horizontal timebase settings, and more instrument
    configuration information to the file.
    - OFF means to disables parameter save.

    Note:
    • When save to internal, the default path is local.
    • When save to external, if the path type is not set, it is
    stored to u-disk0 by default
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
"""
save_csv!(s::SiglentScope, path, source, state) =
    scpi_write(s, _cmd(":SAVE:CSV", _quote(path), source, state))

"""
    save_default!(s, set)

This command saves the current settings or factory settings as default settings.

`:SAVE:DEFault <set>` (guide PDF p. 325)

    <set>:= {CUSTom|FACTory}
    - CUSTom means the current settings.
    - FACTory means factory settings.
"""
save_default!(s::SiglentScope, set) =
    scpi_write(s, _cmd(":SAVE:DEFault", set))

"""
    save_image!(s, path, type, invert)

This command saves the screenshot to external storage.

`:SAVE:IMAGe <path>,<type>,<invert>` (guide PDF p. 326)

    <path>:= Quoted string of path with an extension “.bmp”
    or ”.jpg” or”.png”.
    Users can save to local, net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/test.bmp”
    net storage “net_storage/test.jpg”
    U-disk “U-disk0/test.png”

    <type>:= {BMP|JPG|PNG}

    <invert>:= {OFF|ON}}
    - ON will store images that have inverted colors. This means
    that a normally black background will be white when
    inverted. This setting is recommended if you plan on
    printing the image as an inverted image with a white
    background will save on ink.
    - OFF will store images that are identical to the display of
    the instrument.

    Note:
    • When save to internal, the default path is local.
    • When save to external, if the path type is not set, it is
    stored to u-disk0 by default
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
"""
save_image!(s::SiglentScope, path, type, invert) =
    scpi_write(s, _cmd(":SAVE:IMAGe", _quote(path), type, invert))

"""
    save_matlab!(s, path, source)

This command saves the waveform data of the specified channel to an external USB memory device in Matlab format.

`:SAVE:MATLab <path>,<source>` (guide PDF p. 327)

    <path>:= Quoted string of path with an extension “.mat”.
    Users can save to local, net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/test.bin”
    net storage “net_storage/test.bin”
    U-disk “U-disk0/test.bin”

    <source>:= {C<x>|F<x>|M<x>|D0_D15|DIGital}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - M denotes a memory trace. For example, M1 is memory 1.
    - D0_D15 denotes a digital waveform. Data display by bit.
    - DIGital denotes a digital waveform. Data display by bus.

    Note:
    • When save to internal, the default path is local.
    • When save to external, if the path type is not set, it is
    stored to u-disk0 by default
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
"""
save_matlab!(s::SiglentScope, path, source) =
    scpi_write(s, _cmd(":SAVE:MATLab", _quote(path), source))

"""
    save_reference!(s, path, source)

This command saves the selected channel waveform to external memory as reference.

`:SAVE:REFerence <path>,<source>` (guide PDF p. 328)

    <path>:= Quoted string of path with an extension “.ref”.
    Users can save to local, net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/test.ref”
    net storage “net_storage/test.ref”
    U-disk “U-disk0/test.ref”

    <source>:= {C<x>|F<x>|D<n>}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math
    function 1.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.

    Note:
    The file format is not automatically determined by the file name
    extension. You need to choose a file name with an extension
    which is consistent with the selected file format.
"""
save_reference!(s::SiglentScope, path, source) =
    scpi_write(s, _cmd(":SAVE:REFerence", _quote(path), source))

"""
    save_setup!(s, setup_num)

This command saves the current settings to internal or external memory locations.

`:SAVE:SETup <setup_num>` (guide PDF p. 329)

    <setup_num>:= {INTernal,<num>|EXTernal,<path>}

    <num>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 10].

    <path>:= Quoted string of path with an extension “.xml”. Users
    can recall from local,net storage or U-disk according to
    requirements
    Path type Such as
    local “local/SIGLENT/default.xml”
    net storage “net_storage/SIGLENT/default.xml”
    U-disk “U-disk0/SIGLENT/default.xml”

    Note:
    • When save to internal, the default path is local. And the
    setup file will be stored in local with the name
    "SDS000x.xml"
    • When save to external, if the path type is not set, it is
    stored to u-disk0 by default
    • The file format is not automatically determined by the file
    name extension. You need to choose a file name with an
    extension which is consistent with the selected file format.
"""
save_setup!(s::SiglentScope, setup_num) =
    scpi_write(s, _cmd(":SAVE:SETup", setup_num))

# ---------------------------------------------------------------------- #
# SEARch commands                                                        #
# ---------------------------------------------------------------------- #

export set_search!, get_search, set_search_mode!, get_search_mode, get_search_count,
      get_search_event, search_copy!, set_search_edge_source!, get_search_edge_source,
      set_search_edge_slope!, get_search_edge_slope, set_search_edge_level!,
      get_search_edge_level, set_search_slope_source!, get_search_slope_source,
      set_search_slope_slope!, get_search_slope_slope, set_search_slope_hlevel!,
      get_search_slope_hlevel, set_search_slope_llevel!, get_search_slope_llevel,
      set_search_slope_limit!, get_search_slope_limit, set_search_slope_tupper!,
      get_search_slope_tupper, set_search_slope_tlower!, get_search_slope_tlower,
      set_search_pulse_source!, get_search_pulse_source, set_search_pulse_polarity!,
      get_search_pulse_polarity, set_search_pulse_level!, get_search_pulse_level,
      set_search_pulse_limit!, get_search_pulse_limit, set_search_pulse_tupper!,
      get_search_pulse_tupper, set_search_pulse_tlower!, get_search_pulse_tlower,
      set_search_interval_source!, get_search_interval_source, set_search_interval_slope!,
      get_search_interval_slope, set_search_interval_level!, get_search_interval_level,
      set_search_interval_limit!, get_search_interval_limit, set_search_interval_tupper!,
      get_search_interval_tupper, set_search_interval_tlower!, get_search_interval_tlower,
      set_search_runt_source!, get_search_runt_source, set_search_runt_polarity!,
      get_search_runt_polarity, set_search_runt_hlevel!, get_search_runt_hlevel,
      set_search_runt_llevel!, get_search_runt_llevel, set_search_runt_limit!,
      get_search_runt_limit, set_search_runt_tupper!, get_search_runt_tupper,
      set_search_runt_tlower!, get_search_runt_tlower

"""
    set_search!(s, state)

The command sets the switch of the search function.

`:SEARch <state>` (guide PDF p. 331)

    <state>:= {ON|OFF}
"""
set_search!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SEARch", state))

"""
    get_search(s)

This query returns the current status of the search function.

`:SEARch?` (guide PDF p. 331)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_search(s::SiglentScope) =
    _query_bool(s, ":SEARch?")

"""
    set_search_mode!(s, mode)

The command sets the mode of search.

`:SEARch:MODE <mode>` (guide PDF p. 332)

    <mode>:= {EDGE|SLOPe|PULSE|INTerval|RUNT}
"""
set_search_mode!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":SEARch:MODE", mode))

"""
    get_search_mode(s)

The query returns the current mode of search.

`:SEARch:MODE?` (guide PDF p. 332)

Returns `String`.

Response format:

    <mode>

    <mode>:= {EDGE|SLOPe|PULSE|INTerval|RUNT}
"""
get_search_mode(s::SiglentScope) =
    _query_str(s, ":SEARch:MODE?")

"""
    get_search_count(s)

The query returns the total number of search events in the current screen.

`:SEARch:COUNt?` (guide PDF p. 333)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_search_count(s::SiglentScope) =
    _query_int(s, ":SEARch:COUNt?")

"""
    get_search_event(s)

The query returns the index of the search event in the center of the screen when the oscilloscope acquisition is stopped.

`:SEARch:EVENt?` (guide PDF p. 334)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_search_event(s::SiglentScope) =
    _query_int(s, ":SEARch:EVENt?")

"""
    search_copy!(s, operation)

The command synchronizes the search settings with the trigger settings.

`:SEARch:COPY <operation>` (guide PDF p. 335)

    <operation>:= {FROMtrigger|TOTRigger|CANCel}
    - FROMtrigger means copy trigger settings to search.
    - TOTRigger means copy search settings to trigger.
    - CANCel can undo the above two copying operations.
"""
search_copy!(s::SiglentScope, operation) =
    scpi_write(s, _cmd(":SEARch:COPY", operation))

"""
    set_search_edge_source!(s, source)

The command sets the search source of the edge search.

`:SEARch:EDGE:SOURce <source>` (guide PDF p. 337)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_search_edge_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SEARch:EDGE:SOURce", source))

"""
    get_search_edge_source(s)

The query returns the current search source of the edge search.

`:SEARch:EDGE:SOURce?` (guide PDF p. 337)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_search_edge_source(s::SiglentScope) =
    _query_str(s, ":SEARch:EDGE:SOURce?")

"""
    set_search_edge_slope!(s, slope_type)

The command sets the slope of the edge search.

`:SEARch:EDGE:SLOPe <slope_type>` (guide PDF p. 338)

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
set_search_edge_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":SEARch:EDGE:SLOPe", slope_type))

"""
    get_search_edge_slope(s)

The query returns the current slope setting of the edge search.

`:SEARch:EDGE:SLOPe?` (guide PDF p. 338)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
get_search_edge_slope(s::SiglentScope) =
    _query_str(s, ":SEARch:EDGE:SLOPe?")

"""
    set_search_edge_level!(s, level_value)

The command sets the search level of the edge search.

`:SEARch:EDGE:LEVel <level_value>` (guide PDF p. 339)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_search_edge_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":SEARch:EDGE:LEVel", level_value))

"""
    get_search_edge_level(s)

The query returns the current search level value of the edge search.

`:SEARch:EDGE:LEVel?` (guide PDF p. 339)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_search_edge_level(s::SiglentScope) =
    _query_float(s, ":SEARch:EDGE:LEVel?")

"""
    set_search_slope_source!(s, source)

The command sets the search source of the slope search.

`:SEARch:SLOPe:SOURce <source>` (guide PDF p. 341)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_search_slope_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SEARch:SLOPe:SOURce", source))

"""
    get_search_slope_source(s)

The query returns the current search source of the slope search.

`:SEARch:SLOPe:SOURce?` (guide PDF p. 341)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_search_slope_source(s::SiglentScope) =
    _query_str(s, ":SEARch:SLOPe:SOURce?")

"""
    set_search_slope_slope!(s, slope_type)

The command sets the slope of the slope search.

`:SEARch:SLOPe:SLOPe <slope_type>` (guide PDF p. 342)

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
set_search_slope_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":SEARch:SLOPe:SLOPe", slope_type))

"""
    get_search_slope_slope(s)

The query returns the current slope of the slope search.

`:SEARch:SLOPe:SLOPe?` (guide PDF p. 342)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
get_search_slope_slope(s::SiglentScope) =
    _query_str(s, ":SEARch:SLOPe:SLOPe?")

"""
    set_search_slope_hlevel!(s, high_level_value)

The command sets the high level of the slope search.

`:SEARch:SLOPe:HLEVel <high_level_value>` (guide PDF p. 343)

    <high_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The high level value cannot be less than the low level value
    using by the command :SEARch:SLOPe:LLEVel.
"""
set_search_slope_hlevel!(s::SiglentScope, high_level_value) =
    scpi_write(s, _cmd(":SEARch:SLOPe:HLEVel", high_level_value))

"""
    get_search_slope_hlevel(s)

The query returns the current high level of the slope search.

`:SEARch:SLOPe:HLEVel?` (guide PDF p. 343)

Returns `Float64`.

Response format:

    <high_level_value>

    <high_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.
"""
get_search_slope_hlevel(s::SiglentScope) =
    _query_float(s, ":SEARch:SLOPe:HLEVel?")

"""
    set_search_slope_llevel!(s, low_level_value)

The command sets the low level of the slope search.

`:SEARch:SLOPe:LLEVel <low_level_value>` (guide PDF p. 344)

    <low_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The low level value cannot be greater than the low level value
    using by the command :SEARch:SLOPe:HLEVel.
"""
set_search_slope_llevel!(s::SiglentScope, low_level_value) =
    scpi_write(s, _cmd(":SEARch:SLOPe:LLEVel", low_level_value))

"""
    get_search_slope_llevel(s)

The query returns the current low level of the slope search.

`:SEARch:SLOPe:LLEVel?` (guide PDF p. 344)

Returns `Float64`.

Response format:

    <low_level_value>

    <low_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.
"""
get_search_slope_llevel(s::SiglentScope) =
    _query_float(s, ":SEARch:SLOPe:LLEVel?")

"""
    set_search_slope_limit!(s, type)

The command sets the limit range type of the slope search.

`:SEARch:SLOPe:LIMit <type>` (guide PDF p. 345)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_search_slope_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":SEARch:SLOPe:LIMit", type))

"""
    get_search_slope_limit(s)

The query returns the current limit range type of the slope search.

`:SEARch:SLOPe:LIMit?` (guide PDF p. 345)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_search_slope_limit(s::SiglentScope) =
    _query_str(s, ":SEARch:SLOPe:LIMit?")

"""
    set_search_slope_tupper!(s, value)

The command sets the upper value of the slope search limit type.

`:SEARch:SLOPe:TUPPer <value>` (guide PDF p. 346)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :SEARch:SLOPe:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_search_slope_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:SLOPe:TUPPer", value))

"""
    get_search_slope_tupper(s)

The query returns the current upper value of the slope search limit type.

`:SEARch:SLOPe:TUPPer?` (guide PDF p. 346)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_search_slope_tupper(s::SiglentScope) =
    _query_float(s, ":SEARch:SLOPe:TUPPer?")

"""
    set_search_slope_tlower!(s, value)

The command sets the lower value of the slope search limit type.

`:SEARch:SLOPe:TLOWer <value>` (guide PDF p. 347)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :SEARch:SLOPe:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_search_slope_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:SLOPe:TLOWer", value))

"""
    get_search_slope_tlower(s)

The query returns the current lower value of the slope search limit type.

`:SEARch:SLOPe:TLOWer?` (guide PDF p. 347)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_search_slope_tlower(s::SiglentScope) =
    _query_float(s, ":SEARch:SLOPe:TLOWer?")

"""
    set_search_pulse_source!(s, source)

The command sets the search source of the pulse search.

`:SEARch:PULSe:SOURce <source>` (guide PDF p. 349)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_search_pulse_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SEARch:PULSe:SOURce", source))

"""
    get_search_pulse_source(s)

The query returns the current search source of the pulse search.

`:SEARch:PULSe:SOURce?` (guide PDF p. 349)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_search_pulse_source(s::SiglentScope) =
    _query_str(s, ":SEARch:PULSe:SOURce?")

"""
    set_search_pulse_polarity!(s, polarity_type)

The command sets the polarity of the pulse search.

`:SEARch:PULSe:POLarity <polarity_type>` (guide PDF p. 350)

    <polarity_type>:= {POSitive|NEGative}
"""
set_search_pulse_polarity!(s::SiglentScope, polarity_type) =
    scpi_write(s, _cmd(":SEARch:PULSe:POLarity", polarity_type))

"""
    get_search_pulse_polarity(s)

The query returns the current polarity of the pulse search.

`:SEARch:PULSe:POLarity?` (guide PDF p. 350)

Returns `String`.

Response format:

    <polarity_type>

    <polarity_type>:= {POSitive|NEGative}
"""
get_search_pulse_polarity(s::SiglentScope) =
    _query_str(s, ":SEARch:PULSe:POLarity?")

"""
    set_search_pulse_level!(s, level_value)

The command sets the search level of the pulse search.

`:SEARch:PULSe:LEVel <level_value>` (guide PDF p. 351)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_search_pulse_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":SEARch:PULSe:LEVel", level_value))

"""
    get_search_pulse_level(s)

The query returns the current search level of the pulse search.

`:SEARch:PULSe:LEVel?` (guide PDF p. 351)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_search_pulse_level(s::SiglentScope) =
    _query_float(s, ":SEARch:PULSe:LEVel?")

"""
    set_search_pulse_limit!(s, type)

The command sets the limit range type of the pulse search.

`:SEARch:PULSe:LIMit <type>` (guide PDF p. 352)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_search_pulse_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":SEARch:PULSe:LIMit", type))

"""
    get_search_pulse_limit(s)

The query returns the current limit range type of the pulse search.

`:SEARch:PULSe:LIMit?` (guide PDF p. 352)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_search_pulse_limit(s::SiglentScope) =
    _query_str(s, ":SEARch:PULSe:LIMit?")

"""
    set_search_pulse_tupper!(s, value)

The command sets the upper value of the pulse search limit type.

`:SEARch:PULse:TUPPer <value>` (guide PDF p. 353)

    <value>:= Value in NR3 format.The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :SEARch:PULse:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_search_pulse_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:PULSe:TUPPer", value))

"""
    get_search_pulse_tupper(s)

The query returns the current upper value of the pulse search limit type.

`:SEARch:PULSe:TUPPer?` (guide PDF p. 353)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_search_pulse_tupper(s::SiglentScope) =
    _query_float(s, ":SEARch:PULSe:TUPPer?")

"""
    set_search_pulse_tlower!(s, value)

The command sets the lower value of the pulse search limit type.

`:SEARch:PULSe:TLOWer <value>` (guide PDF p. 354)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :SEARch:PULSe:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_search_pulse_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:PULSe:TLOWer", value))

"""
    get_search_pulse_tlower(s)

The query returns the current lower value of the pulse search limit type.

`:SEARch:PULSe:TLOWer?` (guide PDF p. 354)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_search_pulse_tlower(s::SiglentScope) =
    _query_float(s, ":SEARch:PULSe:TLOWer?")

"""
    set_search_interval_source!(s, source)

The command sets the search source of the interval search.

`:SEARch:INTerval:SOURce <source>` (guide PDF p. 356)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_search_interval_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SEARch:INTerval:SOURce", source))

"""
    get_search_interval_source(s)

The query returns the current search source of the interval search.

`:SEARch:INTerval:SOURce?` (guide PDF p. 356)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_search_interval_source(s::SiglentScope) =
    _query_str(s, ":SEARch:INTerval:SOURce?")

"""
    set_search_interval_slope!(s, slope_type)

The command sets the slope of the interval search.

`:SEARch:INTerval:SLOPe <slope_type>` (guide PDF p. 357)

    <slope_type>:= {RISing|FALLing}
"""
set_search_interval_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":SEARch:INTerval:SLOPe", slope_type))

"""
    get_search_interval_slope(s)

The query returns the current slope of the interval search.

`:SEARch:INTerval:SLOPe?` (guide PDF p. 357)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_search_interval_slope(s::SiglentScope) =
    _query_str(s, ":SEARch:INTerval:SLOPe?")

"""
    set_search_interval_level!(s, level_value)

The command sets the search level of the interval search.

`:SEARch:INTerval:LEVel <level_value>` (guide PDF p. 358)

    <level_value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_search_interval_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":SEARch:INTerval:LEVel", level_value))

"""
    get_search_interval_level(s)

The query returns the current search level of the interval search.

`:SEARch:INTerval:LEVel?` (guide PDF p. 358)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format
"""
get_search_interval_level(s::SiglentScope) =
    _query_float(s, ":SEARch:INTerval:LEVel?")

"""
    set_search_interval_limit!(s, type)

The command sets the limit range type of the interval search.

`:SEARch:INTerval:LIMit <type>` (guide PDF p. 359)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_search_interval_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":SEARch:INTerval:LIMit", type))

"""
    get_search_interval_limit(s)

The query returns the current limit range type of the interval search.

`:SEARch:INTerval:LIMit?` (guide PDF p. 359)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_search_interval_limit(s::SiglentScope) =
    _query_str(s, ":SEARch:INTerval:LIMit?")

"""
    set_search_interval_tupper!(s, value)

The command sets the upper value of the interval search limit type.

`:SEARch:INTerval:TUPPer <value>` (guide PDF p. 360)

    <value>:= Value in NR3 format. The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :SEARch:INTerval:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_search_interval_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:INTerval:TUPPer", value))

"""
    get_search_interval_tupper(s)

The query returns the current upper value of the interval search limit type.

`:SEARch:INTerval:TUPPer?` (guide PDF p. 360)

Returns `Float64`.

Response format:

    <tupper_value>

    <tupper_value>:= Value in NR3 format.
"""
get_search_interval_tupper(s::SiglentScope) =
    _query_float(s, ":SEARch:INTerval:TUPPer?")

"""
    set_search_interval_tlower!(s, value)

The command sets the lower value of the interval search limit type.

`:SEARch:INTerval:TLOWer <value>` (guide PDF p. 361)

    <value>:= Value in NR3 format. The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :SEARch:INTerval:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_search_interval_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:INTerval:TLOWer", value))

"""
    get_search_interval_tlower(s)

The query returns the current lower value of the interval search limit type.

`:SEARch:INTerval:TLOWer?` (guide PDF p. 361)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_search_interval_tlower(s::SiglentScope) =
    _query_float(s, ":SEARch:INTerval:TLOWer?")

"""
    set_search_runt_source!(s, source)

The command sets the search source of the runt search.

`:SEARch:RUNT:SOURce <source>` (guide PDF p. 363)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_search_runt_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SEARch:RUNT:SOURce", source))

"""
    get_search_runt_source(s)

The query returns the current search source of the runt search.

`:SEARch:RUNT:SOURce?` (guide PDF p. 363)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_search_runt_source(s::SiglentScope) =
    _query_str(s, ":SEARch:RUNT:SOURce?")

"""
    set_search_runt_polarity!(s, polarity_type)

The command sets the polarity of the runt search.

`:SEARch:RUNT:POLarity <polarity_type>` (guide PDF p. 364)

    <polarity_type>:= {POSitive|NEGative}
"""
set_search_runt_polarity!(s::SiglentScope, polarity_type) =
    scpi_write(s, _cmd(":SEARch:RUNT:POLarity", polarity_type))

"""
    get_search_runt_polarity(s)

The query returns the current polarity of the runt search.

`:SEARch:RUNT:POLarity?` (guide PDF p. 364)

Returns `String`.

Response format:

    <polarity_type>

    <polarity_type>:= {POSitive|NEGative}
"""
get_search_runt_polarity(s::SiglentScope) =
    _query_str(s, ":SEARch:RUNT:POLarity?")

"""
    set_search_runt_hlevel!(s, value)

The command sets the high search level of the runt search.

`:SEARch:RUNT:HLEVel <value>` (guide PDF p. 365)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The high level value cannot be less than the low level value
    using by the command :SEARch:RUNT:LLEVel.
"""
set_search_runt_hlevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:RUNT:HLEVel", value))

"""
    get_search_runt_hlevel(s)

The query returns the current high search level of the runt search.

`:SEARch:RUNT:HLEVel?` (guide PDF p. 365)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_search_runt_hlevel(s::SiglentScope) =
    _query_float(s, ":SEARch:RUNT:HLEVel?")

"""
    set_search_runt_llevel!(s, value)

The command sets the low search level of the runt search.

`:SEARch:RUNT:LLEVel <value>` (guide PDF p. 366)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]

    Note:
    The low level value cannot be greater than the high level value
    using by the command :SEARch:RUNT:HLEVel.
"""
set_search_runt_llevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:RUNT:LLEVel", value))

"""
    get_search_runt_llevel(s)

The query returns the current low search level of the runt search.

`:SEARch:RUNT:LLEVel?` (guide PDF p. 366)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_search_runt_llevel(s::SiglentScope) =
    _query_float(s, ":SEARch:RUNT:LLEVel?")

"""
    set_search_runt_limit!(s, type)

The command sets the limit range type of the runt search.

`:SEARch:RUNT:LIMit <type>` (guide PDF p. 367)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_search_runt_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":SEARch:RUNT:LIMit", type))

"""
    get_search_runt_limit(s)

The query returns the current limit range type of the runt search.

`:SEARch:RUNT:LIMit?` (guide PDF p. 367)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_search_runt_limit(s::SiglentScope) =
    _query_str(s, ":SEARch:RUNT:LIMit?")

"""
    set_search_runt_tupper!(s, value)

The command sets the upper value of the runt search limit type.

`:SEARch:PULse:RUNT <value>` (guide PDF p. 368)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :SEARch:RUNT:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_search_runt_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:RUNT:TUPPer", value))

"""
    get_search_runt_tupper(s)

The query returns the current upper value of the runt search limit type.

`:SEARch:RUNT:TUPPer?` (guide PDF p. 368)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_search_runt_tupper(s::SiglentScope) =
    _query_float(s, ":SEARch:RUNT:TUPPer?")

"""
    set_search_runt_tlower!(s, value)

The command sets the lower value of the runt search limit type.

`:SEARch:RUNT:TLOWer <value>` (guide PDF p. 369)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :SEARch:RUNT:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_search_runt_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SEARch:RUNT:TLOWer", value))

"""
    get_search_runt_tlower(s)

The query returns the current lower value of the runt search limit type.

`:SEARch:RUNT:TLOWer?` (guide PDF p. 369)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_search_runt_tlower(s::SiglentScope) =
    _query_float(s, ":SEARch:RUNT:TLOWer?")

# ---------------------------------------------------------------------- #
# SYSTem commands                                                        #
# ---------------------------------------------------------------------- #

export set_system_buzzer!, get_system_buzzer, set_system_clock!, get_system_clock,
      set_system_communicate_lan_gateway!, get_system_communicate_lan_gateway,
      set_system_communicate_lan_ipaddress!, get_system_communicate_lan_ipaddress,
      get_system_communicate_lan_mac, set_system_communicate_lan_smask!,
      get_system_communicate_lan_smask, set_system_communicate_lan_type!,
      get_system_communicate_lan_type, set_system_communicate_vncport!,
      get_system_communicate_vncport, set_system_date!, get_system_date, set_system_edumode!,
      get_system_edumode, set_system_language!, get_system_language, set_system_menu!,
      get_system_menu, set_system_nstorage!, get_system_nstorage, system_nstorage_connect!,
      system_nstorage_disconnect!, get_system_nstorage_status, set_system_pon!,
      get_system_pon, system_reboot!, set_system_remote!, get_system_remote, system_selfcal!,
      get_system_selfcal, system_shutdown!, set_system_ssaver!, get_system_ssaver,
      set_system_time!, get_system_time, set_system_touch!, get_system_touch

"""
    set_system_buzzer!(s, state)

The command the status of the buzzer.

`:SYSTem:BUZZer <state>` (guide PDF p. 371)

    <state>:= {ON|OFF}
"""
set_system_buzzer!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:BUZZer", state))

"""
    get_system_buzzer(s)

The query returns the current status of the buzzer.

`:SYSTem:BUZZer?` (guide PDF p. 371)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_system_buzzer(s::SiglentScope) =
    _query_bool(s, ":SYSTem:BUZZer?")

"""
    set_system_clock!(s, source)

The command sets the oscilloscope clock source and the state of the 10 MHz clock output.

`:SYSTem:CLOCk <source>` (guide PDF p. 372)

    <source>:= {EXT|IN_ON|IN_OFF}
    - EXT selects the external clock source. The 10 MHz output
    will be automatically disabled.
    - IN_ON selects the internal clock source and enables the
    10 MHz output.
    - IN_OFF selects the internal clock source and disables the
    10M output.
"""
set_system_clock!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":SYSTem:CLOCk", source))

"""
    get_system_clock(s)

The query returns the oscilloscope current clock source and the state of the 10 MHz clock output.

`:SYSTem:CLOCk?` (guide PDF p. 372)

Returns `String`.

Response format:

    <source>

    <source>:= {EXT|IN_ON|IN_OFF}
"""
get_system_clock(s::SiglentScope) =
    _query_str(s, ":SYSTem:CLOCk?")

"""
    set_system_communicate_lan_gateway!(s, text)

The command is used to set the gateway of the internal network of the oscilloscope.

`:SYSTem:COMMunicate:LAN:GATeway <string>` (guide PDF p. 373)

    <string>:=quoted string of ASCII text.
"""
set_system_communicate_lan_gateway!(s::SiglentScope, text) =
    scpi_write(s, _cmd(":SYSTem:COMMunicate:LAN:GATeway", _quote(text)))

"""
    get_system_communicate_lan_gateway(s)

The query returns the gateway of the network.

`:SYSTem:COMMunicate:LAN:GATeway?` (guide PDF p. 373)

Returns `String`.

Response format:

    <string>
"""
get_system_communicate_lan_gateway(s::SiglentScope) =
    _query_str(s, ":SYSTem:COMMunicate:LAN:GATeway?")

"""
    set_system_communicate_lan_ipaddress!(s, text)

The command sets the IP address of the oscilloscope’s internal network interface.

`:SYSTem:COMMunicate:LAN:IPADdress <string>` (guide PDF p. 374)

    <string>:=quoted string of ASCII text.
"""
set_system_communicate_lan_ipaddress!(s::SiglentScope, text) =
    scpi_write(s, _cmd(":SYSTem:COMMunicate:LAN:IPADdress", _quote(text)))

"""
    get_system_communicate_lan_ipaddress(s)

The query returns the IP address of the oscilloscope’s internal network interface.

`:SYSTem:COMMunicate:LAN:IPADdress?` (guide PDF p. 374)

Returns `String`.

Response format:

    <string>
"""
get_system_communicate_lan_ipaddress(s::SiglentScope) =
    _query_str(s, ":SYSTem:COMMunicate:LAN:IPADdress?")

"""
    get_system_communicate_lan_mac(s)

The query returns the MAC address of the oscilloscope.

`:SYSTem:COMMunicate:LAN:MAC?` (guide PDF p. 375)

Returns `String`.

Response format:

    <byte1>:<byte2>:<byte3>:<byte4>:<byte5>:<byte6>
"""
get_system_communicate_lan_mac(s::SiglentScope) =
    _query_str(s, ":SYSTem:COMMunicate:LAN:MAC?")

"""
    set_system_communicate_lan_smask!(s, text)

The command sets the subnet mask of the oscilloscope’s internal network interface.

`:SYSTem:COMMunicate:LAN:SMASK <string>` (guide PDF p. 376)

    <string>:=quoted string of ASCII text.
"""
set_system_communicate_lan_smask!(s::SiglentScope, text) =
    scpi_write(s, _cmd(":SYSTem:COMMunicate:LAN:SMASk", _quote(text)))

"""
    get_system_communicate_lan_smask(s)

The query returns the subnet mask of the oscilloscope’s internal network interface.

`:SYSTem:COMMunicate:LAN:SMASK?` (guide PDF p. 376)

Returns `String`.

Response format:

    <string>
"""
get_system_communicate_lan_smask(s::SiglentScope) =
    _query_str(s, ":SYSTem:COMMunicate:LAN:SMASk?")

"""
    set_system_communicate_lan_type!(s, state)

The command sets the type of LAN configuration settings.

`:SYSTem:COMMunicate:LAN:TYPE <state>` (guide PDF p. 377)

    <state>:= {STATIC|DHCP}
    - STATIC means that the Ethernet settings will be configured
    manually, using
    commands :SYSTem:COMMunicate:LAN:IPADdress, :SY
    STem:COMMunicate:LAN:SMASK,
    and :SYSTem:COMMunicate:LAN:GATeway
    - DHCP means that the oscilloscope’s IP address, subnet
    mask and gateway settings will be received from a DHCP
    server on the local network.
"""
set_system_communicate_lan_type!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:COMMunicate:LAN:TYPE", state))

"""
    get_system_communicate_lan_type(s)

The query returns the current type of the LAN configuration settings.

`:SYSTem:COMMunicate:LAN:TYPE?` (guide PDF p. 377)

Returns `String`.

Response format:

    <state>

    <state>:= {STATIC|DHCP}
"""
get_system_communicate_lan_type(s::SiglentScope) =
    _query_str(s, ":SYSTem:COMMunicate:LAN:TYPE?")

"""
    set_system_communicate_vncport!(s, value)

The command sets the VNC port of the oscilloscope.

`:SYSTem:COMMunicate:VNCPort <value>` (guide PDF p. 378)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [5900, 5999].
"""
set_system_communicate_vncport!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":SYSTem:COMMunicate:VNCPort", value))

"""
    get_system_communicate_vncport(s)

The query returns the current VNC port of the oscilloscope.

`:SYSTem:COMMunicate:VNCPort?` (guide PDF p. 378)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_system_communicate_vncport(s::SiglentScope) =
    _query_int(s, ":SYSTem:COMMunicate:VNCPort?")

"""
    set_system_date!(s, date)

The command sets the system date of the oscilloscope.

`:SYSTem:DATE <date>` (guide PDF p. 379)

    <date>:= 8-digit NR1 format, from high to low, is expressed as
    a 4-digit year, 2-digit month, and 2-digit day.
"""
set_system_date!(s::SiglentScope, date) =
    scpi_write(s, _cmd(":SYSTem:DATE", date))

"""
    get_system_date(s)

This query returns the oscilloscope current date.

`:SYSTem:DATE?` (guide PDF p. 379)

Returns `String`.

Response format:

    <date>
"""
get_system_date(s::SiglentScope) =
    _query_str(s, ":SYSTem:DATE?")

"""
    set_system_edumode!(s, func, lock)

The command sets the education mode (locks of AutoSetup, measure and cursors) of the oscilloscope.

`:SYSTem:EDUMode <func>,<lock>` (guide PDF p. 380)

    <func>:= {AUTOSet|MEASure|CURSor}

    <lock>:= {ON|OFF}
    - ON means the enable the function.
    - OFF means disable the function.
"""
set_system_edumode!(s::SiglentScope, func, lock) =
    scpi_write(s, _cmd(":SYSTem:EDUMode", func, lock))

"""
    get_system_edumode(s, func=nothing)

The query returns the education mode of the oscilloscope.

`:SYSTem:EDUMode? [<func>]` (guide PDF p. 380)

Returns `String`.

Response format:

    Format 1:
    AUTOSet,<lock>;MEASure,<lock>;CURSor,<lock>

    Format 2:
    <lock>
    <lock>:= {ON|OFF}
"""
get_system_edumode(s::SiglentScope, func=nothing) =
    _query_str(s, _cmd(":SYSTem:EDUMode?", func))

"""
    set_system_language!(s, language)

The command selects the oscilloscope language display.

`:SYSTem:LANGuage <language>` (guide PDF p. 381)

    <language>:=
    {SCHinese|TCHinese|ENGLish|FRENch|JAPanese|KORean|D
    EUTsch|ESPan|RUSSian|ITALiana|PORTuguese}
"""
set_system_language!(s::SiglentScope, language) =
    scpi_write(s, _cmd(":SYSTem:LANGuage", language))

"""
    get_system_language(s)

This query returns the oscilloscope language display.

`:SYSTem:LANGuage?` (guide PDF p. 381)

Returns `String`.

Response format:

    <language>

    <language>:=
    {SCHinese|TCHinese|ENGLish|FRENch|JAPanese|KORean|D
    EUTsch|ESPan|RUSSian|ITALiana|PORTuguese}
"""
get_system_language(s::SiglentScope) =
    _query_str(s, ":SYSTem:LANGuage?")

"""
    set_system_menu!(s, state)

The command sets the state of the menu.

Note: This command is only valid for models with the menu switch.

`:SYSTem:MENU <state>` (guide PDF p. 382)

    <state>:= {ON|OFF}
"""
set_system_menu!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:MENU", state))

"""
    get_system_menu(s)

The query returns the current state of the menu.

`:SYSTem:MENU?` (guide PDF p. 382)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_system_menu(s::SiglentScope) =
    _query_bool(s, ":SYSTem:MENU?")

"""
    set_system_nstorage!(s, path, user, pwd, anon, auto_con, rem_path, rem_user, rem_pwd)

This command attempts to mount the network drive specified by the parameters.

`:SYSTem:NSTorage <path>,<user>,<pwd>,<anon>,<auto_con>,<rem_path>,<rem_user>,<rem_pwd>` (guide PDF p. 383)

    <path>:= Quoted string of the server path to be mounted
    <user>:= Quoted string of the user name.
    <pwd>:= Quoted string of the user password
    <anon>:= Anonymous flag, 1 for ON while 0 for OFF
    <auto_con>:= Automatic connection flag, 1 for ON while 0 for
    OFF
    <rem_path>:= Remember path flag, 1 for ON while 0 for OFF
    <rem_user>:= Remember user flag, 1 for ON while 0 for OFF
    <rem pwd>:= Remember password flag, 1 for ON while 0 for
    OFF
"""
set_system_nstorage!(s::SiglentScope, path, user, pwd, anon, auto_con, rem_path, rem_user, rem_pwd) =
    scpi_write(s, _cmd(":SYSTem:NSTorage", _quote(path), _quote(user), _quote(pwd), anon, auto_con, rem_path, rem_user, rem_pwd))

"""
    get_system_nstorage(s)

This query returns the parameters of the mounted network drive.

`:SYSTem:NSTorage?` (guide PDF p. 383)

Returns `String`.

Response format:

    <path>,<user>,<pwd>,<anon>,<auto_con>,<rem_path>,<rem_
    user>,<rem_pwd>

    Note:
    For security, the password is always returned “***”.
"""
get_system_nstorage(s::SiglentScope) =
    _query_str(s, ":SYSTem:NSTorage?")

"""
    system_nstorage_connect!(s)

This command attempts to mount the network drive.

`:SYSTem:NSTorage:CONNect` (guide PDF p. 384)
"""
system_nstorage_connect!(s::SiglentScope) =
    scpi_write(s, ":SYSTem:NSTorage:CONNect")

"""
    system_nstorage_disconnect!(s)

This command attempts to un-mount the network drive.

`:SYSTem:NSTorage:DISConnect` (guide PDF p. 384)
"""
system_nstorage_disconnect!(s::SiglentScope) =
    scpi_write(s, ":SYSTem:NSTorage:DISConnect")

"""
    get_system_nstorage_status(s)

The query returns the mount status of network drive.

`:SYSTem:NSTorage:STATus?` (guide PDF p. 384)

Returns `Bool`.

Response format:

    <status>

    <status>:= {ON|OFF}.
"""
get_system_nstorage_status(s::SiglentScope) =
    _query_bool(s, ":SYSTem:NSTorage:STATus?")

"""
    set_system_pon!(s, state)

The command sets the state of the Power-On-Line function. When enabled, the instrument will reboot automatically if the power is removed and re-established.

`:SYSTem:PON <state>` (guide PDF p. 385)

    <state>:= {ON|OFF}
"""
set_system_pon!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:PON", state))

"""
    get_system_pon(s)

The query returns the current state of the Power-On-Line function.

`:SYSTem:PON?` (guide PDF p. 385)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_system_pon(s::SiglentScope) =
    _query_bool(s, ":SYSTem:PON?")

"""
    system_reboot!(s)

The command restarts the oscilloscope.

`:SYSTem:REBoot` (guide PDF p. 385)
"""
system_reboot!(s::SiglentScope) =
    scpi_write(s, ":SYSTem:REBoot")

"""
    set_system_remote!(s, state)

The command sets the status of the remote control. When the remote control is turned on, the touch screen, the front panel and the touch screen, front panel and peripheral will be locked, and there will be a remote prompt on the screen.

`:SYSTem:REMote <state>` (guide PDF p. 386)

    <state>:= {ON|OFF}
"""
set_system_remote!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:REMote", state))

"""
    get_system_remote(s)

This query returns the current status of the remote setting.

`:SYSTem:REMote?` (guide PDF p. 386)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_system_remote(s::SiglentScope) =
    _query_bool(s, ":SYSTem:REMote?")

"""
    system_selfcal!(s)

The command instructs the oscilloscope to perform self-calibration.

`:SYSTem:SELFCal` (guide PDF p. 387)
"""
system_selfcal!(s::SiglentScope) =
    scpi_write(s, ":SYSTem:SELFCal")

"""
    get_system_selfcal(s)

The query returns the oscilloscope self-calibration status.

`:SYSTem:SELFCal?` (guide PDF p. 387)

Returns `String`.

Response format:

    <state>

    <state>:= {DOING|DONE}
"""
get_system_selfcal(s::SiglentScope) =
    _query_str(s, ":SYSTem:SELFCal?")

"""
    system_shutdown!(s)

The command shut down the oscilloscope.

`:SYSTem:SHUTdown` (guide PDF p. 387)
"""
system_shutdown!(s::SiglentScope) =
    scpi_write(s, ":SYSTem:SHUTdown")

"""
    set_system_ssaver!(s, time)

The command controls the automatic screensaver, which automatically shuts down the internal color monitor after a preset time.

`:SYSTem:SSAVer <time>` (guide PDF p. 388)

    <time>:= {OFF|1MIN|5MIN|10MIN|30MIN|60MIN}
"""
set_system_ssaver!(s::SiglentScope, time) =
    scpi_write(s, _cmd(":SYSTem:SSAVer", time))

"""
    get_system_ssaver(s)

The query returns whether the automatic screensaver feature is on.

`:SYSTem:SSAVer?` (guide PDF p. 388)

Returns `String`.

Response format:

    <time>

    <time>:= {OFF|1MIN|5MIN|10MIN|30MIN|60MIN}
"""
get_system_ssaver(s::SiglentScope) =
    _query_str(s, ":SYSTem:SSAVer?")

"""
    set_system_time!(s, time)

The command sets the oscilloscope current time using a 24-hour format.

`:SYSTem:TIME <time>` (guide PDF p. 389)

    <time>:= 8-digit NR1 format, from high to low, is expressed as
    2-digit hour, 2-digit minute, and 2-digit second.
"""
set_system_time!(s::SiglentScope, time) =
    scpi_write(s, _cmd(":SYSTem:TIME", time))

"""
    get_system_time(s)

This query returns the oscilloscope current time.

`:SYSTem:TIME?` (guide PDF p. 389)

Returns `String`.

Response format:

    <time>
"""
get_system_time(s::SiglentScope) =
    _query_str(s, ":SYSTem:TIME?")

"""
    set_system_touch!(s, state)

The command sets the status of the touch screen.

`:SYSTem:TOUCh <state>` (guide PDF p. 390)

    <state>:= {ON|OFF}
"""
set_system_touch!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":SYSTem:TOUCh", state))

"""
    get_system_touch(s)

The query returns the current status of the touch screen.

`:SYSTem:TOUCh?` (guide PDF p. 390)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_system_touch(s::SiglentScope) =
    _query_bool(s, ":SYSTem:TOUCh?")

# ---------------------------------------------------------------------- #
# TIMebase commands                                                      #
# ---------------------------------------------------------------------- #

export set_timebase_delay!, get_timebase_delay, set_timebase_reference!,
      get_timebase_reference, set_timebase_reference_position!,
      get_timebase_reference_position, set_timebase_scale!, get_timebase_scale,
      set_timebase_window!, get_timebase_window, set_timebase_window_delay!,
      get_timebase_window_delay, set_timebase_window_scale!, get_timebase_window_scale

"""
    set_timebase_delay!(s, delay_value)

The command specifies the main timebase delay. This delay is the time between the trigger event and the delay reference point on the screen.

`:TIMebase:DELay <delay_value>` (guide PDF p. 392)

    <delay_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2. The range of the value is
    [-5000div*timebase, 5div*timebase].
"""
set_timebase_delay!(s::SiglentScope, delay_value) =
    scpi_write(s, _cmd(":TIMebase:DELay", delay_value))

"""
    get_timebase_delay(s)

The query returns the current delay value.

`:TIMebase:DELay?` (guide PDF p. 392)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_timebase_delay(s::SiglentScope) =
    _query_float(s, ":TIMebase:DELay?")

"""
    set_timebase_reference!(s, type)

The command sets the strategy for the delay value change in the horizontal direction when the horizontal scale is changed.

`:TIMebase:REFerence <type>` (guide PDF p. 393)

    <type>:= {DELay|POSition}
    - DELay means when the time base is changed, the
    horizontal delay value remains fixed. As the horizontal
    timebase scale is changed, the waveform
    expands/contracts around the center of the display.
    - POSition means When the time base is changed, the
    horizontal delay remains fixed to the grid position on the
    display. As the horizontal time base scale is changed, the
    waveform expands/contracts around the position of the
    horizontal display.
"""
set_timebase_reference!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TIMebase:REFerence", type))

"""
    get_timebase_reference(s)

The query returns the current horizontal reference strategy.

`:TIMebase:REFerence?` (guide PDF p. 393)

Returns `String`.

Response format:

    <type>

    <type>:= {DELay|POSition}
"""
get_timebase_reference(s::SiglentScope) =
    _query_str(s, ":TIMebase:REFerence?")

"""
    set_timebase_reference_position!(s, value)

The command sets the horizontal reference center when the reference strategy is DELay.

`:TIMebase:REFerence:POSition <value>` (guide PDF p. 394)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 100].
"""
set_timebase_reference_position!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TIMebase:REFerence:POSition", value))

"""
    get_timebase_reference_position(s)

The query returns the current horizontal reference center.

`:TIMebase:REFerence:POSition?` (guide PDF p. 394)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_timebase_reference_position(s::SiglentScope) =
    _query_int(s, ":TIMebase:REFerence:POSition?")

"""
    set_timebase_scale!(s, value)

The command sets the horizontal scale per division for the main window.

Note: Due to the limitation of the expansion strategy, when the time base is set from large to small, it will automatically adjust to the minimum time base that can be set currently.

`:TIMebase:SCALe <value>` (guide PDF p. 395)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    Note:
    The range of value varies from the models. See the datasheet
    for details.
"""
set_timebase_scale!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TIMebase:SCALe", value))

"""
    get_timebase_scale(s)

The query returns the current horizontal scale setting in seconds per division for the main window.

`:TIMebase:SCALe?` (guide PDF p. 395)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_timebase_scale(s::SiglentScope) =
    _query_float(s, ":TIMebase:SCALe?")

"""
    set_timebase_window!(s, state)

The command turns on or off the zoomed window.

`:TIMebase:WINDow <state>` (guide PDF p. 396)

    <state>:= {ON|OFF}
"""
set_timebase_window!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TIMebase:WINDow", state))

"""
    get_timebase_window(s)

The query returns the state of the zoomed window.

`:TIMebase:WINDow?` (guide PDF p. 396)

Returns `Bool`.

Response format:

    <state>

    <state>:= {ON|OFF}
"""
get_timebase_window(s::SiglentScope) =
    _query_bool(s, ":TIMebase:WINDow?")

"""
    set_timebase_window_delay!(s, delay_value)

The command sets the horizontal position in the zoomed view of the main sweep.

`:TIMebase:WINDow:DELay <delay_value>` (guide PDF p. 397)

    <delay_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    Note:
    • The main sweep range and the main sweep horizontal
    position determine the range for the delay value of the
    zoomed window. It must keep the zoomed view window
    within the main sweep range.
    • If you set the delay to a value outside of the legal range,
    the delay value is automatically set to the nearest legal
    value.
"""
set_timebase_window_delay!(s::SiglentScope, delay_value) =
    scpi_write(s, _cmd(":TIMebase:WINDow:DELay", delay_value))

"""
    get_timebase_window_delay(s)

The query returns the current delay value between the zoomed window and the main sweep.

`:TIMebase:WINDow:DELay?` (guide PDF p. 397)

Returns `Float64`.

Response format:

    <delay_value>

    <delay_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_timebase_window_delay(s::SiglentScope) =
    _query_float(s, ":TIMebase:WINDow:DELay?")

"""
    set_timebase_window_scale!(s, scale_value)

The command sets the zoomed window horizontal scale (seconds/division).

`:TIMebase:WINDow:SCALe <scale_value>` (guide PDF p. 398)

    <scale_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    Note:
    The scale of the zoomed window cannot be greater than that of
    the main window. If you set the value greater than, it will
    automatically be set to the same value as the main window.
"""
set_timebase_window_scale!(s::SiglentScope, scale_value) =
    scpi_write(s, _cmd(":TIMebase:WINDow:SCALe", scale_value))

"""
    get_timebase_window_scale(s)

The query returns the current zoomed window scale setting.

`:TIMebase:WINDow:SCALe?` (guide PDF p. 398)

Returns `Float64`.

Response format:

    <scale_value>

    <scale_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_timebase_window_scale(s::SiglentScope) =
    _query_float(s, ":TIMebase:WINDow:SCALe?")

# ---------------------------------------------------------------------- #
# TRIGger commands                                                       #
# ---------------------------------------------------------------------- #

export get_trigger_frequency, set_trigger_mode!, get_trigger_mode, trigger_run!,
      get_trigger_status, trigger_stop!, set_trigger_type!, get_trigger_type,
      set_trigger_edge_coupling!, get_trigger_edge_coupling, set_trigger_edge_hldevent!,
      get_trigger_edge_hldevent, set_trigger_edge_hldtime!, get_trigger_edge_hldtime,
      set_trigger_edge_holdoff!, get_trigger_edge_holdoff, set_trigger_edge_hstart!,
      get_trigger_edge_hstart, set_trigger_edge_impedance!, get_trigger_edge_impedance,
      set_trigger_edge_level!, get_trigger_edge_level, set_trigger_edge_nreject!,
      get_trigger_edge_nreject, set_trigger_edge_slope!, get_trigger_edge_slope,
      set_trigger_edge_source!, get_trigger_edge_source, set_trigger_slope_coupling!,
      get_trigger_slope_coupling, set_trigger_slope_hldevent!, get_trigger_slope_hldevent,
      set_trigger_slope_hldtime!, get_trigger_slope_hldtime, set_trigger_slope_hlevel!,
      get_trigger_slope_hlevel, set_trigger_slope_holdoff!, get_trigger_slope_holdoff,
      set_trigger_slope_hstart!, get_trigger_slope_hstart, set_trigger_slope_limit!,
      get_trigger_slope_limit, set_trigger_slope_llevel!, get_trigger_slope_llevel,
      set_trigger_slope_nreject!, get_trigger_slope_nreject, set_trigger_slope_slope!,
      get_trigger_slope_slope, set_trigger_slope_source!, get_trigger_slope_source,
      set_trigger_slope_tlower!, get_trigger_slope_tlower, set_trigger_slope_tupper!,
      get_trigger_slope_tupper, set_trigger_pulse_coupling!, get_trigger_pulse_coupling,
      set_trigger_pulse_hldevent!, get_trigger_pulse_hldevent, set_trigger_pulse_hldtime!,
      get_trigger_pulse_hldtime, set_trigger_pulse_holdoff!, get_trigger_pulse_holdoff,
      set_trigger_pulse_hstart!, get_trigger_pulse_hstart, set_trigger_pulse_level!,
      get_trigger_pulse_level, set_trigger_pulse_limit!, get_trigger_pulse_limit,
      set_trigger_pulse_nreject!, get_trigger_pulse_nreject, set_trigger_pulse_polarity!,
      get_trigger_pulse_polarity, set_trigger_pulse_source!, get_trigger_pulse_source,
      set_trigger_pulse_tlower!, get_trigger_pulse_tlower, set_trigger_pulse_tupper!,
      get_trigger_pulse_tupper, set_trigger_video_fcnt!, get_trigger_video_fcnt,
      set_trigger_video_field!, get_trigger_video_field, set_trigger_video_frate!,
      get_trigger_video_frate, set_trigger_video_interlace!, get_trigger_video_interlace,
      set_trigger_video_lcnt!, get_trigger_video_lcnt, set_trigger_video_level!,
      get_trigger_video_level, set_trigger_video_line!, get_trigger_video_line,
      set_trigger_video_source!, get_trigger_video_source, set_trigger_video_standard!,
      get_trigger_video_standard, set_trigger_video_sync!, get_trigger_video_sync,
      set_trigger_window_clevel!, get_trigger_window_clevel, set_trigger_window_coupling!,
      get_trigger_window_coupling, set_trigger_window_dlevel!, get_trigger_window_dlevel,
      set_trigger_window_hldevent!, get_trigger_window_hldevent, set_trigger_window_hldtime!,
      get_trigger_window_hldtime, set_trigger_window_hlevel!, get_trigger_window_hlevel,
      set_trigger_window_holdoff!, get_trigger_window_holdoff, set_trigger_window_hstart!,
      get_trigger_window_hstart, set_trigger_window_llevel!, get_trigger_window_llevel,
      set_trigger_window_nreject!, get_trigger_window_nreject, set_trigger_window_source!,
      get_trigger_window_source, set_trigger_window_type!, get_trigger_window_type,
      set_trigger_interval_coupling!, get_trigger_interval_coupling,
      set_trigger_interval_hldevent!, get_trigger_interval_hldevent,
      set_trigger_interval_hldtime!, get_trigger_interval_hldtime,
      set_trigger_interval_holdoff!, get_trigger_interval_holdoff,
      set_trigger_interval_hstart!, get_trigger_interval_hstart, set_trigger_interval_level!,
      get_trigger_interval_level, set_trigger_interval_limit!, get_trigger_interval_limit,
      set_trigger_interval_nreject!, get_trigger_interval_nreject,
      set_trigger_interval_slope!, get_trigger_interval_slope, set_trigger_interval_source!,
      get_trigger_interval_source, set_trigger_interval_tlower!, get_trigger_interval_tlower,
      set_trigger_interval_tupper!, get_trigger_interval_tupper,
      set_trigger_dropout_coupling!, get_trigger_dropout_coupling,
      set_trigger_dropout_hldevent!, get_trigger_dropout_hldevent,
      set_trigger_dropout_hldtime!, get_trigger_dropout_hldtime,
      set_trigger_dropout_holdoff!, get_trigger_dropout_holdoff, set_trigger_dropout_hstart!,
      get_trigger_dropout_hstart, set_trigger_dropout_level!, get_trigger_dropout_level,
      set_trigger_dropout_nreject!, get_trigger_dropout_nreject, set_trigger_dropout_slope!,
      get_trigger_dropout_slope, set_trigger_dropout_source!, get_trigger_dropout_source,
      set_trigger_dropout_time!, get_trigger_dropout_time, set_trigger_dropout_type!,
      get_trigger_dropout_type, set_trigger_runt_coupling!, get_trigger_runt_coupling,
      set_trigger_runt_hldevent!, get_trigger_runt_hldevent, set_trigger_runt_hldtime!,
      get_trigger_runt_hldtime, set_trigger_runt_hlevel!, get_trigger_runt_hlevel,
      set_trigger_runt_holdoff!, get_trigger_runt_holdoff, set_trigger_runt_hstart!,
      get_trigger_runt_hstart, set_trigger_runt_limit!, get_trigger_runt_limit,
      set_trigger_runt_llevel!, get_trigger_runt_llevel, set_trigger_runt_nreject!,
      get_trigger_runt_nreject, set_trigger_runt_polarity!, get_trigger_runt_polarity,
      set_trigger_runt_source!, get_trigger_runt_source, set_trigger_runt_tlower!,
      get_trigger_runt_tlower, set_trigger_runt_tupper!, get_trigger_runt_tupper,
      set_trigger_pattern_hldevent!, get_trigger_pattern_hldevent,
      set_trigger_pattern_hldtime!, get_trigger_pattern_hldtime,
      set_trigger_pattern_holdoff!, get_trigger_pattern_holdoff, set_trigger_pattern_hstart!,
      get_trigger_pattern_hstart, set_trigger_pattern_input!, get_trigger_pattern_input,
      set_trigger_pattern_level!, get_trigger_pattern_level, set_trigger_pattern_limit!,
      get_trigger_pattern_limit, set_trigger_pattern_logic!, get_trigger_pattern_logic,
      set_trigger_pattern_tlower!, get_trigger_pattern_tlower, set_trigger_pattern_tupper!,
      get_trigger_pattern_tupper, set_trigger_qualified_elevel!,
      get_trigger_qualified_elevel, set_trigger_qualified_eslope!,
      get_trigger_qualified_eslope, set_trigger_qualified_esource!,
      get_trigger_qualified_esource, set_trigger_qualified_limit!,
      get_trigger_qualified_limit, set_trigger_qualified_qlevel!,
      get_trigger_qualified_qlevel, set_trigger_qualified_qsource!,
      get_trigger_qualified_qsource, set_trigger_qualified_tlower!,
      get_trigger_qualified_tlower, set_trigger_qualified_tupper!,
      get_trigger_qualified_tupper, set_trigger_qualified_type!, get_trigger_qualified_type,
      set_trigger_delay_coupling!, get_trigger_delay_coupling, set_trigger_delay_source!,
      get_trigger_delay_source, set_trigger_delay_source2!, get_trigger_delay_source2,
      set_trigger_delay_slope!, get_trigger_delay_slope, set_trigger_delay_slope2!,
      get_trigger_delay_slope2, set_trigger_delay_level!, get_trigger_delay_level,
      set_trigger_delay_level2!, get_trigger_delay_level2, set_trigger_delay_limit!,
      get_trigger_delay_limit, set_trigger_delay_tupper!, get_trigger_delay_tupper,
      set_trigger_delay_tlower!, get_trigger_delay_tlower, set_trigger_nedge_source!,
      get_trigger_nedge_source, set_trigger_nedge_slope!, get_trigger_nedge_slope,
      set_trigger_nedge_idle!, get_trigger_nedge_idle, set_trigger_nedge_edge!,
      get_trigger_nedge_edge, set_trigger_nedge_level!, get_trigger_nedge_level,
      set_trigger_nedge_holdoff!, get_trigger_nedge_holdoff, set_trigger_nedge_hldtime!,
      get_trigger_nedge_hldtime, set_trigger_nedge_hldevent!, get_trigger_nedge_hldevent,
      set_trigger_nedge_hstart!, get_trigger_nedge_hstart, set_trigger_nedge_nreject!,
      get_trigger_nedge_nreject, set_trigger_shold_type!, get_trigger_shold_type,
      set_trigger_shold_csource!, get_trigger_shold_csource, set_trigger_shold_cthreshold!,
      get_trigger_shold_cthreshold, set_trigger_shold_slope!, get_trigger_shold_slope,
      set_trigger_shold_dsource!, get_trigger_shold_dsource, set_trigger_shold_dthreshold!,
      get_trigger_shold_dthreshold, set_trigger_shold_level!, get_trigger_shold_level,
      set_trigger_shold_limit!, get_trigger_shold_limit, set_trigger_shold_tupper!,
      get_trigger_shold_tupper, set_trigger_shold_tlower!, get_trigger_shold_tlower,
      set_trigger_iic_address!, get_trigger_iic_address, set_trigger_iic_alength!,
      get_trigger_iic_alength, set_trigger_iic_condition!, get_trigger_iic_condition,
      set_trigger_iic_dat2!, get_trigger_iic_dat2, set_trigger_iic_data!,
      get_trigger_iic_data, set_trigger_iic_dlength!, get_trigger_iic_dlength,
      set_trigger_iic_limit!, get_trigger_iic_limit, set_trigger_iic_rwbit!,
      get_trigger_iic_rwbit, set_trigger_iic_sclsource!, get_trigger_iic_sclsource,
      set_trigger_iic_sclthreshold!, get_trigger_iic_sclthreshold,
      set_trigger_iic_sdasource!, get_trigger_iic_sdasource, set_trigger_iic_sdathreshold!,
      get_trigger_iic_sdathreshold, set_trigger_spi_bitorder!, get_trigger_spi_bitorder,
      set_trigger_spi_clksource!, get_trigger_spi_clksource, set_trigger_spi_clkthreshold!,
      get_trigger_spi_clkthreshold, set_trigger_spi_cssource!, get_trigger_spi_cssource,
      set_trigger_spi_csthreshold!, get_trigger_spi_csthreshold, set_trigger_spi_cstype!,
      get_trigger_spi_cstype, set_trigger_spi_data!, set_trigger_spi_dlength!,
      get_trigger_spi_dlength, set_trigger_spi_latchedge!, get_trigger_spi_latchedge,
      set_trigger_spi_misosource!, get_trigger_spi_misosource,
      set_trigger_spi_misothreshold!, get_trigger_spi_misothreshold,
      set_trigger_spi_mosisource!, get_trigger_spi_mosisource,
      set_trigger_spi_mosithreshold!, get_trigger_spi_mosithreshold,
      set_trigger_spi_ncssource!, get_trigger_spi_ncssource, set_trigger_spi_ncsthreshold!,
      get_trigger_spi_ncsthreshold, set_trigger_spi_ttype!, get_trigger_spi_ttype,
      set_trigger_uart_baud!, get_trigger_uart_baud, set_trigger_uart_bitorder!,
      get_trigger_uart_bitorder, set_trigger_uart_condition!, get_trigger_uart_condition,
      set_trigger_uart_data!, get_trigger_uart_data, set_trigger_uart_dlength!,
      get_trigger_uart_dlength, set_trigger_uart_idle!, get_trigger_uart_idle,
      set_trigger_uart_limit!, get_trigger_uart_limit, set_trigger_uart_parity!,
      get_trigger_uart_parity, set_trigger_uart_rxsource!, get_trigger_uart_rxsource,
      set_trigger_uart_rxthreshold!, get_trigger_uart_rxthreshold, set_trigger_uart_stop!,
      get_trigger_uart_stop, set_trigger_uart_ttype!, get_trigger_uart_ttype,
      set_trigger_uart_txsource!, get_trigger_uart_txsource, set_trigger_uart_txthreshold!,
      get_trigger_uart_txthreshold, set_trigger_can_baud!, get_trigger_can_baud,
      set_trigger_can_condition!, get_trigger_can_condition, set_trigger_can_dat2!,
      get_trigger_can_dat2, set_trigger_can_data!, get_trigger_can_data, set_trigger_can_id!,
      get_trigger_can_id, set_trigger_can_idlength!, get_trigger_can_idlength,
      set_trigger_can_source!, get_trigger_can_source, set_trigger_can_threshold!,
      get_trigger_can_threshold, set_trigger_lin_baud!, get_trigger_lin_baud,
      set_trigger_lin_condition!, get_trigger_lin_condition, set_trigger_lin_dat2!,
      get_trigger_lin_dat2, set_trigger_lin_data!, get_trigger_lin_data,
      set_trigger_lin_error_checksum!, get_trigger_lin_error_checksum,
      set_trigger_lin_error_dlength!, get_trigger_lin_error_dlength,
      set_trigger_lin_error_id!, get_trigger_lin_error_id, set_trigger_lin_error_parity!,
      get_trigger_lin_error_parity, set_trigger_lin_error_sync!, get_trigger_lin_error_sync,
      set_trigger_lin_id!, get_trigger_lin_id, set_trigger_lin_source!,
      get_trigger_lin_source, set_trigger_lin_standard!, get_trigger_lin_standard,
      set_trigger_lin_threshold!, get_trigger_lin_threshold, set_trigger_flexray_baud!,
      get_trigger_flexray_baud, set_trigger_flexray_condition!,
      get_trigger_flexray_condition, set_trigger_flexray_frame_compare!,
      get_trigger_flexray_frame_compare, set_trigger_flexray_frame_cycle!,
      get_trigger_flexray_frame_cycle, set_trigger_flexray_frame_id!,
      get_trigger_flexray_frame_id, set_trigger_flexray_frame_repetition!,
      get_trigger_flexray_frame_repetition, set_trigger_flexray_source!,
      get_trigger_flexray_source, set_trigger_flexray_threshold!,
      get_trigger_flexray_threshold, set_trigger_canfd_bauddata!, get_trigger_canfd_bauddata,
      set_trigger_canfd_baudnominal!, get_trigger_canfd_baudnominal,
      set_trigger_canfd_condition!, get_trigger_canfd_condition, set_trigger_canfd_dat2!,
      get_trigger_canfd_dat2, set_trigger_canfd_data!, get_trigger_canfd_data,
      set_trigger_canfd_ftype!, get_trigger_canfd_ftype, set_trigger_canfd_id!,
      get_trigger_canfd_id, set_trigger_canfd_idlength!, get_trigger_canfd_idlength,
      set_trigger_canfd_source!, get_trigger_canfd_source, set_trigger_canfd_threshold!,
      get_trigger_canfd_threshold, set_trigger_iis_avariant!, get_trigger_iis_avariant,
      set_trigger_iis_bclksource!, get_trigger_iis_bclksource,
      set_trigger_iis_bclkthreshold!, get_trigger_iis_bclkthreshold,
      set_trigger_iis_bitorder!, get_trigger_iis_bitorder, set_trigger_iis_channel!,
      get_trigger_iis_channel, set_trigger_iis_compare!, get_trigger_iis_compare,
      set_trigger_iis_condition!, get_trigger_iis_condition, set_trigger_iis_dlength!,
      get_trigger_iis_dlength, set_trigger_iis_dsource!, get_trigger_iis_dsource,
      set_trigger_iis_dthreshold!, get_trigger_iis_dthreshold, set_trigger_iis_latchedge!,
      get_trigger_iis_latchedge, set_trigger_iis_lch!, get_trigger_iis_lch,
      set_trigger_iis_value!, get_trigger_iis_value, set_trigger_iis_wssource!,
      get_trigger_iis_wssource, set_trigger_iis_wsthreshold!, get_trigger_iis_wsthreshold

"""
    get_trigger_frequency(s)

The query returns the value of hardware frequency counter in hertz if available. The default precision of the returned frequeny is 3 digits, and the maximum valild precision is 7 digits. Use the command “:FORMat:DATA” to set the data precision.

`:TRIGger:FREQuency?` (guide PDF p. 400)

Returns `Float64`.

Response format:

    <val>

    <val>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_frequency(s::SiglentScope) =
    _query_float(s, ":TRIGger:FREQuency?")

"""
    set_trigger_mode!(s, mode)

The command sets the mode of the trigger.

`:TRIGger:MODE <mode>` (guide PDF p. 401)

    <mode>:= {SINGle|NORMal|AUTO|FTRIG}
    - AUTO: The oscilloscope begins to search for the trigger
    signal that meets the conditions. If the trigger signal is
    satisfied, the running state on the top left corner of the user
    interface shows Trig'd, and the interface shows stable
    waveform. Otherwise, the running state always shows
    Auto, and the interface shows unstable waveform.
    - NORMal: The oscilloscope enters the wait trigger state
    and begins to search for trigger signals that meet the
    conditions. If the trigger signal is satisfied, the running
    state shows Trig'd, and the interface shows stable
    waveform. Otherwise, the running state shows Ready, and
    the interface displays the last triggered waveform
    (previous trigger) or does not display the waveform (no
    previous trigger).
    - SINGle: The backlight of SINGLE key lights up, the
    oscilloscope enters the waiting trigger state and begins to
    search for the trigger signal that meets the conditions. If
    the trigger signal is satisfied, the running state shows
    Trig'd, and the interface shows stable waveform. Then, the
    oscilloscope stops scanning, the RUN/STOP key becomes
    red, and the running status shows Stop. Otherwise, the
    running state shows Ready, and the interface does not
    display the waveform.
    - FTRIG: Force to acquire a frame regardless of whether the
    input signal meets the trigger conditions or not.
"""
set_trigger_mode!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:MODE", mode))

"""
    get_trigger_mode(s)

The query returns the current mode of trigger.

`:TRIGger:MODE?` (guide PDF p. 401)

Returns `String`.

Response format:

    <mode>

    <mode>:= {SINGle|NORMal|AUTO|FTRIG}
"""
get_trigger_mode(s::SiglentScope) =
    _query_str(s, ":TRIGger:MODE?")

"""
    trigger_run!(s)

The command sets the oscilloscope to run.

`:TRIGger:RUN` (guide PDF p. 402)
"""
trigger_run!(s::SiglentScope) =
    scpi_write(s, ":TRIGger:RUN")

"""
    get_trigger_status(s)

The command query returns the current state of the trigger.

`:TRIGger:STATus?` (guide PDF p. 402)

Returns `String`.

Response format:

    <status>

    <status>:= {Arm|Ready|Auto|Trig'd|Stop|Roll}
"""
get_trigger_status(s::SiglentScope) =
    _query_str(s, ":TRIGger:STATus?")

"""
    trigger_stop!(s)

The command sets the oscilloscope from run to stop.

`:TRIGger:STOP` (guide PDF p. 403)
"""
trigger_stop!(s::SiglentScope) =
    scpi_write(s, ":TRIGger:STOP")

"""
    set_trigger_type!(s, type)

The command sets the type of trigger.

`:TRIGger:TYPE <type>` (guide PDF p. 403)

    <type>:= {EDGE|PULSE|SLOPe|INTerval|PATTern|RUNT|
    WINDow|DROPout|VIDeo|QUALified|NTHEdge|DELay|SETup
    hold|IIC|SPI|UART|LIN|CAN|FLEXray|CANFd|IIS|1553B|SENT
    }
"""
set_trigger_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:TYPE", type))

"""
    get_trigger_type(s)

The query returns the current type of trigger.

`:TRIGger:TYPE?` (guide PDF p. 403)

Returns `String`.

Response format:

    <type>

    <type>:= {EDGE|PULSE|SLOPe|INTerval|PATTern|RUNT|
    WINDow|DROPout|VIDeo|QUALified|NTHEdge|DELay|SETup
    hold|IIC|SPI|UART|LIN|CAN|FLEXray|CANFd|IIS|1553B|SENT
    }
"""
get_trigger_type(s::SiglentScope) =
    _query_str(s, ":TRIGger:TYPE?")

"""
    set_trigger_edge_coupling!(s, mode)

The command sets the coupling mode of the edge trigger.

`:TRIGger:EDGE:COUPling <mode>` (guide PDF p. 405)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter that
    adds a low-pass filter in the trigger path to remove
    high-frequency components from the trigger waveform.
    Use the high-frequency rejection filter to remove
    high-frequency noise, such as AM or FM broadcast
    stations, from the trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low-frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_edge_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:EDGE:COUPling", mode))

"""
    get_trigger_edge_coupling(s)

The query returns the current coupling mode of the edge trigger.

`:TRIGger:EDGE:COUPling?` (guide PDF p. 405)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_edge_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:COUPling?")

"""
    set_trigger_edge_hldevent!(s, value)

This command sets the number of holdoff events of the edge trigger.

`:TRIGger:EDGE:HLDEVent <value>` (guide PDF p. 406)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_edge_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:EDGE:HLDEVent", value))

"""
    get_trigger_edge_hldevent(s)

The query returns the current number of holdoff events of the edge trigger.

`:TRIGger:EDGE:HLDEVent?` (guide PDF p. 406)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_edge_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:EDGE:HLDEVent?")

"""
    set_trigger_edge_hldtime!(s, value)

The command sets the holdoff time of the edge trigger.

`:TRIGger:EDGE:HLDTime <value>` (guide PDF p. 407)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
    SHS800X/SHS1000X [80.00E-09, 1.5E+00]
"""
set_trigger_edge_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:EDGE:HLDTime", value))

"""
    get_trigger_edge_hldtime(s)

The query returns the current holdoff time of the edge trigger.

`:TRIGger:EDGE:HLDTime?` (guide PDF p. 407)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_edge_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:EDGE:HLDTime?")

"""
    set_trigger_edge_holdoff!(s, holdoff_type)

The command selects the holdoff type of the edge trigger.

`:TRIGger:EDGE:HOLDoff <holdoff_type>` (guide PDF p. 408)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the number of trigger events that the
    oscilloscope counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope
    waits before re-arming the trigger circuitry.
"""
set_trigger_edge_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:EDGE:HOLDoff", holdoff_type))

"""
    get_trigger_edge_holdoff(s)

The query returns the current holdoff type of the edge trigger.

`:TRIGger:EDGE:HOLDoff?` (guide PDF p. 408)

Returns `String`.

Response format:

    <holdoff_type>

    <holdoff_type>:= {OFF|EVENts|TIME}
"""
get_trigger_edge_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:HOLDoff?")

"""
    set_trigger_edge_hstart!(s, start_holdoff)

The command defines the initial position of the edge trigger holdoff.

`:TRIGger:EDGE:HSTart <start_holdoff>` (guide PDF p. 409)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_edge_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:EDGE:HSTart", start_holdoff))

"""
    get_trigger_edge_hstart(s)

The query returns the initial position of the edge trigger holdoff.

`:TRIGger:EDGE:HSTart?` (guide PDF p. 409)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_edge_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:HSTart?")

"""
    set_trigger_edge_impedance!(s, ohm)

The command sets the edge trigger source impedance, which is only valid when the source is EXT or EXT/5.

`:TRIGger:EDGE:IMPedance <ohm>` (guide PDF p. 410)

    <ohm>:= {ONEMeg|FIFTy}
"""
set_trigger_edge_impedance!(s::SiglentScope, ohm) =
    scpi_write(s, _cmd(":TRIGger:EDGE:IMPedance", ohm))

"""
    get_trigger_edge_impedance(s)

The query returns the impedance of external trigger source.

`:TRIGger:EDGE:IMPedance?` (guide PDF p. 410)

Returns `String`.

Response format:

    <ohm >

    <ohm>:= {ONEMeg|FIFTy}
"""
get_trigger_edge_impedance(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:IMPedance?")

"""
    set_trigger_edge_level!(s, level_value)

The command sets the trigger level of the edge trigger.

`:TRIGger:EDGE:LEVel <level_value>` (guide PDF p. 411)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_edge_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:EDGE:LEVel", level_value))

"""
    get_trigger_edge_level(s)

The query returns the current trigger level value of the edge trigger.

`:TRIGger:EDGE:LEVel?` (guide PDF p. 411)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_trigger_edge_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:EDGE:LEVel?")

"""
    set_trigger_edge_nreject!(s, state)

The command sets the state of the noise rejection.

`:TRIGger:EDGE:NREJect <state>` (guide PDF p. 412)

    <state>:= {OFF|ON}
"""
set_trigger_edge_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:EDGE:NREJect", state))

"""
    get_trigger_edge_nreject(s)

The query returns the current state of the noise rejection.

`:TRIGger:EDGE:NREJect?` (guide PDF p. 412)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_edge_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:EDGE:NREJect?")

"""
    set_trigger_edge_slope!(s, slope_type)

The command sets the slope of the edge trigger.

`:TRIGger:EDGE:SLOPe <slope_type>` (guide PDF p. 413)

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
set_trigger_edge_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:EDGE:SLOPe", slope_type))

"""
    get_trigger_edge_slope(s)

The query returns the current slope setting of the edge trigger.

`:TRIGger:EDGE:SLOPe?` (guide PDF p. 413)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
get_trigger_edge_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:SLOPe?")

"""
    set_trigger_edge_source!(s, source)

The command sets the trigger source of the edge trigger.

`:TRIGger:EDGE:SOURce <source>` (guide PDF p. 414)

    <source>:= {C<x>|D<n>|EX|EX5|LINE}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_edge_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:EDGE:SOURce", source))

"""
    get_trigger_edge_source(s)

The query returns the current trigger source of the edge trigger.

`:TRIGger:EDGE:SOURce?` (guide PDF p. 414)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>|EX|EX5|LINE}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_edge_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:EDGE:SOURce?")

"""
    set_trigger_slope_coupling!(s, mode)

The command sets the coupling mode of the slope trigger.

`:TRIGger:SLOPe:COUPling <mode>` (guide PDF p. 416)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high frequency
    components from the trigger waveform. Use the
    high-frequency reject filter to remove high-frequency
    noise, such as AM or FM broadcast stations, from the
    trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_slope_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:COUPling", mode))

"""
    get_trigger_slope_coupling(s)

The query returns the current the coupling mode of the slope trigger.

`:TRIGger:SLOPe:COUPling?` (guide PDF p. 416)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_slope_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:COUPling?")

"""
    set_trigger_slope_hldevent!(s, value)

This command sets the number of holdoff events of the slope trigger.

`:TRIGger:SLOPe:HLDEVent <value>` (guide PDF p. 417)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_slope_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:HLDEVent", value))

"""
    get_trigger_slope_hldevent(s)

The query returns the current number of holdoff events of the slope trigger.

`:TRIGger:SLOPe:HLDEVent?` (guide PDF p. 417)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_slope_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:SLOPe:HLDEVent?")

"""
    set_trigger_slope_hldtime!(s, value)

This This command sets the holdoff time of the slope trigger.

`:TRIGger:SLOPe:HLDTime <value>` (guide PDF p. 418)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_slope_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:HLDTime", value))

"""
    get_trigger_slope_hldtime(s)

The query returns the current holdoff time of the slope trigger.

`:TRIGger:SLOPe:HLDTime?` (guide PDF p. 418)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_slope_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:SLOPe:HLDTime?")

"""
    set_trigger_slope_hlevel!(s, high_level_value)

The command sets the high level of the slope trigger.

`:TRIGger:SLOPe:HLEVel <high_level_value>` (guide PDF p. 419)

    <high_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The high level value cannot be less than the low level value
    using by the command :TRIGger:SLOPe:LLEVel.
"""
set_trigger_slope_hlevel!(s::SiglentScope, high_level_value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:HLEVel", high_level_value))

"""
    get_trigger_slope_hlevel(s)

The query returns the current high level of the slope trigger.

`:TRIGger:SLOPe:HLEVel?` (guide PDF p. 419)

Returns `Float64`.

Response format:

    <high_level_value>

    <high_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.
"""
get_trigger_slope_hlevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:SLOPe:HLEVel?")

"""
    set_trigger_slope_holdoff!(s, holdoff_type)

The command selects the holdoff type of the slope trigger.

`:TRIGger:SLOPe:HOLDoff <holdoff_type>` (guide PDF p. 420)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry
"""
set_trigger_slope_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:HOLDoff", holdoff_type))

"""
    get_trigger_slope_holdoff(s)

The query returns the curent holdoff type of the slope trigger.

`:TRIGger:SLOPe:HOLDoff?` (guide PDF p. 420)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type>:= {OFF|EVENts|TIME}
"""
get_trigger_slope_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:HOLDoff?")

"""
    set_trigger_slope_hstart!(s, type)

The command defines the initial position of the slope trigger holdoff.

`:TRIGger:SLOPe:HSTart <type>` (guide PDF p. 421)

    <start_type>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_slope_hstart!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:HSTart", type))

"""
    get_trigger_slope_hstart(s)

The query returns the initial position of the slope trigger holdoff.

`:TRIGger:SLOPe:HSTart?` (guide PDF p. 421)

Returns `String`.

Response format:

    <type>

    <type>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_slope_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:HSTart?")

"""
    set_trigger_slope_limit!(s, type)

The command sets the limit range type of the slope trigger.

`:TRIGger:SLOPe:LIMit <type>` (guide PDF p. 422)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_slope_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:LIMit", type))

"""
    get_trigger_slope_limit(s)

The query returns the current limit range type of the slope trigger.

`:TRIGger:SLOPe:LIMit?` (guide PDF p. 422)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_slope_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:LIMit?")

"""
    set_trigger_slope_llevel!(s, low_level_value)

The command sets the low level of the slope trigger.

`:TRIGger:SLOPe:LLEVel <low_level_value>` (guide PDF p. 423)

    <low_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The low level value cannot be greater than the low level value
    using by the command :TRIGger:SLOPe:HLEVel.
"""
set_trigger_slope_llevel!(s::SiglentScope, low_level_value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:LLEVel", low_level_value))

"""
    get_trigger_slope_llevel(s)

The query returns the current low level of the slope trigger.

`:TRIGger:SLOPe:LLEVel?` (guide PDF p. 423)

Returns `Float64`.

Response format:

    <low_level_value>

    <low_level_value>:= Value in NR3 format, including a decimal
    point and exponent, like 1.23E+2.
"""
get_trigger_slope_llevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:SLOPe:LLEVel?")

"""
    set_trigger_slope_nreject!(s, state)

The command sets the state of noise rejection.

`:TRIGger:SLOPe:NREJect <state>` (guide PDF p. 424)

    <state>:= {OFF|ON}
"""
set_trigger_slope_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:NREJect", state))

"""
    get_trigger_slope_nreject(s)

The query returns the current state of noise rejection.

`:TRIGger:SLOPe:NREJect?` (guide PDF p. 424)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_slope_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:SLOPe:NREJect?")

"""
    set_trigger_slope_slope!(s, slope_type)

The command sets the slope of the slope trigger.

`:TRIGger:SLOPe:SLOPe <slope_type>` (guide PDF p. 425)

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
set_trigger_slope_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:SLOPe", slope_type))

"""
    get_trigger_slope_slope(s)

The query returns the current slope of the slope trigger.

`:TRIGger:SLOPe:SLOPe?` (guide PDF p. 425)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing|ALTernate}
"""
get_trigger_slope_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:SLOPe?")

"""
    set_trigger_slope_source!(s, source)

The command sets the trigger source of the slope trigger.

`:TRIGger:SLOPe:SOURce <source>` (guide PDF p. 426)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_slope_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:SOURce", source))

"""
    get_trigger_slope_source(s)

The query returns the current trigger source of the slope trigger.

`:TRIGger:SLOPe:SOURce?` (guide PDF p. 426)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_slope_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:SLOPe:SOURce?")

"""
    set_trigger_slope_tlower!(s, value)

The command sets the lower value of the slope trigger limit type.

`:TRIGger:SLOPe:TLOWer <value>` (guide PDF p. 427)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:SLOPe:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_slope_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:TLOWer", value))

"""
    get_trigger_slope_tlower(s)

The query returns the current lower value of the slope trigger limit type.

`:TRIGger:SLOPe:TLOWer?` (guide PDF p. 427)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_slope_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:SLOPe:TLOWer?")

"""
    set_trigger_slope_tupper!(s, value)

The command sets the upper value of the slope trigger limit type.

`:TRIGger:SLOPe:TUPPer <value>` (guide PDF p. 428)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:SLOPe:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_slope_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SLOPe:TUPPer", value))

"""
    get_trigger_slope_tupper(s)

The query returns the current upper value of the slope trigger limit type.

`:TRIGger:SLOPe:TUPPer?` (guide PDF p. 428)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_slope_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:SLOPe:TUPPer?")

"""
    set_trigger_pulse_coupling!(s, mode)

The command sets the coupling mode of the pulse trigger.

`:TRIGger:PULSe:COUPling <mode>` (guide PDF p. 430)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high frequency
    components from the trigger waveform. Use the
    high-frequency rejection filter to remove high-frequency
    noise, such as AM or FM broadcast stations, from the
    trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_pulse_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:PULSe:COUPling", mode))

"""
    get_trigger_pulse_coupling(s)

The query returns the coupling mode of the pulse trigger.

`:TRIGger:PULSe:COUPling?` (guide PDF p. 430)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_pulse_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:COUPling?")

"""
    set_trigger_pulse_hldevent!(s, value)

This command sets the number of holdoff events of the pulse trigger.

`:TRIGger:PULSe:HLDEVent <value>` (guide PDF p. 431)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_pulse_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PULSe:HLDEVent", value))

"""
    get_trigger_pulse_hldevent(s)

The query returns the current number of holdoff events of the pulse trigger.

`:TRIGger:PULSe:HLDEVent?` (guide PDF p. 431)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_pulse_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:PULSe:HLDEVent?")

"""
    set_trigger_pulse_hldtime!(s, value)

This This command sets the holdoff time of the pulse trigger.

`:TRIGger:PULSe:HLDTime <value>` (guide PDF p. 432)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_pulse_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PULSe:HLDTime", value))

"""
    get_trigger_pulse_hldtime(s)

The query returns the current holdoff time of the pulse trigger.

`:TRIGger:PULSe:HLDTime?` (guide PDF p. 432)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_pulse_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:PULSe:HLDTime?")

"""
    set_trigger_pulse_holdoff!(s, holdoff_type)

The command selects the holdoff type of the pulse trigger.

`:TRIGger:PULSe:HOLDoff <holdoff_type>` (guide PDF p. 433)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_pulse_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:PULSe:HOLDoff", holdoff_type))

"""
    get_trigger_pulse_holdoff(s)

The query returns the current holdoff type of the pulse trigger.

`:TRIGger:PULSe:HOLDoff?` (guide PDF p. 433)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type >:= {OFF|EVENts|TIME}
"""
get_trigger_pulse_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:HOLDoff?")

"""
    set_trigger_pulse_hstart!(s, start_holdoff)

The command defines the initial position of the pulse trigger holdoff.

`:TRIGger:PULSe:HSTart <start_holdoff>` (guide PDF p. 434)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_pulse_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:PULSe:HSTart", start_holdoff))

"""
    get_trigger_pulse_hstart(s)

The query returns the initial position of the pulse trigger holdoff.

`:TRIGger:PULSe:HSTart?` (guide PDF p. 434)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_pulse_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:HSTart?")

"""
    set_trigger_pulse_level!(s, level_value)

The command sets the trigger level of the pulse trigger.

`:TRIGger:PULSe:LEVel <level_value>` (guide PDF p. 435)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_pulse_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:PULSe:LEVel", level_value))

"""
    get_trigger_pulse_level(s)

The query returns the current trigger level of the pulse trigger.

`:TRIGger:PULSe:LEVel?` (guide PDF p. 435)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_trigger_pulse_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:PULSe:LEVel?")

"""
    set_trigger_pulse_limit!(s, type)

The command sets the limit range type of the pulse trigger.

`:TRIGger:PULSe:LIMit <type>` (guide PDF p. 436)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_pulse_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:PULSe:LIMit", type))

"""
    get_trigger_pulse_limit(s)

The query returns the current limit range type of the pulse trigger.

`:TRIGger:PULSe:LIMit?` (guide PDF p. 436)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_pulse_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:LIMit?")

"""
    set_trigger_pulse_nreject!(s, state)

The command sets the state of noise rejection.

`:TRIGger:PULSe:NREJect <state>` (guide PDF p. 437)

    <state>:= {OFF|ON}
"""
set_trigger_pulse_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:PULSe:NREJect", state))

"""
    get_trigger_pulse_nreject(s)

The query returns the current state of the noise rejection function.

`:TRIGger:PULSe:NREJect?` (guide PDF p. 437)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_pulse_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:PULSe:NREJect?")

"""
    set_trigger_pulse_polarity!(s, polarity_type)

The command sets the polarity of the pulse trigger.

`:TRIGger:PULSe:POLarity <polarity_type>` (guide PDF p. 438)

    <polarity_type>:= {POSitive|NEGative}
"""
set_trigger_pulse_polarity!(s::SiglentScope, polarity_type) =
    scpi_write(s, _cmd(":TRIGger:PULSe:POLarity", polarity_type))

"""
    get_trigger_pulse_polarity(s)

The query returns the current polarity of the pulse trigger.

`:TRIGger:PULSe:POLarity?` (guide PDF p. 438)

Returns `String`.

Response format:

    <polarity_type>

    <polarity_type>:= {POSitive|NEGative}
"""
get_trigger_pulse_polarity(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:POLarity?")

"""
    set_trigger_pulse_source!(s, source)

The command sets the trigger source of the pulse trigger.

`:TRIGger:PULSe:SOURce <source>` (guide PDF p. 439)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_pulse_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:PULSe:SOURce", source))

"""
    get_trigger_pulse_source(s)

The query returns the current trigger source of the pulse trigger.

`:TRIGger:PULSe:SOURce?` (guide PDF p. 439)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_pulse_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:PULSe:SOURce?")

"""
    set_trigger_pulse_tlower!(s, value)

The command sets the lower value of the pulse trigger limit type.

`:TRIGger:PULSe:TLOWer <value>` (guide PDF p. 440)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:PULSe:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_pulse_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PULSe:TLOWer", value))

"""
    get_trigger_pulse_tlower(s)

The query returns the current lower value of the pulse trigger limit type.

`:TRIGger:PULSe:TLOWer?` (guide PDF p. 440)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_pulse_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:PULSe:TLOWer?")

"""
    set_trigger_pulse_tupper!(s, value)

The command sets the upper value of the pulse trigger limit type.

`:TRIGger:PULse:TUPPer <value>` (guide PDF p. 441)

    <value>:= Value in NR3 format.The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:PULse:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_pulse_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PULSe:TUPPer", value))

"""
    get_trigger_pulse_tupper(s)

The query returns the current upper value of the pulse trigger limit type.

`:TRIGger:PULSe:TUPPer?` (guide PDF p. 441)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_pulse_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:PULSe:TUPPer?")

"""
    set_trigger_video_fcnt!(s, field_cnt)

The command sets the fields of the custom video trigger.

`:TRIGger:VIDeo:FCNT <field_cnt>` (guide PDF p. 443)

    <field_cnt>:= {1|2|4|8}
"""
set_trigger_video_fcnt!(s::SiglentScope, field_cnt) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:FCNT", field_cnt))

"""
    get_trigger_video_fcnt(s)

The query returns the current fields of the custom video trigger.

`:TRIGger:VIDeo:FCNT?` (guide PDF p. 443)

Returns `String`.

Response format:

    <field_cnt>

    <field_cnt>:= {1|2|4|8}
"""
get_trigger_video_fcnt(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:FCNT?")

"""
    set_trigger_video_field!(s, field)

The command sets the synchronous trigger field when the video standard is NTSC, PAL, 1080i/50 or 1080i/60.

`:TRIGger:VIDeo:FIELd <field>` (guide PDF p. 444)

    <field>:= {1|2}
"""
set_trigger_video_field!(s::SiglentScope, field) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:FIELd", field))

"""
    get_trigger_video_field(s)

The query returns the current synchronous trigger field when the video standard is NTSC, PAL, 1080i/50 or 1080i/60.

`:TRIGger:VIDeo:FIELd?` (guide PDF p. 444)

Returns `String`.

Response format:

    <field>

    <field>:= {1|2}
"""
get_trigger_video_field(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:FIELd?")

"""
    set_trigger_video_frate!(s, frate)

The command sets the frame rate of the custom video trigger.

`:TRIGger:VIDeo:FRATe <frate>` (guide PDF p. 445)

    <frate>:= {25Hz|30Hz|50Hz|60Hz}
"""
set_trigger_video_frate!(s::SiglentScope, frate) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:FRATe", frate))

"""
    get_trigger_video_frate(s)

The query returns the current frame rate of the custom video trigger.

`:TRIGger:VIDeo:FRATe?` (guide PDF p. 445)

Returns `String`.

Response format:

    <frate>

    <frate>:= {25Hz|30Hz|50Hz|60Hz}
"""
get_trigger_video_frate(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:FRATe?")

"""
    set_trigger_video_interlace!(s, interlace)

The command sets the interlace of the custom video trigger.

`:TRIGger:VIDeo:INTerlace <interlace>` (guide PDF p. 446)

    <interlace>:= {1|2|4|8}
"""
set_trigger_video_interlace!(s::SiglentScope, interlace) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:INTerlace", interlace))

"""
    get_trigger_video_interlace(s)

The query returns the current interlace of the custom video trigger.

`:TRIGger:VIDeo:INTerlace?` (guide PDF p. 446)

Returns `String`.

Response format:

    <interlace>

    <interlace>:= {1|2|4|8}
"""
get_trigger_video_interlace(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:INTerlace?")

"""
    set_trigger_video_lcnt!(s, line_cnt)

The command sets the lines of the custom video trigger.

If the "Of Lines" is set to 800, the correct relationship between the interface, of fields, trigger line and trigger field is as follows:

Of Lines Interlace Of Fields Trigger Line Trigger Field 800 1:1 1 800 1 800 2:1 1/2/4/8 400 1/1~2/1~4/1~8 800 4:1 1/2/4/8 300 1/1~2/1~4/1~8 800 8:1 1/2/4/8 100 1/1~2/1~4/1~8

`:TRIGger:VIDeo:LCNT <line_cnt>` (guide PDF p. 447)

    <line_cnt>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [300, 2000].
"""
set_trigger_video_lcnt!(s::SiglentScope, line_cnt) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:LCNT", line_cnt))

"""
    get_trigger_video_lcnt(s)

The query returns the current of lines of the custom video trigger.

`:TRIGger:VIDeo:LCNT?` (guide PDF p. 447)

Returns `Int`.

Response format:

    <line_cnt>

    <line_cnt>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_video_lcnt(s::SiglentScope) =
    _query_int(s, ":TRIGger:VIDeo:LCNT?")

"""
    set_trigger_video_level!(s, level_value)

The command sets the trigger level of the video trigger.

`:TRIGger:VIDeo:LEVel <level_value>` (guide PDF p. 448)

    <level_value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offse
    t,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offse
    t,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_video_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:LEVel", level_value))

"""
    get_trigger_video_level(s)

The query returns the current trigger level of the video trigger.

`:TRIGger:VIDeo:LEVel?` (guide PDF p. 448)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format
"""
get_trigger_video_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:VIDeo:LEVel?")

"""
    set_trigger_video_line!(s, line)

The command sets the synchronous trigger line when the video standard is not custom.

`:TRIGger:VIDeo:LINE <line>` (guide PDF p. 449)

    <line>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    The following table shows the corresponding relations between
    line and field for all video standards(except for custom)
    Standard Field 1 Field 2
    NTSC [1, 263] [1, 262]
    PAL [1, 313] [1, 312]
    HDTV 720P/50,
    720P/60 [1, 750]
    HDTV 1080P/50,
    1080P/60 [1, 1125]
    HDTV 1080i/50,
    1080i/60 [1, 563] [1, 562]
"""
set_trigger_video_line!(s::SiglentScope, line) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:LINE", line))

"""
    get_trigger_video_line(s)

The query returns the current synchronous trigger line when the video standard is not custom.

`:TRIGger:VIDeo:LINE?` (guide PDF p. 449)

Returns `Int`.

Response format:

    <line>

    <line>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_video_line(s::SiglentScope) =
    _query_int(s, ":TRIGger:VIDeo:LINE?")

"""
    set_trigger_video_source!(s, source)

The command sets the trigger source of the video trigger.

`:TRIGger:VIDeo:SOURce <source>` (guide PDF p. 450)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_video_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:SOURce", source))

"""
    get_trigger_video_source(s)

The query returns the current trigger source of the video trigger.

`:TRIGger:VIDeo:SOURce?` (guide PDF p. 450)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_video_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:SOURce?")

"""
    set_trigger_video_standard!(s, standard)

The command sets the standard of the video trigger.

`:TRIGger:VIDeo:STANdard <standard>` (guide PDF p. 451)

    <standard>:=
    {NTSC|PAL|P720L50|P720L60|P1080L50|P1080L60|I1080L50
    |I1080L60|CUSTom}
"""
set_trigger_video_standard!(s::SiglentScope, standard) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:STANdard", standard))

"""
    get_trigger_video_standard(s)

The query returns the current standard of the video trigger.

`:TRIGger:VIDeo:STANdard?` (guide PDF p. 451)

Returns `String`.

Response format:

    <standard>

    <standard>:=
    {NTSC|PAL|P720L50|P720L60|P1080L50|P1080L60|I1080L50
    |I1080L60|CUSTom}
"""
get_trigger_video_standard(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:STANdard?")

"""
    set_trigger_video_sync!(s, sync)

The command sets the sync mode of the video trigger.

`:TRIGger:VIDeo:SYNC <sync>` (guide PDF p. 452)

    <sync>:= {SELect|ANY}
"""
set_trigger_video_sync!(s::SiglentScope, sync) =
    scpi_write(s, _cmd(":TRIGger:VIDeo:SYNC", sync))

"""
    get_trigger_video_sync(s)

The query returns the current sync mode of the video trigger.

`:TRIGger:VIDeo:SYNC?` (guide PDF p. 452)

Returns `String`.

Response format:

    <sync>

    <sync>:= {SELect|ANY}
"""
get_trigger_video_sync(s::SiglentScope) =
    _query_str(s, ":TRIGger:VIDeo:SYNC?")

"""
    set_trigger_window_clevel!(s, value)

The command sets the center level of the window trigger.

`:TRIGger:WINDow:CLEVel <value>` (guide PDF p. 454)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_window_clevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:CLEVel", value))

"""
    get_trigger_window_clevel(s)

The query returns the current center level of the window trigger.

`:TRIGger:WINDow:CLEVel?` (guide PDF p. 454)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_window_clevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:WINDow:CLEVel?")

"""
    set_trigger_window_coupling!(s, mode)

The command sets the coupling mode of the window trigger.

`:TRIGger:WINDow:COUPling <mode>` (guide PDF p. 455)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high-frequency
    components from the trigger waveform. Use the high
    frequency rejection filter to remove high-frequency noise,
    such as AM or FM broadcast stations, from the trigger
    path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_window_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:WINDow:COUPling", mode))

"""
    get_trigger_window_coupling(s)

The query returns the current coupling mode of the window trigger

`:TRIGger:WINDow:COUPling?` (guide PDF p. 455)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_window_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:WINDow:COUPling?")

"""
    set_trigger_window_dlevel!(s, value)

The command sets the delta level of window trigger.

`:TRIGger:WINDow:DLEVel <value>` (guide PDF p. 456)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_window_dlevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:DLEVel", value))

"""
    get_trigger_window_dlevel(s)

The query returns the current delta level of window trigger.

`:TRIGger:WINDow:DLEVel?` (guide PDF p. 456)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_window_dlevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:WINDow:DLEVel?")

"""
    set_trigger_window_hldevent!(s, value)

This command sets the number of holdoff events of the window trigger.

`:TRIGger:WINDow:HLDEVent <value>` (guide PDF p. 457)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_window_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:HLDEVent", value))

"""
    get_trigger_window_hldevent(s)

The query returns the current number of holdoff events of the window trigger.

`:TRIGger:WINDow:HLDEVent?` (guide PDF p. 457)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_window_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:WINDow:HLDEVent?")

"""
    set_trigger_window_hldtime!(s, value)

This This command sets the holdoff time of the window trigger.

`:TRIGger:WINDow:HLDTime <value>` (guide PDF p. 458)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_window_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:HLDTime", value))

"""
    get_trigger_window_hldtime(s)

The query returns the current holdoff time of the window trigger.

`:TRIGger:WINDow:HLDTime?` (guide PDF p. 458)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_window_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:WINDow:HLDTime?")

"""
    set_trigger_window_hlevel!(s, value)

The command sets the high trigger level of window trigger.

`:TRIGger:WINDow:HLEVel <value>` (guide PDF p. 459)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The high level value cannot be less than the low level value
    using by the command :TRIGger:WINDow:LLEVel.
"""
set_trigger_window_hlevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:HLEVel", value))

"""
    get_trigger_window_hlevel(s)

The query returns the current high trigger level of window trigger.

`:TRIGger:WINDow:HLEVel?` (guide PDF p. 459)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_window_hlevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:WINDow:HLEVel?")

"""
    set_trigger_window_holdoff!(s, holdoff_type)

The command selects the holdoff type of the window trigger.

`:TRIGger:WINDow:HOLDoff <holdoff_type>` (guide PDF p. 460)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_window_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:WINDow:HOLDoff", holdoff_type))

"""
    get_trigger_window_holdoff(s)

The query returns the current holdoff type of the window trigger.

`:TRIGger:WINDow:HOLDoff?` (guide PDF p. 460)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type >:= {OFF|EVENts|TIME}
"""
get_trigger_window_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:WINDow:HOLDoff?")

"""
    set_trigger_window_hstart!(s, start_holdoff)

The command defines the initial position of the window trigger holdoff.

`:TRIGger:WINDow:HSTart <start_holdoff>` (guide PDF p. 461)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the time
    of the last trigger.
"""
set_trigger_window_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:WINDow:HSTart", start_holdoff))

"""
    get_trigger_window_hstart(s)

The query returns the initial position of the window trigger holdoff.

`:TRIGger:WINDow:HSTart?` (guide PDF p. 461)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_window_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:WINDow:HSTart?")

"""
    set_trigger_window_llevel!(s, value)

The command sets the low trigger level of the window trigger.

`:TRIGger:WINDow:LLEVel <value>` (guide PDF p. 462)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]

    Note:
    The low level value cannot be greater than the high level value
    using by the command :TRIGger:WINDow:HLEVel.
"""
set_trigger_window_llevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:WINDow:LLEVel", value))

"""
    get_trigger_window_llevel(s)

The query returns the current low trigger level of the window trigger.

`:TRIGger:WINDow:LLEVel?` (guide PDF p. 462)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_window_llevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:WINDow:LLEVel?")

"""
    set_trigger_window_nreject!(s, state)

The command the state of noise reject.

`:TRIGger:WINDow:NREJect <state>` (guide PDF p. 463)

    <state>:= {OFF|ON}
"""
set_trigger_window_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:WINDow:NREJect", state))

"""
    get_trigger_window_nreject(s)

The query returns the current state of noise reject.

`:TRIGger:WINDow:NREJect?` (guide PDF p. 463)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_window_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:WINDow:NREJect?")

"""
    set_trigger_window_source!(s, source)

The command sets the trigger source of the window trigger.

`:TRIGger:WINDow:SOURce <source>` (guide PDF p. 464)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_window_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:WINDow:SOURce", source))

"""
    get_trigger_window_source(s)

The query returns the current trigger source of the window trigger.

`:TRIGger:WINDow:SOURce?` (guide PDF p. 464)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_window_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:WINDow:SOURce?")

"""
    set_trigger_window_type!(s, type)

The command sets the window type of the window trigger.

`:TRIGger:WINDow:TYPE <type>` (guide PDF p. 465)

    <type>:= {ABSolute|RELative}
"""
set_trigger_window_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:WINDow:TYPE", type))

"""
    get_trigger_window_type(s)

The query returns the current window type of the window trigger.

`:TRIGger:WINDow:TYPE?` (guide PDF p. 465)

Returns `String`.

Response format:

    <type>

    <type>:= {ABSolute|RELative}
"""
get_trigger_window_type(s::SiglentScope) =
    _query_str(s, ":TRIGger:WINDow:TYPE?")

"""
    set_trigger_interval_coupling!(s, mode)

The command sets the coupling mode of the interval trigger.

`:TRIGger:INTerval:COUPling <mode>` (guide PDF p. 467)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high-frequency
    components from the trigger waveform. Use the
    high-frequency reject filter to remove high-frequency
    noise, such as AM or FM broadcast stations, from the
    trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_interval_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:INTerval:COUPling", mode))

"""
    get_trigger_interval_coupling(s)

The query returns the current coupling mode of the interval trigger.

`:TRIGger:INTerval:COUPling?` (guide PDF p. 467)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_interval_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:COUPling?")

"""
    set_trigger_interval_hldevent!(s, value)

This command sets the number of holdoff events of the interval trigger.

`:TRIGger:INTerval:HLDEVent <value>` (guide PDF p. 468)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_interval_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:INTerval:HLDEVent", value))

"""
    get_trigger_interval_hldevent(s)

The query returns the current number of holdoff events of the interval trigger.

`:TRIGger:INTerval:HLDEVent?` (guide PDF p. 468)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_interval_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:INTerval:HLDEVent?")

"""
    set_trigger_interval_hldtime!(s, value)

This This command sets the holdoff time of the interval trigger.

`:TRIGger:INTerval:HLDTime <value>` (guide PDF p. 469)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_interval_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:INTerval:HLDTime", value))

"""
    get_trigger_interval_hldtime(s)

The query returns the current holdoff time of the interval trigger.

`:TRIGger:INTerval:HLDTime?` (guide PDF p. 469)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_interval_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:INTerval:HLDTime?")

"""
    set_trigger_interval_holdoff!(s, holdoff_type)

The command selects the holdoff type of the interval trigger.

`:TRIGger:INTerval:HOLDoff <holdoff_type>` (guide PDF p. 470)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_interval_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:INTerval:HOLDoff", holdoff_type))

"""
    get_trigger_interval_holdoff(s)

The query returns the current holdoff type of the interval trigger.

`:TRIGger:INTerval:HOLDoff?` (guide PDF p. 470)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type >:= {OFF|EVENts|TIME}
"""
get_trigger_interval_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:HOLDoff?")

"""
    set_trigger_interval_hstart!(s, start_holdoff)

The command sets the start holdoff mode of the interval trigger.

`:TRIGger:INTerval:HSTart <start_holdoff>` (guide PDF p. 471)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    LAST_TRIG means the initial position of holdoff is the first time
    point satisfying the trigger condition.
    ACQ_START means the initial position of holdoff is the time of
    the last trigger.
"""
set_trigger_interval_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:INTerval:HSTart", start_holdoff))

"""
    get_trigger_interval_hstart(s)

The query returns the current start holdoff mode of the interval trigger.

`:TRIGger:INTerval:HSTart?` (guide PDF p. 471)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_interval_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:HSTart?")

"""
    set_trigger_interval_level!(s, level_value)

The command sets the trigger level of the interval trigger.

`:TRIGger:INTerval:LEVel <level_value>` (guide PDF p. 472)

    <level_value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_interval_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:INTerval:LEVel", level_value))

"""
    get_trigger_interval_level(s)

The query returns the current trigger level of the interval trigger.

`:TRIGger:INTerval:LEVel?` (guide PDF p. 472)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format
"""
get_trigger_interval_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:INTerval:LEVel?")

"""
    set_trigger_interval_limit!(s, type)

The command sets the limit range type of the interval trigger.

`:TRIGger:INTerval:LIMit <type>` (guide PDF p. 473)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_interval_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:INTerval:LIMit", type))

"""
    get_trigger_interval_limit(s)

The query returns the current limit range type of the interval trigger.

`:TRIGger:INTerval:LIMit?` (guide PDF p. 473)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_interval_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:LIMit?")

"""
    set_trigger_interval_nreject!(s, state)

The command sets the state of the noise rejection.

`:TRIGger:INTerval:NREJect <state>` (guide PDF p. 474)

    <state>:= {OFF|ON}
"""
set_trigger_interval_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:INTerval:NREJect", state))

"""
    get_trigger_interval_nreject(s)

The query returns the current state of the noise rejection function.

`:TRIGger:INTerval:NREJect?` (guide PDF p. 474)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_interval_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:INTerval:NREJect?")

"""
    set_trigger_interval_slope!(s, slope_type)

The command sets the slope of the interval trigger.

`:TRIGger:INTerval:SLOPe <slope_type>` (guide PDF p. 475)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_interval_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:INTerval:SLOPe", slope_type))

"""
    get_trigger_interval_slope(s)

The query returns the current slope of the interval trigger.

`:TRIGger:INTerval:SLOPe?` (guide PDF p. 475)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_interval_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:SLOPe?")

"""
    set_trigger_interval_source!(s, source)

The command sets the trigger source of the interval trigger.

`:TRIGger:INTerval:SOURce <source>` (guide PDF p. 476)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_interval_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:INTerval:SOURce", source))

"""
    get_trigger_interval_source(s)

The query returns the current trigger source of the interval trigger.

`:TRIGger:INTerval:SOURce?` (guide PDF p. 476)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_interval_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:INTerval:SOURce?")

"""
    set_trigger_interval_tlower!(s, value)

The command sets the lower value of the interval trigger limit type.

`:TRIGger:INTerval:TLOWer <value>` (guide PDF p. 477)

    <value>:= Value in NR3 format. The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:INTerval:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_interval_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:INTerval:TLOWer", value))

"""
    get_trigger_interval_tlower(s)

The query returns the current lower value of the interval trigger limit type.

`:TRIGger:INTerval:TLOWer?` (guide PDF p. 477)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format
"""
get_trigger_interval_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:INTerval:TLOWer?")

"""
    set_trigger_interval_tupper!(s, value)

The command sets the upper value of the interval trigger limit type.

`:TRIGger:INTerval:TUPPer <value>` (guide PDF p. 478)

    <value>:= Value in NR3 format. The range of the value varies
    by model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:INTerval:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_interval_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:INTerval:TUPPer", value))

"""
    get_trigger_interval_tupper(s)

The query returns the current upper value of the interval trigger limit type.

`:TRIGger:INTerval:TUPPer?` (guide PDF p. 478)

Returns `Float64`.

Response format:

    <tupper_value>

    <tupper_value>:= Value in NR3 format.
"""
get_trigger_interval_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:INTerval:TUPPer?")

"""
    set_trigger_dropout_coupling!(s, mode)

The command sets the coupling mode of the dropout trigger.

`:TRIGger:DROPout:COUPling <mode>` (guide PDF p. 480)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high-frequency
    components from the trigger waveform. Use the
    high-frequency rejection filter to remove high-frequency
    noise, such as AM or FM broadcast stations, from the
    trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_dropout_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:DROPout:COUPling", mode))

"""
    get_trigger_dropout_coupling(s)

The query returns the current coupling mode of the dropout trigger.

`:TRIGger:DROPout:COUPling?` (guide PDF p. 480)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_dropout_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:COUPling?")

"""
    set_trigger_dropout_hldevent!(s, value)

This command sets the number of holdoff events of the dropout trigger.

`:TRIGger:DROPout:HLDEVent <value>` (guide PDF p. 481)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_dropout_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:DROPout:HLDEVent", value))

"""
    get_trigger_dropout_hldevent(s)

The query returns the current number of holdoff events of the dropout trigger.

`:TRIGger:DROPout:HLDEVent?` (guide PDF p. 481)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_dropout_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:DROPout:HLDEVent?")

"""
    set_trigger_dropout_hldtime!(s, value)

This This command sets the holdoff time of the dropout trigger.

`:TRIGger:DROPout:HLDTime <value>` (guide PDF p. 482)

    <value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_dropout_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:DROPout:HLDTime", value))

"""
    get_trigger_dropout_hldtime(s)

The query returns the current holdoff time of the dropout trigger.

`:TRIGger:DROPout:HLDTime?` (guide PDF p. 482)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_dropout_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:DROPout:HLDTime?")

"""
    set_trigger_dropout_holdoff!(s, holdoff_type)

The command selects the holdoff type of the dropout trigger.

`:TRIGger:DROPout:HOLDoff <holdoff_type>` (guide PDF p. 483)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_dropout_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:DROPout:HOLDoff", holdoff_type))

"""
    get_trigger_dropout_holdoff(s)

The query returns the current holdoff type of the dropout trigger.

`:TRIGger:DROPout:HOLDoff?` (guide PDF p. 483)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type>:= {OFF|EVENts|TIME}
"""
get_trigger_dropout_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:HOLDoff?")

"""
    set_trigger_dropout_hstart!(s, start_holdoff)

The command sets the start holdoff mode of the dropout trigger.

`:TRIGger:DROPout:HSTart <start_holdoff>` (guide PDF p. 484)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_dropout_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:DROPout:HSTart", start_holdoff))

"""
    get_trigger_dropout_hstart(s)

The query returns the current start holdoff mode of the dropout trigger.

`:TRIGger:DROPout:HSTart?` (guide PDF p. 484)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_dropout_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:HSTart?")

"""
    set_trigger_dropout_level!(s, level_value)

The command sets the trigger level of the dropout trigger.

`:TRIGger:DROPout:LEVel <level_value>` (guide PDF p. 485)

    <level_value>:= Value in NR3 format.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_dropout_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:DROPout:LEVel", level_value))

"""
    get_trigger_dropout_level(s)

The query returns the current trigger level of the dropout trigger.

`:TRIGger:DROPout:LEVel?` (guide PDF p. 485)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format.
"""
get_trigger_dropout_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:DROPout:LEVel?")

"""
    set_trigger_dropout_nreject!(s, state)

The command sets the state of the noise rejection.

`:TRIGger:DROPout:NREJect <state>` (guide PDF p. 486)

    <state>:= {OFF|ON}
"""
set_trigger_dropout_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:DROPout:NREJect", state))

"""
    get_trigger_dropout_nreject(s)

The query returns the current state of the noise rejection function.

`:TRIGger:DROPout:NREJect?` (guide PDF p. 486)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_dropout_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:DROPout:NREJect?")

"""
    set_trigger_dropout_slope!(s, slope_type)

The command sets the slope of the dropout trigger.

`:TRIGger:DROPout:SLOPe <slope_type>` (guide PDF p. 487)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_dropout_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:DROPout:SLOPe", slope_type))

"""
    get_trigger_dropout_slope(s)

The query returns the current slope of the dropout trigger.

`:TRIGger:DROPout:SLOPe?` (guide PDF p. 487)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_dropout_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:SLOPe?")

"""
    set_trigger_dropout_source!(s, source)

The command sets the trigger source of the dropout trigger.

`:TRIGger:DROPout:SOURce <source>` (guide PDF p. 488)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_dropout_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:DROPout:SOURce", source))

"""
    get_trigger_dropout_source(s)

The query returns the current trigger source of the dropout trigger.

`:TRIGger:DROPout:SOURce?` (guide PDF p. 488)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_dropout_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:SOURce?")

"""
    set_trigger_dropout_time!(s, time)

The command sets the dropout time of the dropout trigger.

`:TRIGger:DROPout:TIME <time>` (guide PDF p. 489)

    <time>:= Value in NR3 format. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]
"""
set_trigger_dropout_time!(s::SiglentScope, time) =
    scpi_write(s, _cmd(":TRIGger:DROPout:TIME", time))

"""
    get_trigger_dropout_time(s)

The query returns the current time of the dropout trigger.

`:TRIGger:DROPout:TIME?` (guide PDF p. 489)

Returns `Float64`.

Response format:

    <time>

    <time>:= Value in NR3 format
"""
get_trigger_dropout_time(s::SiglentScope) =
    _query_float(s, ":TRIGger:DROPout:TIME?")

"""
    set_trigger_dropout_type!(s, type)

The command sets the over time type of the dropout trigger.

`:TRIGger:DROPout:TYPE <type>` (guide PDF p. 490)

    <type>:= {EDGE|STATe}
"""
set_trigger_dropout_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:DROPout:TYPE", type))

"""
    get_trigger_dropout_type(s)

The query returns the current over time type of the dropout trigger.

`:TRIGger:DROPout:TYPE?` (guide PDF p. 490)

Returns `String`.

Response format:

    <type>

    <type>:= {EDGE|STATe}
"""
get_trigger_dropout_type(s::SiglentScope) =
    _query_str(s, ":TRIGger:DROPout:TYPE?")

"""
    set_trigger_runt_coupling!(s, mode)

The command sets the coupling mode of the runt trigger.

`:TRIGger:RUNT:COUPling <mode>` (guide PDF p. 492)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter adds a
    low-pass filter in the trigger path to remove high frequency
    components from the trigger waveform. Use the
    high-frequency reject filter to remove high-frequency
    noise, such as AM or FM broadcast stations, from the
    trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_runt_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:RUNT:COUPling", mode))

"""
    get_trigger_runt_coupling(s)

The query returns the current coupling mode of the runt trigger.

`:TRIGger:RUNT:COUPling?` (guide PDF p. 492)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_runt_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:COUPling?")

"""
    set_trigger_runt_hldevent!(s, value)

This command sets the number of holdoff events of the runt trigger.

`:TRIGger:RUNT:HLDEVent <value>` (guide PDF p. 493)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_runt_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:HLDEVent", value))

"""
    get_trigger_runt_hldevent(s)

The query returns the current number of holdoff events of the runt trigger.

`:TRIGger:RUNT:HLDEVent?` (guide PDF p. 493)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_runt_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:RUNT:HLDEVent?")

"""
    set_trigger_runt_hldtime!(s, value)

This This command sets the holdoff time of the runt trigger.

`:TRIGger:RUNT:HLDTime <value>` (guide PDF p. 494)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_runt_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:HLDTime", value))

"""
    get_trigger_runt_hldtime(s)

The query returns the current holdoff time of the runt trigger.

`:TRIGger:RUNT:HLDTime?` (guide PDF p. 494)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_runt_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:RUNT:HLDTime?")

"""
    set_trigger_runt_hlevel!(s, value)

The command sets the high trigger level of the runt trigger.

`:TRIGger:RUNT:HLEVel <value>` (guide PDF p. 495)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]

    Note:
    The high level value cannot be less than the low level value
    using by the command :TRIGger:RUNT:LLEVel.
"""
set_trigger_runt_hlevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:HLEVel", value))

"""
    get_trigger_runt_hlevel(s)

The query returns the current high trigger level of the runt trigger.

`:TRIGger:RUNT:HLEVel?` (guide PDF p. 495)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_runt_hlevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:RUNT:HLEVel?")

"""
    set_trigger_runt_holdoff!(s, holdoff_type)

The command selects the holdoff type of the runt trigger.

`:TRIGger:RUNT:HOLDoff <holdoff_type>` (guide PDF p. 496)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_runt_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:RUNT:HOLDoff", holdoff_type))

"""
    get_trigger_runt_holdoff(s)

The query returns the current holdoff type of the runt trigger.

`:TRIGger:RUNT:HOLDoff?` (guide PDF p. 496)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type>:= {OFF|EVENts|TIME}
"""
get_trigger_runt_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:HOLDoff?")

"""
    set_trigger_runt_hstart!(s, start_holdoff)

The command sets the start holdoff mode of the runt trigger.

`:TRIGger:RUNT:HSTart <start_holdoff>` (guide PDF p. 497)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_runt_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:RUNT:HSTart", start_holdoff))

"""
    get_trigger_runt_hstart(s)

The query returns the current start holdoff mode of the runt trigger.

`:TRIGger:RUNT:HSTart?` (guide PDF p. 497)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_runt_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:HSTart?")

"""
    set_trigger_runt_limit!(s, type)

The command sets the limit range type of the runt trigger.

`:TRIGger:RUNT:LIMit <type>` (guide PDF p. 498)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_runt_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:RUNT:LIMit", type))

"""
    get_trigger_runt_limit(s)

The query returns the current limit range type of the runt trigger.

`:TRIGger:RUNT:LIMit?` (guide PDF p. 498)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_runt_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:LIMit?")

"""
    set_trigger_runt_llevel!(s, value)

The command sets the low trigger level of the runt trigger.

`:TRIGger:RUNT:LLEVel <value>` (guide PDF p. 499)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]

    Note:
    The low level value cannot be greater than the high level value
    using by the command :TRIGger:RUNT:HLEVel.
"""
set_trigger_runt_llevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:LLEVel", value))

"""
    get_trigger_runt_llevel(s)

The query returns the current low trigger level of the runt trigger.

`:TRIGger:RUNT:LLEVel?` (guide PDF p. 499)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_runt_llevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:RUNT:LLEVel?")

"""
    set_trigger_runt_nreject!(s, state)

The command sets the state of noise rejection.

`:TRIGger:RUNT:NREJect <state>` (guide PDF p. 500)

    <state>:= {OFF|ON}
"""
set_trigger_runt_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:RUNT:NREJect", state))

"""
    get_trigger_runt_nreject(s)

The query returns the current state of noise rejection function.

`:TRIGger:RUNT:NREJect?` (guide PDF p. 500)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_runt_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:RUNT:NREJect?")

"""
    set_trigger_runt_polarity!(s, polarity_type)

The command sets the polarity of the runt trigger.

`:TRIGger:RUNT:POLarity <polarity_type>` (guide PDF p. 501)

    <polarity_type>:= {POSitive|NEGative}
"""
set_trigger_runt_polarity!(s::SiglentScope, polarity_type) =
    scpi_write(s, _cmd(":TRIGger:RUNT:POLarity", polarity_type))

"""
    get_trigger_runt_polarity(s)

The query returns the current polarity of the runt trigger.

`:TRIGger:RUNT:POLarity?` (guide PDF p. 501)

Returns `String`.

Response format:

    <polarity_type>

    <polarity_type>:= {POSitive|NEGative}
"""
get_trigger_runt_polarity(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:POLarity?")

"""
    set_trigger_runt_source!(s, source)

The command sets the trigger source of the runt trigger.

`:TRIGger:RUNT:SOURce <source>` (guide PDF p. 502)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_runt_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:RUNT:SOURce", source))

"""
    get_trigger_runt_source(s)

The query returns the current trigger source of the runt trigger.

`:TRIGger:RUNT:SOURce?` (guide PDF p. 502)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_runt_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:RUNT:SOURce?")

"""
    set_trigger_runt_tlower!(s, value)

The command sets the lower value of the runt trigger limit type.

`:TRIGger:RUNT:TLOWer <value>` (guide PDF p. 503)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:RUNT:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_runt_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:TLOWer", value))

"""
    get_trigger_runt_tlower(s)

The query returns the current lower value of the runt trigger limit type.

`:TRIGger:RUNT:TLOWer?` (guide PDF p. 503)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_runt_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:RUNT:TLOWer?")

"""
    set_trigger_runt_tupper!(s, value)

The command sets the upper value of the runt trigger limit type.

`:TRIGger:PULse:RUNT <value>` (guide PDF p. 504)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value varies by
    model, see the table below for details.
    Model Value Range
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [2.00E-09, 2.00E+01]
    SHS800X/SHS1000X [2.00E-09, 4.20E+00]

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:RUNT:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_runt_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:RUNT:TUPPer", value))

"""
    get_trigger_runt_tupper(s)

The query returns the current upper value of the runt trigger limit type.

`:TRIGger:RUNT:TUPPer?` (guide PDF p. 504)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_runt_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:RUNT:TUPPer?")

"""
    set_trigger_pattern_hldevent!(s, value)

This command sets the number of holdoff events of the pattern trigger.

`:TRIGger:PATTern:HLDEVent <value>` (guide PDF p. 506)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_pattern_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PATTern:HLDEVent", value))

"""
    get_trigger_pattern_hldevent(s)

The query returns the current number of holdoff events of the pattern trigger.

`:TRIGger:PATTern:HLDEVent?` (guide PDF p. 506)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_pattern_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:PATTern:HLDEVent?")

"""
    set_trigger_pattern_hldtime!(s, value)

This This command sets the holdoff time of the pattern trigger.

`:TRIGger:PATTern:HLDTime <value>` (guide PDF p. 507)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Mode value
    SDS5000X
    SDS2000X Plus
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
    SHS800X/SHS1000X [80.00E-09, 1.50E+00]
"""
set_trigger_pattern_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PATTern:HLDTime", value))

"""
    get_trigger_pattern_hldtime(s)

The query returns the current holdoff time of the pattern trigger.

`:TRIGger:PATTern:HLDTime?` (guide PDF p. 507)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_pattern_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:PATTern:HLDTime?")

"""
    set_trigger_pattern_holdoff!(s, holdoff_type)

The command selects the holdoff type of the pattern trigger.

`:TRIGger:PATTern:HOLDoff <holdoff_type>` (guide PDF p. 508)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff
    - EVENts means the amount of events that the oscilloscope
    counts before re-arming the trigger circuitry
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry
"""
set_trigger_pattern_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:PATTern:HOLDoff", holdoff_type))

"""
    get_trigger_pattern_holdoff(s)

The query returns the current holdoff type of the pattern trigger.

`:TRIGger:PATTern:HOLDoff?` (guide PDF p. 508)

Returns `String`.

Response format:

    <holdoff_type>

    < holdoff_type >:= {OFF|EVENts|TIME}
"""
get_trigger_pattern_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:PATTern:HOLDoff?")

"""
    set_trigger_pattern_hstart!(s, start_holdoff)

The command sets the start holdoff mode of the pattern trigger.

`:TRIGger:PATTern:HSTart <start_holdoff>` (guide PDF p. 509)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_pattern_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:PATTern:HSTart", start_holdoff))

"""
    get_trigger_pattern_hstart(s)

The query returns the current start holdoff mode of the pattern trigger.

`:TRIGger:PATTern:HSTart?` (guide PDF p. 509)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_pattern_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:PATTern:HSTart?")

"""
    set_trigger_pattern_input!(s, logic...)

The command specifies the logical input condition for the channel (Cx) and digital channel (Dx) of the pattern trigger.

`:TRIGger:PATTern:INPut <logic>[...[,<logic>]]` (guide PDF p. 510)

    <logic>:= {X|L|H}
    - X means the "don't care" state.
    - H means the logic high state.
    - L means the logic low state.

    Note:
    Parameters are configured to corresponding sources in the order
    of C1-C<n>, D0-D15.
"""
set_trigger_pattern_input!(s::SiglentScope, logic...) =
    scpi_write(s, _cmd(":TRIGger:PATTern:INPut", logic...))

"""
    get_trigger_pattern_input(s)

The query returns the logical input condition of pattern trigger.

`:TRIGger:PATTern:INPut?` (guide PDF p. 510)

Returns `String`.

Response format:

    <input>

    <input>:= {X|L|H}
"""
get_trigger_pattern_input(s::SiglentScope) =
    _query_str(s, ":TRIGger:PATTern:INPut?")

"""
    set_trigger_pattern_level!(s, source, value)

The command sets the trigger level of source in the pattern trigger.

`:TRIGger:PATTern:LEVel <source>,<value>` (guide PDF p. 511)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_pattern_level!(s::SiglentScope, source, value) =
    scpi_write(s, _cmd(":TRIGger:PATTern:LEVel", source, value))

"""
    get_trigger_pattern_level(s, source)

The query returns the current trigger level of source in the pattern trigger.

`:TRIGger:PATTern:LEVel? <source>` (guide PDF p. 511)

Returns `String`.

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

Response format:

    <source>,<value>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <value>:= Value in NR3 format.
"""
get_trigger_pattern_level(s::SiglentScope, source) =
    _query_str(s, _cmd(":TRIGger:PATTern:LEVel?", source))

"""
    set_trigger_pattern_limit!(s, type)

The command sets the limit range type of the pattern trigger when the logic combination is AND or NOR.

`:TRIGger:PATTern:LIMit <type>` (guide PDF p. 513)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_pattern_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:PATTern:LIMit", type))

"""
    get_trigger_pattern_limit(s)

The query returns the current limit range type of the pattern trigger.

`:TRIGger:PATTern:LIMit?` (guide PDF p. 513)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_pattern_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:PATTern:LIMit?")

"""
    set_trigger_pattern_logic!(s, type)

The command sets the logical combination of the input channels for the pattern trigger.

`:TRIGger:PATTern:LOGic <type>` (guide PDF p. 514)

    <type>:= {AND|OR|NAND|NOR}
"""
set_trigger_pattern_logic!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:PATTern:LOGic", type))

"""
    get_trigger_pattern_logic(s)

The query returns the current logical combination of the pattern trigger.

`:TRIGger:PATTern:LOGic?` (guide PDF p. 514)

Returns `String`.

Response format:

    <logic_type>

    <logic_type>:= {AND|OR|NAND|NOR}
"""
get_trigger_pattern_logic(s::SiglentScope) =
    _query_str(s, ":TRIGger:PATTern:LOGic?")

"""
    set_trigger_pattern_tlower!(s, value)

The command sets the lower value of the pattern trigger limit type when the logic combination is AND or NOR.

`:TRIGger:PATTern:TLOWer <value>` (guide PDF p. 515)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [2.00E-09,
    2.00E+01].

    Note:
    - The lower value cannot be greater than the upper value
    using by the command :TRIGger:PATTern:TUPPer.
    - The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_pattern_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PATTern:TLOWer", value))

"""
    get_trigger_pattern_tlower(s)

The query returns the current lower value of the pattern trigger limit type.

`:TRIGger:PATTern:TLOWer?` (guide PDF p. 515)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_pattern_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:PATTern:TLOWer?")

"""
    set_trigger_pattern_tupper!(s, value)

The command sets the upper value of the pattern trigger limit type when the logic combination is AND or NOR.

`:TRIGger:PULse:PATTern <value>` (guide PDF p. 516)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [3.00E-09,
    2.00E+01].

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:PATTern:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_pattern_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:PATTern:TUPPer", value))

"""
    get_trigger_pattern_tupper(s)

The query returns the current upper value of the pattern trigger limit type.

`:TRIGger:PATTern:TUPPer?` (guide PDF p. 516)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_pattern_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:PATTern:TUPPer?")

"""
    set_trigger_qualified_elevel!(s, value)

The command sets the edge trigger level of the edge source in the qualified trigger.

`:TRIGger:QUALified:ELEVel <value>` (guide PDF p. 518)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_qualified_elevel!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:QUALified:ELEVel", value))

"""
    get_trigger_qualified_elevel(s)

The query returns the current edge trigger level in the qualified trigger.

`:TRIGger:QUALified:ELEVel?` (guide PDF p. 518)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_qualified_elevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:QUALified:ELEVel?")

"""
    set_trigger_qualified_eslope!(s, type)

The command sets the edge trigger slope in the qualified trigger.

`:TRIGger:QUALified:ESLope <type>` (guide PDF p. 519)

    <type>:= {RISing|FALLing}
"""
set_trigger_qualified_eslope!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:QUALified:ESLope", type))

"""
    get_trigger_qualified_eslope(s)

The query returns the current edge trigger slope in the qualified trigger.

`:TRIGger:QUALified:ESLope?` (guide PDF p. 519)

Returns `String`.

Response format:

    <type>

    <type>:= {RISing|FALLing}
"""
get_trigger_qualified_eslope(s::SiglentScope) =
    _query_str(s, ":TRIGger:QUALified:ESLope?")

"""
    set_trigger_qualified_esource!(s, source)

The command sets the edge trigger source in the qualified trigger.

`:TRIGger:QUALified:ESource <source>` (guide PDF p. 520)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_qualified_esource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:QUALified:ESource", source))

"""
    get_trigger_qualified_esource(s)

The query returns the current edge trigger source in the qualified trigger.

`:TRIGger:QUALified:ESource?` (guide PDF p. 520)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_qualified_esource(s::SiglentScope) =
    _query_str(s, ":TRIGger:QUALified:ESource?")

"""
    set_trigger_qualified_limit!(s, type)

The command sets the limit range type when the qualified type is “State with Delay” or “Edge with Delay” in the qualified trigger.

`:TRIGger:QUALified:LIMit <type>` (guide PDF p. 521)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_qualified_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:QUALified:LIMit", type))

"""
    get_trigger_qualified_limit(s)

The query returns the current limit range type in the qualified trigger.

`:TRIGger:QUALified:LIMit?` (guide PDF p. 521)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_qualified_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:QUALified:LIMit?")

"""
    set_trigger_qualified_qlevel!(s, level)

The command sets the level of the qualify source in the qualified trigger.

`:TRIGger:QUALified:QLEVel <level>` (guide PDF p. 522)

    <level>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_qualified_qlevel!(s::SiglentScope, level) =
    scpi_write(s, _cmd(":TRIGger:QUALified:QLEVel", level))

"""
    get_trigger_qualified_qlevel(s)

The query returns the current level of the qualify source in the qualified trigger.

`:TRIGger:QUALified:QLEVel?` (guide PDF p. 522)

Returns `Float64`.

Response format:

    <level>

    <level>:= Value in NR3 format.
"""
get_trigger_qualified_qlevel(s::SiglentScope) =
    _query_float(s, ":TRIGger:QUALified:QLEVel?")

"""
    set_trigger_qualified_qsource!(s, source)

The command sets the qualify source of the qualified trigger.

`:TRIGger:QUALified:QSource <source>` (guide PDF p. 523)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_qualified_qsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:QUALified:QSource", source))

"""
    get_trigger_qualified_qsource(s)

The query returns the current qualify source of the qualified trigger.

`:TRIGger:QUALified:QSource?` (guide PDF p. 523)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_qualified_qsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:QUALified:QSource?")

"""
    set_trigger_qualified_tlower!(s, value)

The command sets the limit lower value when the qualified type is “Edge with Delay” or “State with Delay” in the qualified trigger.

`:TRIGger:QUALified:TLOWer <value>` (guide PDF p. 524)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [2.00E-09,
    2.00E+01].

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:QUALified:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_qualified_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:QUALified:TLOWer", value))

"""
    get_trigger_qualified_tlower(s)

The query returns the current delay lower value in the qualified trigger.

`:TRIGger:QUALified:TLOWer?` (guide PDF p. 524)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_qualified_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:QUALified:TLOWer?")

"""
    set_trigger_qualified_tupper!(s, value)

The command sets limit upper value when the qualified type is “Edge with Delay” or “State with Delay” in the qualified trigger.

`:TRIGger:QUALified:TUPPer <value>` (guide PDF p. 525)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [3.00E-09,
    2.00E+01].

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:QUALified:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_qualified_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:QUALified:TUPPer", value))

"""
    get_trigger_qualified_tupper(s)

The query returns the current delay upper value in the qualified trigger.

`:TRIGger:QUALified:TUPPer?` (guide PDF p. 525)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_qualified_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:QUALified:TUPPer?")

"""
    set_trigger_qualified_type!(s, type, option=nothing)

The command sets the qualified type of the qualified trigger.

`:TRIGger:QUALified:TYPE <type>[,<option>]` (guide PDF p. 526)

    <type>:= {STATe|STATE_DLY|EDGE|EDGE_DLY}

    <option>:= {LOW|HIGH} when <type> is STATe or STATE_DLY
    <option>:= {RISing|FALLing} when <type> is EDGE or
    EDGE_DLY
"""
set_trigger_qualified_type!(s::SiglentScope, type, option=nothing) =
    scpi_write(s, _cmd(":TRIGger:QUALified:TYPE", type, option))

"""
    get_trigger_qualified_type(s)

The query returns the current qualified type of the qualified trigger.

`:TRIGger:QUALified:TYPE?` (guide PDF p. 526)

Returns `String`.

Response format:

    <type>[,<option>]

    <type>:= {STATe|STATE_DLY|EDGE|EDGE_DLY}

    <option>:= {LOW|HIGH} when <type> is STATe or STATE_DLY
    <option>:= {RISing|FALLing} when <type> is EDGE or
    EDGE_DLY
"""
get_trigger_qualified_type(s::SiglentScope) =
    _query_str(s, ":TRIGger:QUALified:TYPE?")

"""
    set_trigger_delay_coupling!(s, mode)

The command sets the coupling mode of the delay trigger.

`:TRIGger:DELay:COUPling <mode>` (guide PDF p. 528)

    <mode>:= {DC|AC|LFREJect|HFREJect}
    - DC coupling allows dc and ac signals into the trigger path.
    - AC coupling places a high-pass filter in the trigger path,
    removing dc offset voltage from the trigger waveform. Use
    AC coupling to get a stable edge trigger when your
    waveform has a large dc offset.
    - HFREJect which is a high-frequency rejection filter that
    adds a low-pass filter in the trigger path to remove
    high-frequency components from the trigger waveform.
    Use the high-frequency rejection filter to remove
    high-frequency noise, such as AM or FM broadcast
    stations, from the trigger path.
    - LFREJect which is a low frequency rejection filter adds a
    high-pass filter in series with the trigger waveform to
    remove any unwanted low-frequency components from a
    trigger waveform, such as power line frequencies, that can
    interfere with proper triggering.
"""
set_trigger_delay_coupling!(s::SiglentScope, mode) =
    scpi_write(s, _cmd(":TRIGger:DELay:COUPling", mode))

"""
    get_trigger_delay_coupling(s)

The query returns the current coupling mode of the delay trigger.

`:TRIGger:DELay:COUPling?` (guide PDF p. 528)

Returns `String`.

Response format:

    <mode>

    <mode>:= {DC|AC|LFREJect|HFREJect}
"""
get_trigger_delay_coupling(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:COUPling?")

"""
    set_trigger_delay_source!(s, state...)

The command sets the level state of trigger source A in the delay trigger.

`:TRIGger:DELay:SOURce <state>[...[,<state>]]` (guide PDF p. 529)

    <state>:= {X|L|H}
    - X means the "don't care" state.
    - H means the logic high state.
    - L means the logic low state.

    Note:
    Parameters are configured to corresponding sources in the
    order of C1-C<n>, D0-D15.
"""
set_trigger_delay_source!(s::SiglentScope, state...) =
    scpi_write(s, _cmd(":TRIGger:DELay:SOURce", state...))

"""
    get_trigger_delay_source(s)

The query returns the current level state of trigger source A in the delay trigger.

`:TRIGger:DELay:SOURce?` (guide PDF p. 529)

Returns `String`.

Response format:

    <state>

    <state>:= {X|L|H}
"""
get_trigger_delay_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:SOURce?")

"""
    set_trigger_delay_source2!(s, source)

The command sets the trigger source B in the delay trigger.

`:TRIGger:DELay:SOURce2 <source>` (guide PDF p. 530)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_delay_source2!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:DELay:SOURce2", source))

"""
    get_trigger_delay_source2(s)

The query returns the current trigger source B in the delay trigger.

`:TRIGger:DELay:SOURce2?` (guide PDF p. 530)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_delay_source2(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:SOURce2?")

"""
    set_trigger_delay_slope!(s, slope_type)

The command sets the slope of source A in the delay trigger.

`:TRIGger:DELay:SLOPe <slope_type>` (guide PDF p. 531)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_delay_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:DELay:SLOPe", slope_type))

"""
    get_trigger_delay_slope(s)

The query returns the slope of source A in the delay trigger.

`:TRIGger:DELay:SLOPe?` (guide PDF p. 531)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_delay_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:SLOPe?")

"""
    set_trigger_delay_slope2!(s, slope_type)

The command sets the slope of source B in the delay trigger.

`:TRIGger:DELay:SLOPe2 <slope_type>` (guide PDF p. 532)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_delay_slope2!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:DELay:SLOPe2", slope_type))

"""
    get_trigger_delay_slope2(s)

The query returns the slope of source B in the delay trigger.

`:TRIGger:DELay:SLOPe2?` (guide PDF p. 532)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_delay_slope2(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:SLOPe2?")

"""
    set_trigger_delay_level!(s, source, value)

The command sets the level of source A in the delay trigger.

`:TRIGger:DELay:LEVel <source>,<value>` (guide PDF p. 533)

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_delay_level!(s::SiglentScope, source, value) =
    scpi_write(s, _cmd(":TRIGger:DELay:LEVel", source, value))

"""
    get_trigger_delay_level(s, source)

The query returns the current trigger level of source A in the delay trigger.

`:TRIGger:DELay:LEVel? <source>` (guide PDF p. 533)

Returns `String`.

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

Response format:

    <source>,<value>

    <source>:= {C<x>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <value>:= Value in NR3 format.
"""
get_trigger_delay_level(s::SiglentScope, source) =
    _query_str(s, _cmd(":TRIGger:DELay:LEVel?", source))

"""
    set_trigger_delay_level2!(s, level_value)

The command sets the trigger level of source B in the delay trigger.

`:TRIGger:DELay:LEVel2 <level_value>` (guide PDF p. 534)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_delay_level2!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:DELay:LEVel2", level_value))

"""
    get_trigger_delay_level2(s)

The query returns the current trigger level of source B in the delay trigger.

`:TRIGger:DELay:LEVel2?` (guide PDF p. 534)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_trigger_delay_level2(s::SiglentScope) =
    _query_float(s, ":TRIGger:DELay:LEVel2?")

"""
    set_trigger_delay_limit!(s, type)

The command sets the limit range type of the delay trigger.

`:TRIGger:DELay:LIMit <type>` (guide PDF p. 535)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_delay_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:DELay:LIMit", type))

"""
    get_trigger_delay_limit(s)

The query returns the current limit range type of the delay trigger.

`:TRIGger:DELay:LIMit?` (guide PDF p. 535)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_delay_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:DELay:LIMit?")

"""
    set_trigger_delay_tupper!(s, value)

The command sets the limit upper value of the delay trigger limit type.

`:TRIGger:DELay:TUPPer <value>` (guide PDF p. 536)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [3.00E-09,
    2.00E+01].

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:DELay:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_delay_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:DELay:TUPPer", value))

"""
    get_trigger_delay_tupper(s)

The query returns the current limit upper value of the delay trigger limit type.

`:TRIGger:DELay:TUPPer?` (guide PDF p. 536)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_delay_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:DELay:TUPPer?")

"""
    set_trigger_delay_tlower!(s, value)

The command sets the limit lower value of the delay trigger limit type.

`:TRIGger:DELay:TLOWer <value>` (guide PDF p. 537)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [2.00E-09,
    2.00E+01].

    Note:
    • The lower value cannot be greater than the upper value
    using by the command :TRIGger:DELay:TUPPer.
    • The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_delay_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:DELay:TLOWer", value))

"""
    get_trigger_delay_tlower(s)

The query returns the current limit lower value of the delay trigger limit type.

`:TRIGger:DELay:TLOWer?` (guide PDF p. 537)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_delay_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:DELay:TLOWer?")

"""
    set_trigger_nedge_source!(s, source)

The command sets the trigger source of the Nth edge trigger.

`:TRIGger:NEDGe:SOURce <source>` (guide PDF p. 539)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_nedge_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:SOURce", source))

"""
    get_trigger_nedge_source(s)

The query returns the current trigger source of the Nth edge trigger.

`:TRIGger:NEDGe:SOURce?` (guide PDF p. 539)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_nedge_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:NEDGe:SOURce?")

"""
    set_trigger_nedge_slope!(s, slope_type)

The command sets the slope of the Nth edge trigger.

`:TRIGger:NEDGe:SLOPe <slope_type>` (guide PDF p. 540)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_nedge_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:SLOPe", slope_type))

"""
    get_trigger_nedge_slope(s)

The query returns the current slope setting of the Nth edge trigger.

`:TRIGger:NEDGe:SLOPe?` (guide PDF p. 540)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_nedge_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:NEDGe:SLOPe?")

"""
    set_trigger_nedge_idle!(s, value)

The command sets the idle time of the Nth edge trigger.

`:TRIGger:NEDGe:IDLE <value>` (guide PDF p. 541)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 2.00E+01]
"""
set_trigger_nedge_idle!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:IDLE", value))

"""
    get_trigger_nedge_idle(s)

The query returns the current idle time of the Nth edge trigger.

`:TRIGger:NEDGe:IDLE?` (guide PDF p. 541)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_nedge_idle(s::SiglentScope) =
    _query_float(s, ":TRIGger:NEDGe:IDLE?")

"""
    set_trigger_nedge_edge!(s, value)

This command sets the edge num of the Nth edge trigger.

`:TRIGger:NEDGe:EDGE <value>` (guide PDF p. 542)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 65535].
"""
set_trigger_nedge_edge!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:EDGE", value))

"""
    get_trigger_nedge_edge(s)

The query returns the current edge num of the Nth edge trigger.

`:TRIGger:NEDGe:EDGE?` (guide PDF p. 542)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_nedge_edge(s::SiglentScope) =
    _query_int(s, ":TRIGger:NEDGe:EDGE?")

"""
    set_trigger_nedge_level!(s, level_value)

The command sets the trigger level of the Nth edge trigger.

`:TRIGger:NEDGe:LEVel <level_value>` (guide PDF p. 543)

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_nedge_level!(s::SiglentScope, level_value) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:LEVel", level_value))

"""
    get_trigger_nedge_level(s)

The query returns the current trigger level value of the Nth edge trigger.

`:TRIGger:NEDGe:LEVel?` (guide PDF p. 543)

Returns `Float64`.

Response format:

    <level_value>

    <level_value>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.
"""
get_trigger_nedge_level(s::SiglentScope) =
    _query_float(s, ":TRIGger:NEDGe:LEVel?")

"""
    set_trigger_nedge_holdoff!(s, holdoff_type)

The command selects the holdoff type of the Nth edge trigger.

`:TRIGger:NEDGe:HOLDoff <holdoff_type>` (guide PDF p. 544)

    <holdoff_type>:= {OFF|EVENts|TIME}
    - OFF means to turn off the holdoff.
    - EVENts means the number of trigger events that the
    oscilloscope counts before re-arming the trigger circuitry.
    - TIME means the amount of time that the oscilloscope waits
    before re-arming the trigger circuitry.
"""
set_trigger_nedge_holdoff!(s::SiglentScope, holdoff_type) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:HOLDoff", holdoff_type))

"""
    get_trigger_nedge_holdoff(s)

The query returns the current holdoff type of the Nth edge trigger.

`:TRIGger:NEDGe:HOLDoff?` (guide PDF p. 544)

Returns `String`.

Response format:

    <holdoff_type>

    <holdoff_type>:= {OFF|EVENts|TIME}
"""
get_trigger_nedge_holdoff(s::SiglentScope) =
    _query_str(s, ":TRIGger:NEDGe:HOLDoff?")

"""
    set_trigger_nedge_hldtime!(s, value)

The command sets the holdoff time of the Nth edge trigger.

`:TRIGger:NEDGe:HLDTime <value>` (guide PDF p. 545)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS5000X
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SDS2000X HD
    [8.00E-09, 3.00E+01]
"""
set_trigger_nedge_hldtime!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:HLDTime", value))

"""
    get_trigger_nedge_hldtime(s)

The query returns the current holdoff time of the Nth edge trigger.

`:TRIGger:NEDGe:HLDTime?` (guide PDF p. 545)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_nedge_hldtime(s::SiglentScope) =
    _query_float(s, ":TRIGger:NEDGe:HLDTime?")

"""
    set_trigger_nedge_hldevent!(s, value)

This command sets the number of holdoff events of the Nth edge trigger.

`:TRIGger:NEDGe:HLDEVent <value>` (guide PDF p. 546)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 100000000].
"""
set_trigger_nedge_hldevent!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:HLDEVent", value))

"""
    get_trigger_nedge_hldevent(s)

The query returns the current number of holdoff events of the Nth edge trigger.

`:TRIGger:NEDGe:HLDEVent?` (guide PDF p. 546)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_nedge_hldevent(s::SiglentScope) =
    _query_int(s, ":TRIGger:NEDGe:HLDEVent?")

"""
    set_trigger_nedge_hstart!(s, start_holdoff)

The command defines the initial position of the Nth edge trigger holdoff.

`:TRIGger:NEDGe:HSTart <start_holdoff>` (guide PDF p. 547)

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
    - LAST_TRIG means the initial position of holdoff is the first
    time point satisfying the trigger condition.
    - ACQ_START means the initial position of holdoff is the
    time of the last trigger.
"""
set_trigger_nedge_hstart!(s::SiglentScope, start_holdoff) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:HSTart", start_holdoff))

"""
    get_trigger_nedge_hstart(s)

The query returns the initial position of the Nth edge trigger holdoff.

`:TRIGger:NEDGe:HSTart?` (guide PDF p. 547)

Returns `String`.

Response format:

    <start_holdoff>

    <start_holdoff>:= {LAST_TRIG|ACQ_START}
"""
get_trigger_nedge_hstart(s::SiglentScope) =
    _query_str(s, ":TRIGger:NEDGe:HSTart?")

"""
    set_trigger_nedge_nreject!(s, state)

The command sets the state of the noise rejection.

`:TRIGger:NEDGe:NREJect <state>` (guide PDF p. 548)

    <state>:= {OFF|ON}
"""
set_trigger_nedge_nreject!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:NEDGe:NREJect", state))

"""
    get_trigger_nedge_nreject(s)

The query returns the current state of the noise rejection.

`:TRIGger:NEDGe:NREJect?` (guide PDF p. 548)

Returns `Bool`.

Response format:

    <state>

    <state>:= {OFF|ON}
"""
get_trigger_nedge_nreject(s::SiglentScope) =
    _query_bool(s, ":TRIGger:NEDGe:NREJect?")

"""
    set_trigger_shold_type!(s, type)

The command sets the trigger type of the setup/hold trigger.

`:TRIGger:SHOLd:TYPE <type>` (guide PDF p. 550)

    <type>:= {SETup|HOLD}
"""
set_trigger_shold_type!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:TYPE", type))

"""
    get_trigger_shold_type(s)

The query returns the current the trigger type of the setup/hold trigger.

`:TRIGger:SHOLd:TYPE?` (guide PDF p. 550)

Returns `String`.

Response format:

    <slope>

    <slope>:= {SETup|HOLD}
"""
get_trigger_shold_type(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:TYPE?")

"""
    set_trigger_shold_csource!(s, source)

The command sets the clock source of the setup/hold trigger.

`:TRIGger:SHOLd:CSource <source>` (guide PDF p. 551)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_shold_csource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:CSource", source))

"""
    get_trigger_shold_csource(s)

The query returns the current clock source of the setup/hold trigger.

`:TRIGger:SHOLd:CSource?` (guide PDF p. 551)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_shold_csource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:CSource?")

"""
    set_trigger_shold_cthreshold!(s, value)

The command sets the threshold of clock source of the setup/hold trigger.

`:TRIGger:SHOLd:CTHReshold <value>` (guide PDF p. 552)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_shold_cthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:CTHReshold", value))

"""
    get_trigger_shold_cthreshold(s)

The query returns the current threshold of clock source of the setup/hold trigger.

`:TRIGger:SHOLd:CTHReshold?` (guide PDF p. 552)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_shold_cthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SHOLd:CTHReshold?")

"""
    set_trigger_shold_slope!(s, slope_type)

The command sets the clock slope of the setup/hold trigger.

`:TRIGger:SHOLd:SLOPe <slope_type>` (guide PDF p. 553)

    <slope_type>:= {RISing|FALLing}
"""
set_trigger_shold_slope!(s::SiglentScope, slope_type) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:SLOPe", slope_type))

"""
    get_trigger_shold_slope(s)

The query returns the current the clock slope of the setup/hold trigger.

`:TRIGger:SHOLd:SLOPe?` (guide PDF p. 553)

Returns `String`.

Response format:

    <slope_type>

    <slope_type>:= {RISing|FALLing}
"""
get_trigger_shold_slope(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:SLOPe?")

"""
    set_trigger_shold_dsource!(s, source)

The command sets the data source of the setup/hold trigger.

`:TRIGger:SHOLd:DSource <source>` (guide PDF p. 554)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_shold_dsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:DSource", source))

"""
    get_trigger_shold_dsource(s)

The query returns the current data source of the setup/hold trigger.

`:TRIGger:SHOLd:DSource?` (guide PDF p. 554)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_shold_dsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:DSource?")

"""
    set_trigger_shold_dthreshold!(s, value)

The command sets the threshold of data source of the setup/hold trigger.

`:TRIGger:SHOLd:DTHReshold <value>` (guide PDF p. 555)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_shold_dthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:DTHReshold", value))

"""
    get_trigger_shold_dthreshold(s)

The query returns the current threshold of data source of the setup/hold trigger.

`:TRIGger:SHOLd:DTHReshold?` (guide PDF p. 555)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_shold_dthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SHOLd:DTHReshold?")

"""
    set_trigger_shold_level!(s, level_state)

The command sets the level state of data source of the setup/hold trigger.

`:TRIGger:SHOLd:LEVel <level_state>` (guide PDF p. 556)

    <level_value>:= {LOW|HIGH}
"""
set_trigger_shold_level!(s::SiglentScope, level_state) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:LEVel", level_state))

"""
    get_trigger_shold_level(s)

The query returns the current level state of data source of the setup/hold trigger.

`:TRIGger:SHOLd:LEVel?` (guide PDF p. 556)

Returns `String`.

Response format:

    <level_state>

    <level_value>:= {LOW|HIGH}
"""
get_trigger_shold_level(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:LEVel?")

"""
    set_trigger_shold_limit!(s, type)

The command sets the limit range type of the setup/hold trigger.

`:TRIGger:SHOLd:LIMit <type>` (guide PDF p. 557)

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
set_trigger_shold_limit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:LIMit", type))

"""
    get_trigger_shold_limit(s)

The query returns the current limit range type of the setup/hold trigger.

`:TRIGger:SHOLd:LIMit?` (guide PDF p. 557)

Returns `String`.

Response format:

    <type>

    <type>:= {LESSthan|GREATerthan|INNer|OUTer}
"""
get_trigger_shold_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:SHOLd:LIMit?")

"""
    set_trigger_shold_tupper!(s, value)

The command sets the upper value of the setup/hold trigger limit type.

`:TRIGger:SHOLd:TUPPer <value>` (guide PDF p. 558)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [2.00E-09,
    2.00E+01].

    Note:
    • The upper value cannot be less than the lower value using
    by the command :TRIGger:SHOLd:TLOWer.
    • The command is not valid when the limit range type is
    GREATerthan.
"""
set_trigger_shold_tupper!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:TUPPer", value))

"""
    get_trigger_shold_tupper(s)

The query returns the current upper value of the setup/hold trigger limit type.

`:TRIGger:SHOLd:TUPPer?` (guide PDF p. 558)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_shold_tupper(s::SiglentScope) =
    _query_float(s, ":TRIGger:SHOLd:TUPPer?")

"""
    set_trigger_shold_tlower!(s, value)

The command sets the lower value of the setup/hold trigger limit type.

`:TRIGger:SHOLd:TLOWer <value>` (guide PDF p. 559)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2. The range of the value is [2.00E-09,
    2.00E+01].

    Note:
    The lower value cannot be greater than the upper value using
    by the command :TRIGger:SHOLd:TUPPer.
    The command is not valid when the limit range type is
    LESSthan.
"""
set_trigger_shold_tlower!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SHOLd:TLOWer", value))

"""
    get_trigger_shold_tlower(s)

The query returns the current lower value of the setup/hold trigger limit type.

`:TRIGger:SHOLd:TLOWer?` (guide PDF p. 559)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_shold_tlower(s::SiglentScope) =
    _query_float(s, ":TRIGger:SHOLd:TLOWer?")

"""
    set_trigger_iic_address!(s, addr)

The command sets the address of the IIC bus trigger.

`:TRIGger:IIC:ADDRess <addr>` (guide PDF p. 561)

    <addr>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 127].
"""
set_trigger_iic_address!(s::SiglentScope, addr) =
    scpi_write(s, _cmd(":TRIGger:IIC:ADDRess", addr))

"""
    get_trigger_iic_address(s)

The query returns the current address of the IIC bus trigger.

`:TRIGger:IIC:ADDRess?` (guide PDF p. 561)

Returns `Int`.

Response format:

    <addr>

    <addr>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iic_address(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIC:ADDRess?")

"""
    set_trigger_iic_alength!(s, length)

The command sets the length of address of the IIC bus trigger.

`:TRIGger:IIC:ALENgth <length>` (guide PDF p. 562)

    <length>:= {7BIT|10BIT}
"""
set_trigger_iic_alength!(s::SiglentScope, length) =
    scpi_write(s, _cmd(":TRIGger:IIC:ALENgth", length))

"""
    get_trigger_iic_alength(s)

The query returns the current length of address of the IIC bus trigger.

`:TRIGger:IIC:ALENgth?` (guide PDF p. 562)

Returns `String`.

Response format:

    <addr_length>

    <addr_length>:= {7BIT|10BIT}
"""
get_trigger_iic_alength(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:ALENgth?")

"""
    set_trigger_iic_condition!(s, condition)

The command sets the trigger condition of the IIC bus.

`:TRIGger:IIC:CONDition <condition>` (guide PDF p. 563)

    <condition>:=
    {STARt|STOP|RESTart|NACK|EEPRom|7ADDRess|10ADDRe
    ss|DLENgth}
"""
set_trigger_iic_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:IIC:CONDition", condition))

"""
    get_trigger_iic_condition(s)

The query returns the current trigger condition of the IIC bus.

`:TRIGger:IIC:CONDition?` (guide PDF p. 563)

Returns `String`.

Response format:

    <condition>

    <condition>:=
    {STARt|STOP|RESTart|NACK|EEPRom|7ADDRess|10ADDRe
    ss|DLENgth}
"""
get_trigger_iic_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:CONDition?")

"""
    set_trigger_iic_dat2!(s, data)

The command sets the data2 of the IIC bus trigger.

`:TRIGger:IIC:DAT2 <data>` (guide PDF p. 564)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data2 value.
"""
set_trigger_iic_dat2!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:IIC:DAT2", data))

"""
    get_trigger_iic_dat2(s)

The query returns the current data2 of the IIC bus trigger.

`:TRIGger:IIC:DAT2?` (guide PDF p. 564)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iic_dat2(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIC:DAT2?")

"""
    set_trigger_iic_data!(s, data)

The command sets the data of the IIC bus trigger.

`:TRIGger:IIC:DATA <data>` (guide PDF p. 565)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data value.
"""
set_trigger_iic_data!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:IIC:DATA", data))

"""
    get_trigger_iic_data(s)

The query returns the current data of the IIC bus trigger.

`:TRIGger:IIC:DATA?` (guide PDF p. 565)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iic_data(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIC:DATA?")

"""
    set_trigger_iic_dlength!(s, length)

The command sets the data length of the IIC bus trigger.

`:TRIGger:IIC:DLENgth <length>` (guide PDF p. 566)

    <length>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 12].
"""
set_trigger_iic_dlength!(s::SiglentScope, length) =
    scpi_write(s, _cmd(":TRIGger:IIC:DLENgth", length))

"""
    get_trigger_iic_dlength(s)

The query returns the current data length of the IIC bus trigger.

`:TRIGger:IIC:DLENgth?` (guide PDF p. 566)

Returns `Int`.

Response format:

    <length>

    <length>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iic_dlength(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIC:DLENgth?")

"""
    set_trigger_iic_limit!(s, limit_type)

The command sets the data comparison type when the trigger condition is EEPROM on the IIC bus trigger.

`:TRIGger:IIC:LIMit <limit_type>` (guide PDF p. 567)

    <limit_type>:= {EQUal|GREaterthan|LESSthan}
"""
set_trigger_iic_limit!(s::SiglentScope, limit_type) =
    scpi_write(s, _cmd(":TRIGger:IIC:LIMit", limit_type))

"""
    get_trigger_iic_limit(s)

The query returns the current the limit range type when the trigger condition is EEPROM.

`:TRIGger:IIC:LIMit?` (guide PDF p. 567)

Returns `String`.

Response format:

    <limit_type>

    <limit_type>:= {EQUal|GREaterthan|LESSthan}
"""
get_trigger_iic_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:LIMit?")

"""
    set_trigger_iic_rwbit!(s, type)

The command sets whether the trigger frame is read address or write address when the IIC trigger condition is 7 or 10 ADDR&DATA.

`:TRIGger:IIC:RWBit <type>` (guide PDF p. 568)

    <type>:= {WRITe|READ|ANY}
"""
set_trigger_iic_rwbit!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:IIC:RWBit", type))

"""
    get_trigger_iic_rwbit(s)

The query returns the current read write bit of the IIC bus trigger.

`:TRIGger:IIC:RWBit?` (guide PDF p. 568)

Returns `String`.

Response format:

    <type>

    <type>:= {WRITe|READ|ANY}
"""
get_trigger_iic_rwbit(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:RWBit?")

"""
    set_trigger_iic_sclsource!(s, source)

The command selects the SCL source of the IIC bus trigger.

`:TRIGger:IIC:SCLSource <source>` (guide PDF p. 569)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_iic_sclsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:IIC:SCLSource", source))

"""
    get_trigger_iic_sclsource(s)

This query returns the current SCL source of the IIC bus trigger.

`:TRIGger:IIC:SCLSource?` (guide PDF p. 569)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_iic_sclsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:SCLSource?")

"""
    set_trigger_iic_sclthreshold!(s, value)

The command sets the threshold of the SCL on IIC bus trigger.

`:TRIGger:IIC:SCLThreshold <value>` (guide PDF p. 570)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_iic_sclthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIC:SCLThreshold", value))

"""
    get_trigger_iic_sclthreshold(s)

This query returns the current threshold of the SCL on IIC bus trigger.

`:TRIGger:IIC:SCLThreshold?` (guide PDF p. 570)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_iic_sclthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:IIC:SCLThreshold?")

"""
    set_trigger_iic_sdasource!(s, source)

The command selects the SDA source of the IIC bus trigger.

`:TRIGger:IIC:SDASource <source>` (guide PDF p. 571)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_iic_sdasource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:IIC:SDASource", source))

"""
    get_trigger_iic_sdasource(s)

This query returns the current SDA source of the IIC bus trigger.

`:TRIGger:IIC:SDASource?` (guide PDF p. 571)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_iic_sdasource(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIC:SDASource?")

"""
    set_trigger_iic_sdathreshold!(s, value)

The command sets the threshold of the SDA on IIC bus trigger.

`:TRIGger:IIC:SDAThreshold <value>` (guide PDF p. 572)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_iic_sdathreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIC:SDAThreshold", value))

"""
    get_trigger_iic_sdathreshold(s)

This query returns the current threshold of the SDA on IIC bus trigger.

`:TRIGger:IIC:SDAThreshold?` (guide PDF p. 572)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_iic_sdathreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:IIC:SDAThreshold?")

"""
    set_trigger_spi_bitorder!(s, bit_order)

The command sets the bit order of the SPI bus trigger.

`:TRIGger:SPI:BITorder <bit_order>` (guide PDF p. 574)

    <bit_order>:= {LSM|MSB}
"""
set_trigger_spi_bitorder!(s::SiglentScope, bit_order) =
    scpi_write(s, _cmd(":TRIGger:SPI:BITorder", bit_order))

"""
    get_trigger_spi_bitorder(s)

The query returns the current bit order of the SPI bus trigger.

`:TRIGger:SPI:BITorder?` (guide PDF p. 574)

Returns `String`.

Response format:

    <bit_order>

    <bit_order>:= {LSM|MSB}
"""
get_trigger_spi_bitorder(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:BITorder?")

"""
    set_trigger_spi_clksource!(s, source)

The command selects the CLK source of the SPI bus trigger.

`:TRIGger:SPI:CLKSource <source>` (guide PDF p. 575)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_spi_clksource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SPI:CLKSource", source))

"""
    get_trigger_spi_clksource(s)

This query returns the current CLK source of the SPI bus trigger.

`:TRIGger:SPI:CLKSource?` (guide PDF p. 575)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_spi_clksource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:CLKSource?")

"""
    set_trigger_spi_clkthreshold!(s, clk_threshold)

The command sets the threshold of the CLK on SPI bus trigger.

`:TRIGger:SPI:CLKThreshold <clk_threshold>` (guide PDF p. 576)

    <clk_threshold>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_spi_clkthreshold!(s::SiglentScope, clk_threshold) =
    scpi_write(s, _cmd(":TRIGger:SPI:CLKThreshold", clk_threshold))

"""
    get_trigger_spi_clkthreshold(s)

This query returns the current threshold of the CLK on SPI bus trigger.

`:TRIGger:SPI:CLKThreshold?` (guide PDF p. 576)

Returns `Float64`.

Response format:

    <clk_threshold>

    <clk_threshold>:= Value in NR3 format.
"""
get_trigger_spi_clkthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SPI:CLKThreshold?")

"""
    set_trigger_spi_cssource!(s, source)

The command sets the CS source of the SPI bus trigger.

`:TRIGger:SPI:CSSource <source>` (guide PDF p. 577)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_spi_cssource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SPI:CSSource", source))

"""
    get_trigger_spi_cssource(s)

The query returns the current CS source of the SPI bus trigger.

`:TRIGger:SPI:CSSource?` (guide PDF p. 577)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}
    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_spi_cssource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:CSSource?")

"""
    set_trigger_spi_csthreshold!(s, threshold)

The command sets the threshold of the CS on SPI bus trigger.

`:TRIGger:SPI:CSThreshold <threshold>` (guide PDF p. 578)

    <threshold>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_spi_csthreshold!(s::SiglentScope, threshold) =
    scpi_write(s, _cmd(":TRIGger:SPI:CSThreshold", threshold))

"""
    get_trigger_spi_csthreshold(s)

This query returns the current threshold of the CS on SPI bus trigger.

`:TRIGger:SPI:CSThreshold?` (guide PDF p. 578)

Returns `Float64`.

Response format:

    <threshold>

    <threshold>:= Value in NR3 format.
"""
get_trigger_spi_csthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SPI:CSThreshold?")

"""
    set_trigger_spi_cstype!(s, type)

The command sets the chip selection type of the SPI bus trigger.

`:TRIGger:SPI:CSTYpe <type>` (guide PDF p. 579)

    <type>:= {NCS|CS|TIMeout[,<time>]}
    - CS means set to chip select state
    - NCS means set to non-chip select state
    - TIMeout indicates set to clock timeout status

    <time>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value is [1.00E-07, 5.00E-03].
"""
set_trigger_spi_cstype!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:SPI:CSTYpe", type))

"""
    get_trigger_spi_cstype(s)

This query returns the current chip selection type of the SPI bus trigger.

`:TRIGger:SPI:CSTYpe?` (guide PDF p. 579)

Returns `String`.

Response format:

    <type>

    <type>:= {NCS|CS|TIMeout[,<time>]}

    <time>:= Value in NR3 format.
"""
get_trigger_spi_cstype(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:CSTYpe?")

"""
    set_trigger_spi_data!(s, data...)

The command sets the data of the SPI bus trigger.

`:TRIGger:SPI:DATA <data>[,<data>[...[,<data>]]]` (guide PDF p. 580)

    <data>:= {0|1|X}

    Note:
    • The number of parameters should be consistent with the
    data length using by the
    command :TRIGger:SPI:DLENgth.
    • Parameters are assigned to each bit in order from high to
    low.
"""
set_trigger_spi_data!(s::SiglentScope, data...) =
    scpi_write(s, _cmd(":TRIGger:SPI:DATA", data...))

"""
    set_trigger_spi_dlength!(s, data_length)

The command sets the data length of the SPI bus trigger.

`:TRIGger:SPI:DLENgth <data_length>` (guide PDF p. 581)

    <data_length>:= Value in NR1 format, including an integer and
    no decimal point, like 1. The range of the value is [4, 96].
"""
set_trigger_spi_dlength!(s::SiglentScope, data_length) =
    scpi_write(s, _cmd(":TRIGger:SPI:DLENgth", data_length))

"""
    get_trigger_spi_dlength(s)

The query returns the current data length of the SPI bus trigger.

`:TRIGger:SPI:DLENgth?` (guide PDF p. 581)

Returns `Int`.

Response format:

    <data_length>

    <data_length>:= Value in NR1 format, including an integer and
    no decimal point, like 1.
"""
get_trigger_spi_dlength(s::SiglentScope) =
    _query_int(s, ":TRIGger:SPI:DLENgth?")

"""
    set_trigger_spi_latchedge!(s, slope)

The command selects the sampling edge of CLK on SPI bus trigger.

`:TRIGger:SPI:CLK:LATChedge <slope>` (guide PDF p. 582)

    <slope>:= {RISing|FALLing}
"""
set_trigger_spi_latchedge!(s::SiglentScope, slope) =
    scpi_write(s, _cmd(":TRIGger:SPI:LATChedge", slope))

"""
    get_trigger_spi_latchedge(s)

This query returns the sampling edge of CLK on SPI bus trigger.

`:TRIGger:SPI:LATC?` (guide PDF p. 582)

Returns `String`.

Response format:

    <slope>

    <slope>:= {RISing|FALLing}
"""
get_trigger_spi_latchedge(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:LATChedge?")

"""
    set_trigger_spi_misosource!(s, source)

The command selects the MISO source of the SPI bus trigger.

`:TRIGger:SPI:MISOSource <source>` (guide PDF p. 583)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_spi_misosource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SPI:MISOSource", source))

"""
    get_trigger_spi_misosource(s)

This query returns the current MISO source of the SPI bus trigger.

`:TRIGger:SPI:MISOSource?` (guide PDF p. 583)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_spi_misosource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:MISOSource?")

"""
    set_trigger_spi_misothreshold!(s, value)

The command sets the threshold of the MISO on SPI bus trigger.

`:TRIGger:SPI:MISOThreshold <value>` (guide PDF p. 584)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_spi_misothreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SPI:MISOThreshold", value))

"""
    get_trigger_spi_misothreshold(s)

This query returns the current threshold of the MISO on SPI bus trigger.

`:TRIGger:SPI:MISOThreshold?` (guide PDF p. 584)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_spi_misothreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SPI:MISOThreshold?")

"""
    set_trigger_spi_mosisource!(s, source)

The command selects the MOSI source of the SPI bus trigger.

`:TRIGger:SPI:MOSISource <source>` (guide PDF p. 585)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_spi_mosisource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SPI:MOSISource", source))

"""
    get_trigger_spi_mosisource(s)

This query returns the current MOSI source of the SPI bus trigger.

`:TRIGger:SPI:MOSISource?` (guide PDF p. 585)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_spi_mosisource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:MOSISource?")

"""
    set_trigger_spi_mosithreshold!(s, value)

The command sets the threshold of the MOSI on SPI bus trigger.

`:TRIGger:SPI:MOSIThreshold <value>` (guide PDF p. 586)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_spi_mosithreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SPI:MOSIThreshold", value))

"""
    get_trigger_spi_mosithreshold(s)

The query returns the current threshold of the MOSI on SPI bus trigger.

`:TRIGger:SPI:MOSIThreshold?` (guide PDF p. 586)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_spi_mosithreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SPI:MOSIThreshold?")

"""
    set_trigger_spi_ncssource!(s, source)

The command sets the NCS source of the SPI bus trigger.

`:TRIGger:SPI:NCSSource <source>` (guide PDF p. 587)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_spi_ncssource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:SPI:NCSSource", source))

"""
    get_trigger_spi_ncssource(s)

The query returns the current NCS source of the SPI bus trigger.

`:TRIGger:SPI:NCSSource?` (guide PDF p. 587)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_spi_ncssource(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:NCSSource?")

"""
    set_trigger_spi_ncsthreshold!(s, value)

The command sets the threshold of the NCS on SPI bus trigger.

`:TRIGger:SPI:NCSThreshold <value>` (guide PDF p. 588)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_spi_ncsthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:SPI:NCSThreshold", value))

"""
    get_trigger_spi_ncsthreshold(s)

This query returns the current threshold of the NCS on SPI bus trigger.

`:TRIGger:SPI:NCSThreshold?` (guide PDF p. 588)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_spi_ncsthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:SPI:NCSThreshold?")

"""
    set_trigger_spi_ttype!(s, trigger_type)

The command sets the trigger type of the SPI bus trigger.

`:TRIGger:SPI:TTYPe <trigger_type>` (guide PDF p. 589)

    <trigger_type>:= {MISO|MOSI}
"""
set_trigger_spi_ttype!(s::SiglentScope, trigger_type) =
    scpi_write(s, _cmd(":TRIGger:SPI:TTYPe", trigger_type))

"""
    get_trigger_spi_ttype(s)

The query returns the current trigger type of the SPI bus trigger.

`:TRIGger:SPI:TTYPe?` (guide PDF p. 589)

Returns `String`.

Response format:

    <trigger_type>

    <trigger_type>:= {MISO|MOSI}
"""
get_trigger_spi_ttype(s::SiglentScope) =
    _query_str(s, ":TRIGger:SPI:TTYPe?")

"""
    set_trigger_uart_baud!(s, baud)

The command sets the baud rate of the UART bus trigger.

`:TRIGger:UART:BAUD <baud>` (guide PDF p. 591)

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|38400
    bps|57600bps|115200bps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [300, 20000000].
"""
set_trigger_uart_baud!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:UART:BAUD", baud))

"""
    get_trigger_uart_baud(s)

The query returns the current baud rate of the UART bus trigger.

`:TRIGger:UART:BAUD?` (guide PDF p. 591)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|38400
    bps|57600bps|115200bps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_uart_baud(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:BAUD?")

"""
    set_trigger_uart_bitorder!(s, order)

The command sets the bit order of the UART trigger.

`:TRIGger:UART:BITorder <order>` (guide PDF p. 592)

    <order>:= {LSM|MSB}
"""
set_trigger_uart_bitorder!(s::SiglentScope, order) =
    scpi_write(s, _cmd(":TRIGger:UART:BITorder", order))

"""
    get_trigger_uart_bitorder(s)

The query returns the current bit order of the UART trigger.

`:TRIGger:UART:BITorder?` (guide PDF p. 592)

Returns `String`.

Response format:

    <order>

    <order>:= {LSM|MSB}
"""
get_trigger_uart_bitorder(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:BITorder?")

"""
    set_trigger_uart_condition!(s, condition)

The command sets the condition of the UART bus trigger.

`:TRIGger:UART:CONDition <condition>` (guide PDF p. 593)

    <condition>:= {STARt|STOP|DATA|ERRor}
"""
set_trigger_uart_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:UART:CONDition", condition))

"""
    get_trigger_uart_condition(s)

The query returns the current condition of the UART bus trigger.

`:TRIGger:UART:CONDition?` (guide PDF p. 593)

Returns `String`.

Response format:

    <condition>

    <condition>:= {STARt|STOP|DATA|ERRor}
"""
get_trigger_uart_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:CONDition?")

"""
    set_trigger_uart_data!(s, data)

The command sets the data of the UART bus trigger.

`:TRIGger:UART:DATA <data>` (guide PDF p. 594)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    • The range of the value is related to data length by using
    the command :TRIGger:UART:DLENgth.
    • Use the don’t care data (256, data length is 8) to ignore the
    data value.
"""
set_trigger_uart_data!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:UART:DATA", data))

"""
    get_trigger_uart_data(s)

The query returns the current data of the UART bus trigger.

`:TRIGger:UART:DATA?` (guide PDF p. 594)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_uart_data(s::SiglentScope) =
    _query_int(s, ":TRIGger:UART:DATA?")

"""
    set_trigger_uart_dlength!(s, value)

The command sets the data length of the UART bus trigger.

`:TRIGger:UART:DLENgth <value>` (guide PDF p. 595)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [5, 8].
"""
set_trigger_uart_dlength!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:UART:DLENgth", value))

"""
    get_trigger_uart_dlength(s)

The query returns the current data length of the UART bus trigger.

`:TRIGger:UART:DLENgth?` (guide PDF p. 595)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_uart_dlength(s::SiglentScope) =
    _query_int(s, ":TRIGger:UART:DLENgth?")

"""
    set_trigger_uart_idle!(s, idle)

The command sets the idle level of the UART bus trigger.

`:TRIGger:UART:IDLE <idle>` (guide PDF p. 596)

    <idle>:= {LOW|HIGH}
"""
set_trigger_uart_idle!(s::SiglentScope, idle) =
    scpi_write(s, _cmd(":TRIGger:UART:IDLE", idle))

"""
    get_trigger_uart_idle(s)

The query returns the current idle level of the UART bus trigger.

`:TRIGger:UART:IDLE?` (guide PDF p. 596)

Returns `String`.

Response format:

    <idle>

    <idle>:= {LOW|HIGH}
"""
get_trigger_uart_idle(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:IDLE?")

"""
    set_trigger_uart_limit!(s, limit_type)

The command sets the data comparison type of the UART bus trigger when the trigger condition is Data.

`:TRIGger:UART:LIMit <limit_type>` (guide PDF p. 597)

    <limit_type>:= {EQUal|GREaterthan|LESSthan}
"""
set_trigger_uart_limit!(s::SiglentScope, limit_type) =
    scpi_write(s, _cmd(":TRIGger:UART:LIMit", limit_type))

"""
    get_trigger_uart_limit(s)

The query returns the current data comparison type of the UART bus trigger.

`:TRIGger:UART:LIMit?` (guide PDF p. 597)

Returns `String`.

Response format:

    <limit_type>

    <limit_type>:= {EQUal|GREaterthan|LESSthan}
"""
get_trigger_uart_limit(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:LIMit?")

"""
    set_trigger_uart_parity!(s, parity)

The command sets the parity check of the UART bus trigger.

`:TRIGger:UART:PARity <parity>` (guide PDF p. 598)

    <parity>:= {NONE|ODD|EVEN|MARK|SPACe}
"""
set_trigger_uart_parity!(s::SiglentScope, parity) =
    scpi_write(s, _cmd(":TRIGger:UART:PARity", parity))

"""
    get_trigger_uart_parity(s)

The query returns the current parity check of the UART bus trigger.

`:TRIGger:UART:PARity?` (guide PDF p. 598)

Returns `String`.

Response format:

    <parity_check>

    <parity_check>:= {NONE|ODD|EVEN|MARK|SPACe}
"""
get_trigger_uart_parity(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:PARity?")

"""
    set_trigger_uart_rxsource!(s, source)

The command sets the RX source of the UART bus trigger.

`:TRIGger:UART:RXSource <source>` (guide PDF p. 599)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_uart_rxsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:UART:RXSource", source))

"""
    get_trigger_uart_rxsource(s)

The query returns the current RX source of the UART bus trigger.

`:TRIGger:UART:RXSource?` (guide PDF p. 599)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_uart_rxsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:RXSource?")

"""
    set_trigger_uart_rxthreshold!(s, value)

The command sets the threshold of RX on UART bus trigger.

`:TRIGger:UART:RXThreshold <value>` (guide PDF p. 600)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_uart_rxthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:UART:RXThreshold", value))

"""
    get_trigger_uart_rxthreshold(s)

The query returns the current threshold of RX on UART bus trigger.

`:TRIGger:UART:RXThreshold?` (guide PDF p. 600)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_uart_rxthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:UART:RXThreshold?")

"""
    set_trigger_uart_stop!(s, bit)

The command sets the length of the stop bit on UART bus trigger.

`:TRIGger:UART:STOP <bit>` (guide PDF p. 601)

    <bit>:= {1|1.5|2}
"""
set_trigger_uart_stop!(s::SiglentScope, bit) =
    scpi_write(s, _cmd(":TRIGger:UART:STOP", bit))

"""
    get_trigger_uart_stop(s)

The query returns the current length of the stop bit on UART bus trigger.

`:TRIGger:UART:STOP?` (guide PDF p. 601)

Returns `String`.

Response format:

    <bit>

    <bit>:= {1|1.5|2}
"""
get_trigger_uart_stop(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:STOP?")

"""
    set_trigger_uart_ttype!(s, trigger_type)

The command sets the trigger type of the UART bus trigger.

`:TRIGger:UART:TTYPe <trigger_type>` (guide PDF p. 602)

    <trigger_type>:= {RX|TX}
"""
set_trigger_uart_ttype!(s::SiglentScope, trigger_type) =
    scpi_write(s, _cmd(":TRIGger:UART:TTYPe", trigger_type))

"""
    get_trigger_uart_ttype(s)

The query returns the current trigger type of the UART bus trigger.

`:TRIGger:UART:TTYPe?` (guide PDF p. 602)

Returns `String`.

Response format:

    <trigger_type>

    <trigger_type>:= {RX|TX}
"""
get_trigger_uart_ttype(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:TTYPe?")

"""
    set_trigger_uart_txsource!(s, source)

The command sets the TX source of the UART bus trigger.

`:TRIGger:UART:TXSource <source>` (guide PDF p. 603)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_uart_txsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:UART:TXSource", source))

"""
    get_trigger_uart_txsource(s)

The query returns the current TX source of the UART bus trigger.

`:TRIGger:UART:TXSource?` (guide PDF p. 603)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_uart_txsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:UART:TXSource?")

"""
    set_trigger_uart_txthreshold!(s, value)

The command sets the threshold of TX on the UART bus trigger.

`:TRIGger:UART:TXThreshold <value>` (guide PDF p. 604)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_uart_txthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:UART:TXThreshold", value))

"""
    get_trigger_uart_txthreshold(s)

The query returns the current threshold of TX on the UART bus trigger.

`:TRIGger:UART:TXThreshold?` (guide PDF p. 604)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_uart_txthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:UART:TXThreshold?")

"""
    set_trigger_can_baud!(s, baud)

The command sets the baud rate of the CAN bus trigger.

The command query returns the baud rate of the CAN bus trigger.

`:TRIGger:CAN:BAUD <baud>` (guide PDF p. 606)

    <baud>:=
    {5kbps|10kbps|20kbps|50kbps|100kbps|125kbps|250kbps|500
    kbps|800kbps|1Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [5000, 1000000].
"""
set_trigger_can_baud!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:CAN:BAUD", baud))

"""
    get_trigger_can_baud(s)

The command sets the baud rate of the CAN bus trigger.

The command query returns the baud rate of the CAN bus trigger.

`:TRIGger:CAN:BAUD?` (guide PDF p. 606)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {5kbps|10kbps|20kbps|50kbps|100kbps|125kbps|250kbps|500
    kbps|800kbps|1Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_can_baud(s::SiglentScope) =
    _query_str(s, ":TRIGger:CAN:BAUD?")

"""
    set_trigger_can_condition!(s, condition)

The command sets the trigger condition for the CAN bus trigger.

`:TRIGger:CAN:CONDition <condition>` (guide PDF p. 607)

    <condition>:= {STARt|REMote|ID|ID_AND_DATA|ERRor}
"""
set_trigger_can_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:CAN:CONDition", condition))

"""
    get_trigger_can_condition(s)

The query returns the current trigger condition for the CAN bus trigger.

`:TRIGger:CAN:CONDition?` (guide PDF p. 607)

Returns `String`.

Response format:

    <condition>

    <condition>:= {STARt|REMote|ID|ID_AND_DATA|ERRor}
"""
get_trigger_can_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:CAN:CONDition?")

"""
    set_trigger_can_dat2!(s, data)

The command sets the data2 of the CAN bus trigger.

`:TRIGger:CAN:DAT2 <data>` (guide PDF p. 608)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data2 value.
"""
set_trigger_can_dat2!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:CAN:DAT2", data))

"""
    get_trigger_can_dat2(s)

The query returns the current data2 of the CAN bus trigger.

`:TRIGger:CAN:DAT2?` (guide PDF p. 608)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_can_dat2(s::SiglentScope) =
    _query_int(s, ":TRIGger:CAN:DAT2?")

"""
    set_trigger_can_data!(s, data)

The command sets the data of the CAN bus trigger.

`:TRIGger:CAN:DATA <data>` (guide PDF p. 609)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data value.
"""
set_trigger_can_data!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:CAN:DATA", data))

"""
    get_trigger_can_data(s)

The query returns the current data of the CAN bus trigger.

`:TRIGger:CAN:DATA?` (guide PDF p. 609)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_can_data(s::SiglentScope) =
    _query_int(s, ":TRIGger:CAN:DATA?")

"""
    set_trigger_can_id!(s, id)

The command sets the ID of the CAN bus trigger.

`:TRIGger:CAN:ID <id>` (guide PDF p. 610)

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    The range of the value is [0, 536870912] when the ID length is
    29 bits. The range of the value is [0, 2048] when the ID length
    is 11 bits.

    Note:
    Use the don’t care data (536870912, ID length is 29 bits) to
    ignore the ID value.
"""
set_trigger_can_id!(s::SiglentScope, id) =
    scpi_write(s, _cmd(":TRIGger:CAN:ID", id))

"""
    get_trigger_can_id(s)

The query returns the current ID of the CAN bus trigger.

`:TRIGger:CAN:ID?` (guide PDF p. 610)

Returns `Int`.

Response format:

    <id>

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_can_id(s::SiglentScope) =
    _query_int(s, ":TRIGger:CAN:ID?")

"""
    set_trigger_can_idlength!(s, id_length)

The command sets the ID length of the CAN bus trigger when the trigger condition is Remote, ID or ID+Data.

`:TRIGger:CAN:IDLENgth <id_length>` (guide PDF p. 611)

    <id_length>:= {11BITS|29BITS}
"""
set_trigger_can_idlength!(s::SiglentScope, id_length) =
    scpi_write(s, _cmd(":TRIGger:CAN:IDLength", id_length))

"""
    get_trigger_can_idlength(s)

The query returns the current ID length of the CAN bus trigger.

`:TRIGger:CAN:IDLENgth?` (guide PDF p. 611)

Returns `String`.

Response format:

    <id_length>

    <id_length>:= {11BITS|29BITS}
"""
get_trigger_can_idlength(s::SiglentScope) =
    _query_str(s, ":TRIGger:CAN:IDLength?")

"""
    set_trigger_can_source!(s, source)

The command selects the source of the CAN bus trigger.

`:TRIGger:CAN:SOURce <source>` (guide PDF p. 612)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_can_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:CAN:SOURce", source))

"""
    get_trigger_can_source(s)

The query returns the current source of the CAN bus trigger.

`:TRIGger:CAN:SOURce?` (guide PDF p. 612)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_can_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:CAN:SOURce?")

"""
    set_trigger_can_threshold!(s, value)

The command sets the threshold of the source on CAN bus trigger.

`:TRIGger:CAN:THReshold <value>` (guide PDF p. 613)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_can_threshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:CAN:THReshold", value))

"""
    get_trigger_can_threshold(s)

The query returns the current threshold of the source on CAN bus trigger.

`:TRIGger:CAN:THReshold?` (guide PDF p. 613)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.
"""
get_trigger_can_threshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:CAN:THReshold?")

"""
    set_trigger_lin_baud!(s, baud)

The command sets the baud rate of the LIN bus trigger.

`:TRIGger:LIN:BAUD <baud>` (guide PDF p. 615)

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|CUST
    om[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [300, 20000000].
"""
set_trigger_lin_baud!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:LIN:BAUD", baud))

"""
    get_trigger_lin_baud(s)

The query returns the current baud rate of the LIN bus trigger.

`:TRIGger:LIN:BAUD?` (guide PDF p. 615)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {600bps|1200bps|2400bps|4800bps|9600bps|19200bps|CUST
    om[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_baud(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:BAUD?")

"""
    set_trigger_lin_condition!(s, condition)

The command sets the trigger condition of the LIN bus.

`:TRIGger:LIN:CONDition <condition>` (guide PDF p. 616)

    <condition>:= {BReak|ID|ID_AND_DATA|DATA_ERROR}
"""
set_trigger_lin_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:LIN:CONDition", condition))

"""
    get_trigger_lin_condition(s)

The query returns the current trigger condition of the LIN bus.

`:TRIGger:LIN:CONDition?` (guide PDF p. 616)

Returns `String`.

Response format:

    <condition>

    <condition>:= {BReak|ID|ID_AND_DATA|DATA_ERROR}
"""
get_trigger_lin_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:CONDition?")

"""
    set_trigger_lin_dat2!(s, data)

The command sets the data2 of the LIN bus trigger when the trigger condition is ID+Data.

`:TRIGger:LIN:DAT2 <data>` (guide PDF p. 617)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data2 value.
"""
set_trigger_lin_dat2!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:LIN:DAT2", data))

"""
    get_trigger_lin_dat2(s)

The query returns the current data2 of the LIN bus trigger.

`:TRIGger:LIN:DAT2?` (guide PDF p. 617)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_dat2(s::SiglentScope) =
    _query_int(s, ":TRIGger:LIN:DAT2?")

"""
    set_trigger_lin_data!(s, data)

The command sets the data of the LIN bus trigger when the trigger condition is ID+Data.

`:TRIGger:LIN:DATA <data>` (guide PDF p. 618)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data value.
"""
set_trigger_lin_data!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:LIN:DATA", data))

"""
    get_trigger_lin_data(s)

The query returns the current data1 of the LIN bus trigger.

`:TRIGger:LIN:DATA?` (guide PDF p. 618)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_data(s::SiglentScope) =
    _query_int(s, ":TRIGger:LIN:DATA?")

"""
    set_trigger_lin_error_checksum!(s, state)

The command sets the checksum error state of the LIN bus trigger when the trigger condition is Error.

`:TRIGger:LIN:ERRor:CHECksum <state>` (guide PDF p. 619)

    <state>:= {0|1}
    - 0 means OFF
    - 1 means ON
"""
set_trigger_lin_error_checksum!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:LIN:ERRor:CHECksum", state))

"""
    get_trigger_lin_error_checksum(s)

The query returns the current checksum error state of the LIN bus trigger.

`:TRIGger:LIN:ERRor:CHECksum?` (guide PDF p. 619)

Returns `String`.

Response format:

    <state>

    <state>:= {0|1}
"""
get_trigger_lin_error_checksum(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:ERRor:CHECksum?")

"""
    set_trigger_lin_error_dlength!(s, length)

The command sets the data length of the error frame when the trigger condition is Error and the checksum error state is on.

`:TRIGger:LIN:DLENgth <length>` (guide PDF p. 620)

    <length>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1, 8].
"""
set_trigger_lin_error_dlength!(s::SiglentScope, length) =
    scpi_write(s, _cmd(":TRIGger:LIN:ERRor:DLENgth", length))

"""
    get_trigger_lin_error_dlength(s)

The query returns the current data length of the error frame on LIN bus.

`:TRIGger:LIN:DLENgth?` (guide PDF p. 620)

Returns `Int`.

Response format:

    <length>

    <length>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_error_dlength(s::SiglentScope) =
    _query_int(s, ":TRIGger:LIN:ERRor:DLENgth?")

"""
    set_trigger_lin_error_id!(s, id)

The command sets the error frame ID of the LIN bus when the trigger condition is Error and the checksum error state is on.

`:TRIGger:LIN:ERRor:ID <id>` (guide PDF p. 621)

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 63].
"""
set_trigger_lin_error_id!(s::SiglentScope, id) =
    scpi_write(s, _cmd(":TRIGger:LIN:ERRor:ID", id))

"""
    get_trigger_lin_error_id(s)

The query returns the current error frame ID of the LIN bus.

`:TRIGger:LIN:ERRor:ID?` (guide PDF p. 621)

Returns `Int`.

Response format:

    <id>

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_error_id(s::SiglentScope) =
    _query_int(s, ":TRIGger:LIN:ERRor:ID?")

"""
    set_trigger_lin_error_parity!(s, state)

The command sets the header parity error state of the LIN bus trigger when the trigger condition is Error.

`:TRIGger:LIN:ERRor:PARity <state>` (guide PDF p. 622)

    <state>:= {0|1}
    - 0 means OFF
    - 1 means ON
"""
set_trigger_lin_error_parity!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:LIN:ERRor:PARity", state))

"""
    get_trigger_lin_error_parity(s)

The query returns the header parity error state of the LIN bus trigger.

`:TRIGger:LIN:ERRor:PARity?` (guide PDF p. 622)

Returns `String`.

Response format:

    <state>

    <state>:= {0|1}
"""
get_trigger_lin_error_parity(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:ERRor:PARity?")

"""
    set_trigger_lin_error_sync!(s, state)

The command sets the sync byte error state of the LIN bus trigger.

`:TRIGger:LIN:ERRor:SYNC <state>` (guide PDF p. 623)

    <state>:= {0|1}
"""
set_trigger_lin_error_sync!(s::SiglentScope, state) =
    scpi_write(s, _cmd(":TRIGger:LIN:ERRor:SYNC", state))

"""
    get_trigger_lin_error_sync(s)

The query returns the current sync byte error state of the LIN bus trigger.

`:TRIGger:LIN:ERRor:SYNC?` (guide PDF p. 623)

Returns `String`.

Response format:

    <state>

    <state>:= {0|1}
"""
get_trigger_lin_error_sync(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:ERRor:SYNC?")

"""
    set_trigger_lin_id!(s, id)

The command sets the ID of the LIN bus when the trigger condition is ID.

`:TRIGger:LIN:ID <id>` (guide PDF p. 624)

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 64].

    Note:
    Use the don’t care data (64) to ignore the ID value.
"""
set_trigger_lin_id!(s::SiglentScope, id) =
    scpi_write(s, _cmd(":TRIGger:LIN:ID", id))

"""
    get_trigger_lin_id(s)

The query returns the current ID of the LIN bus trigger.

`:TRIGger:LIN:ID?` (guide PDF p. 624)

Returns `Int`.

Response format:

    <id>

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_lin_id(s::SiglentScope) =
    _query_int(s, ":TRIGger:LIN:ID?")

"""
    set_trigger_lin_source!(s, source)

The command selects the trigger source of the LIN bus.

`:TRIGger:LIN:Source <source>` (guide PDF p. 625)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_lin_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:LIN:SOURce", source))

"""
    get_trigger_lin_source(s)

The query returns the current trigger source of the LIN bus.

`:TRIGger:LIN:Source?` (guide PDF p. 625)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_lin_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:SOURce?")

"""
    set_trigger_lin_standard!(s, version)

The command sets the LIN protocol standard when the trigger condition is Error and the checksum error state is on.

`:TRIGger:LIN:STANdard <version>` (guide PDF p. 626)

    <version>:= {0|1}
    - 0 means Rev1.3
    - 1 means Rev2.x
"""
set_trigger_lin_standard!(s::SiglentScope, version) =
    scpi_write(s, _cmd(":TRIGger:LIN:STANdard", version))

"""
    get_trigger_lin_standard(s)

The query returns the current protocol standard of the LIN bus.

`:TRIGger:LIN:STANdard?` (guide PDF p. 626)

Returns `String`.

Response format:

    <version>

    <version>:= {0|1}
"""
get_trigger_lin_standard(s::SiglentScope) =
    _query_str(s, ":TRIGger:LIN:STANdard?")

"""
    set_trigger_lin_threshold!(s, value)

The command sets the threshold of the source on LIN bus trigger.

`:TRIGger:LIN:THReshold <value>` (guide PDF p. 627)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    SHS800X/SHS1000X
    [-4.5*vertical_scale-vertical_offset
    , 4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset
    , 4.1*vertical_scale-vertical_offset]
"""
set_trigger_lin_threshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:LIN:THReshold", value))

"""
    get_trigger_lin_threshold(s)

The query returns the current threshold of source on the LIN bus trigger.

`:TRIGger:LIN:THReshold?` (guide PDF p. 627)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_lin_threshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:LIN:THReshold?")

"""
    set_trigger_flexray_baud!(s, baud)

The command sets the baud rate of the Flexray bus trigger.

`:TRIGger:FLEXray:BAUD <baud>` (guide PDF p. 629)

    <baud>:= {2500kbps|5Mbps|10Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [1000000,
    20000000].
"""
set_trigger_flexray_baud!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:BAUD", baud))

"""
    get_trigger_flexray_baud(s)

The query returns the current baud rate of the Flexray bus trigger.

`:TRIGger:FLEXray:BAUD?` (guide PDF p. 629)

Returns `String`.

Response format:

    <baud>

    <baud>:= {2500kbps|5Mbps|10Mbps|CUSTom[,<value>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_flexray_baud(s::SiglentScope) =
    _query_str(s, ":TRIGger:FLEXray:BAUD?")

"""
    set_trigger_flexray_condition!(s, condition)

The command sets the trigger condition of FLEXray bus.

`:TRIGger:FLEXray:CONDition <condition>` (guide PDF p. 630)

    <condition>:= {TSS|FRAMe|SYMBol|ERRor}
"""
set_trigger_flexray_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:CONDition", condition))

"""
    get_trigger_flexray_condition(s)

The query returns the current trigger condition of FLEXray bus.

`:TRIGger:FLEXray:CONDition?` (guide PDF p. 630)

Returns `String`.

Response format:

    <condition>

    <condition>:= {TSS|FRAMe|SYMBol|ERRor}
"""
get_trigger_flexray_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:FLEXray:CONDition?")

"""
    set_trigger_flexray_frame_compare!(s, type)

The command sets the frame cycle compare type of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:COMPare <type>` (guide PDF p. 631)

    <type >:= {ANY|EQUal|GREaterthan|LESSthan}
"""
set_trigger_flexray_frame_compare!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:FRAMe:COMPare", type))

"""
    get_trigger_flexray_frame_compare(s)

The query returns the current frame cycle compare type of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:COMPare?` (guide PDF p. 631)

Returns `String`.

Response format:

    <type >

    <type >:= {ANY|EQUal|GREaterthan|LESSthan}
"""
get_trigger_flexray_frame_compare(s::SiglentScope) =
    _query_str(s, ":TRIGger:FLEXray:FRAMe:COMPare?")

"""
    set_trigger_flexray_frame_cycle!(s, cycle)

The command sets the frame cycle of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:CYCLe <cycle>` (guide PDF p. 632)

    <cycle>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 63].
"""
set_trigger_flexray_frame_cycle!(s::SiglentScope, cycle) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:FRAMe:CYCLe", cycle))

"""
    get_trigger_flexray_frame_cycle(s)

The query returns the current frame cycle of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:CYCLe?` (guide PDF p. 632)

Returns `Int`.

Response format:

    <cycle>

    <cycle>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_flexray_frame_cycle(s::SiglentScope) =
    _query_int(s, ":TRIGger:FLEXray:FRAMe:CYCLe?")

"""
    set_trigger_flexray_frame_id!(s, id)

The command sets the frame ID of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:ID <id>` (guide PDF p. 633)

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 2048].

    Note:
    Use the don’t care data (2048) to ignore the ID value.
"""
set_trigger_flexray_frame_id!(s::SiglentScope, id) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:FRAMe:ID", id))

"""
    get_trigger_flexray_frame_id(s)

The query returns the current frame ID of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:ID?` (guide PDF p. 633)

Returns `Int`.

Response format:

    <id>

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_flexray_frame_id(s::SiglentScope) =
    _query_int(s, ":TRIGger:FLEXray:FRAMe:ID?")

"""
    set_trigger_flexray_frame_repetition!(s, times)

The command sets the cycle repetition of FLEXray bus trigger when the cycle compare type is Equal

`:TRIGger:FLEXray:FRAMe:REPetition <times>` (guide PDF p. 634)

    <times>:= {1|2|4|8|16|32|64}
"""
set_trigger_flexray_frame_repetition!(s::SiglentScope, times) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:FRAMe:REPetition", times))

"""
    get_trigger_flexray_frame_repetition(s)

The query returns the current frame repetition of FLEXray bus trigger.

`:TRIGger:FLEXray:FRAMe:REPetition?` (guide PDF p. 634)

Returns `String`.

Response format:

    <times>

    <times>:= {1|2|4|8|16|32|64}
"""
get_trigger_flexray_frame_repetition(s::SiglentScope) =
    _query_str(s, ":TRIGger:FLEXray:FRAMe:REPetition?")

"""
    set_trigger_flexray_source!(s, source)

The command selects the source of FLEXray bus trigger.

`:TRIGger:FLEXray:Source <source>` (guide PDF p. 635)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_flexray_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:SOURce", source))

"""
    get_trigger_flexray_source(s)

The query returns the current source of FLEXray bus trigger.

`:TRIGger:FLEXray:Source?` (guide PDF p. 635)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_flexray_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:FLEXray:SOURce?")

"""
    set_trigger_flexray_threshold!(s, value)

The command sets the threshold of the source on FLEXray bus trigger.

`:TRIGger:FLEXray:THReshold <value>` (guide PDF p. 636)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_flexray_threshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:FLEXray:THReshold", value))

"""
    get_trigger_flexray_threshold(s)

The query returns the current threshold of the source on FLEXray bus trigger.

`:TRIGger:FLEXray:THReshold?` (guide PDF p. 636)

Returns `Float64`.

Response format:

    < value>

    < value>:= Value in NR3 format.
"""
get_trigger_flexray_threshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:FLEXray:THReshold?")

"""
    set_trigger_canfd_bauddata!(s, baud)

The command sets the data baud rate of the CAN FD bus trigger when the frame type is Both or CAN FD.

`:TRIGger:CANFd:BAUDData <baud>` (guide PDF p. 638)

    <baud>:=
    {500kbps|1Mbps|2Mbps|5Mbps|8Mbps|10Mbps|CUSTom[,<val
    ue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [100000,
    10000000].
"""
set_trigger_canfd_bauddata!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:CANFd:BAUDData", baud))

"""
    get_trigger_canfd_bauddata(s)

The query returns the current data baud rate of the CAN FD bus trigger.

`:TRIGger:CANFd:BAUDData?` (guide PDF p. 638)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {500kbps|1Mbps|2Mbps|5Mbps|8Mbps|10Mbps|CUSTom[,<val
    ue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_canfd_bauddata(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:BAUDData?")

"""
    set_trigger_canfd_baudnominal!(s, baud)

The command sets the nominal baud rate of the CAN FD bus trigger.

`:TRIGger:CANFd:BAUDNominal <baud>` (guide PDF p. 639)

    <baud>:=
    {10kbps|25kbps|50kbps|100kbps|250kbps|1Mbps|CUSTom[,<v
    alue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [10000,
    1000000].
"""
set_trigger_canfd_baudnominal!(s::SiglentScope, baud) =
    scpi_write(s, _cmd(":TRIGger:CANFd:BAUDNominal", baud))

"""
    get_trigger_canfd_baudnominal(s)

The query returns the current nominal baud rate of the CAN FD bus trigger.

`:TRIGger:CANFd:BAUDNominal?` (guide PDF p. 639)

Returns `String`.

Response format:

    <baud>

    <baud>:=
    {10kbps|25kbps|50kbps|100kbps|250kbps|1Mbps|CUSTom[,<v
    alue>]}

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_canfd_baudnominal(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:BAUDNominal?")

"""
    set_trigger_canfd_condition!(s, condition)

The command sets the trigger condition for the CAN FD bus trigger.

`:TRIGger:CANFd:CONDition <condition>` (guide PDF p. 640)

    <condition>:= {STARt|REMote|ID|ID_AND_DATA|ERRor}
"""
set_trigger_canfd_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:CANFd:CONDition", condition))

"""
    get_trigger_canfd_condition(s)

The query returns the current trigger condition for the CAN FD bus trigger.

`:TRIGger:CANFd:CONDition?` (guide PDF p. 640)

Returns `String`.

Response format:

    <condition>

    <condition>:= {STARt|REMote|ID|ID_AND_DATA|ERRor}
"""
get_trigger_canfd_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:CONDition?")

"""
    set_trigger_canfd_dat2!(s, data)

The command sets the data2 of the CAN FD bus when the trigger condition is ID+Data.

`:TRIGger:CANFd:DAT2 <data>` (guide PDF p. 641)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data2 value.
"""
set_trigger_canfd_dat2!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:CANFd:DAT2", data))

"""
    get_trigger_canfd_dat2(s)

The query returns the current data2 of the CAN FD bus trigger.

`:TRIGger:CANFd:DAT2?` (guide PDF p. 641)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_canfd_dat2(s::SiglentScope) =
    _query_int(s, ":TRIGger:CANFd:DAT2?")

"""
    set_trigger_canfd_data!(s, data)

The command the data of the CAN FD bus trigger.

`:TRIGger:CANFd:DATA <data>` (guide PDF p. 642)

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1. The range of the value is [0, 256].

    Note:
    Use the don’t care data (256) to ignore the data value.
"""
set_trigger_canfd_data!(s::SiglentScope, data) =
    scpi_write(s, _cmd(":TRIGger:CANFd:DATA", data))

"""
    get_trigger_canfd_data(s)

The query returns the current data of the CAN FD bus trigger.

`:TRIGger:CANFd:DATA?` (guide PDF p. 642)

Returns `Int`.

Response format:

    <data>

    <data>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_canfd_data(s::SiglentScope) =
    _query_int(s, ":TRIGger:CANFd:DATA?")

"""
    set_trigger_canfd_ftype!(s, frame_type)

This command sets the frame type of the CAN FD bus trigger.

`:TRIGger:CANFd:FTYPe <frame_type>` (guide PDF p. 643)

    <frame_type>:= {BOTH|CAN|CANFd}
"""
set_trigger_canfd_ftype!(s::SiglentScope, frame_type) =
    scpi_write(s, _cmd(":TRIGger:CANFd:FTYPe", frame_type))

"""
    get_trigger_canfd_ftype(s)

The query returns the current frame type of the CAN FD bus trigger.

`:TRIGger:CANFd:FTYPe?` (guide PDF p. 643)

Returns `String`.

Response format:

    <frame_type>

    <frame_type>:= {BOTH|CAN|CANFd}
"""
get_trigger_canfd_ftype(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:FTYPe?")

"""
    set_trigger_canfd_id!(s, id)

The command sets the ID of the CAN FD bus trigger when the trigger condition is Remote, ID or ID+Data.

`:TRIGger:CANFd:ID <id>` (guide PDF p. 644)

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    The range of the value is [0, 536870911] when the ID length is
    29 bits. The range of the value is [0, 2047] when the ID length
    is 11 bits.

    Note:
    Use the don’t care data (536870912, ID length is 29) to ignore
    the data value.
"""
set_trigger_canfd_id!(s::SiglentScope, id) =
    scpi_write(s, _cmd(":TRIGger:CANFd:ID", id))

"""
    get_trigger_canfd_id(s)

The query returns the current ID of the CAN FD bus trigger.

`:TRIGger:CANFd:ID?` (guide PDF p. 644)

Returns `Int`.

Response format:

    <id>

    <id>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_canfd_id(s::SiglentScope) =
    _query_int(s, ":TRIGger:CANFd:ID?")

"""
    set_trigger_canfd_idlength!(s, length)

The command sets the ID length of the CAN FD bus trigger.

`:TRIGger:CANFd:IDLENgth <length>` (guide PDF p. 645)

    <length>:= {11BITS|29BITS}
"""
set_trigger_canfd_idlength!(s::SiglentScope, length) =
    scpi_write(s, _cmd(":TRIGger:CANFd:IDLength", length))

"""
    get_trigger_canfd_idlength(s)

The query returns the current ID length of the CAN FD bus trigger.

`:TRIGger:CANFd:IDLENgth?` (guide PDF p. 645)

Returns `String`.

Response format:

    <length>

    <length>:= {11BITS|29BITS}
"""
get_trigger_canfd_idlength(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:IDLength?")

"""
    set_trigger_canfd_source!(s, source)

The command selects the source of the CAN FD bus trigger.

`:TRIGger:CANFd:SOURce <source>` (guide PDF p. 646)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_canfd_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:CANFd:SOURce", source))

"""
    get_trigger_canfd_source(s)

The query returns the current source of the CAN FD bus trigger.

`:TRIGger:CANFd:SOURce?` (guide PDF p. 646)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_canfd_source(s::SiglentScope) =
    _query_str(s, ":TRIGger:CANFd:SOURce?")

"""
    set_trigger_canfd_threshold!(s, threshold)

The command sets the threshold of the source on CAN FD bus triggering.

`:TRIGger:CANFd:THReshold <threshold>` (guide PDF p. 647)

    <threshold>:= Value in NR3 format, including a decimal point
    and exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_canfd_threshold!(s::SiglentScope, threshold) =
    scpi_write(s, _cmd(":TRIGger:CANFd:THReshold", threshold))

"""
    get_trigger_canfd_threshold(s)

The query returns the current threshold of the source on CAN FD bus triggering.

`:TRIGger:CANFd:THReshold?` (guide PDF p. 647)

Returns `Float64`.

Response format:

    <threshold>

    <threshold>:= Value in NR3 format.
"""
get_trigger_canfd_threshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:CANFd:THReshold?")

"""
    set_trigger_iis_avariant!(s, type)

The command sets the audio variant of the IIS bus trigger.

`:TRIGger:IIS:AVARiant <type>` (guide PDF p. 649)

    <type>:= {IIS|LJ|RJ}
"""
set_trigger_iis_avariant!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:IIS:AVARiant", type))

"""
    get_trigger_iis_avariant(s)

The query returns the current audio variant of the IIS bus trigger.

`:TRIGger:IIS:AVARiant?` (guide PDF p. 649)

Returns `String`.

Response format:

    <type>

    <type>:= {IIS|LJ|RJ}
"""
get_trigger_iis_avariant(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:AVARiant?")

"""
    set_trigger_iis_bclksource!(s, source)

The command selects the BCLK source of the IIS bus trigger.

`:TRIGger:IIS:BCLKSource <source>` (guide PDF p. 650)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_iis_bclksource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:IIS:BCLKSource", source))

"""
    get_trigger_iis_bclksource(s)

The query returns the current BCLK source of the IIS bus trigger.

`:TRIGger:IIS:BCLKSource?` (guide PDF p. 650)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_iis_bclksource(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:BCLKSource?")

"""
    set_trigger_iis_bclkthreshold!(s, value)

The command sets the threshold of the BCLK on LIN bus trigger.

`:TRIGger:IIS:BCLKThreshold <value>` (guide PDF p. 651)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_iis_bclkthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIS:BCLKThreshold", value))

"""
    get_trigger_iis_bclkthreshold(s)

The query returns the current threshold of the BCLK on LIN bus trigger.

`:TRIGger:IIS:BCLKThreshold?` (guide PDF p. 651)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_iis_bclkthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:IIS:BCLKThreshold?")

"""
    set_trigger_iis_bitorder!(s, order)

The command sets the bit order of the IIS bus trigger.

`:TRIGger:IIS:BITorder <order>` (guide PDF p. 652)

    <order>:= {LSM|MSB}
"""
set_trigger_iis_bitorder!(s::SiglentScope, order) =
    scpi_write(s, _cmd(":TRIGger:IIS:BITorder", order))

"""
    get_trigger_iis_bitorder(s)

The query returns the current bit order of the IIS bus trigger.

`:TRIGger:IIS:BITorder?` (guide PDF p. 652)

Returns `String`.

Response format:

    <order>

    <order>:= {LSM|MSB}
"""
get_trigger_iis_bitorder(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:BITorder?")

"""
    set_trigger_iis_channel!(s, channel)

The command sets the channel of the IIS bus trigger.

`:TRIGger:IIS:CHANnel <channel>` (guide PDF p. 653)

    <channel>:= {LEFT|RIGHT}
"""
set_trigger_iis_channel!(s::SiglentScope, channel) =
    scpi_write(s, _cmd(":TRIGger:IIS:CHANnel", channel))

"""
    get_trigger_iis_channel(s)

The query returns the current channel of the IIS bus trigger

`:TRIGger:IIS:CHANnel?` (guide PDF p. 653)

Returns `String`.

Response format:

    <channel>

    <channel>:= {LEFT|RIGHT}
"""
get_trigger_iis_channel(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:CHANnel?")

"""
    set_trigger_iis_compare!(s, type)

The command sets the data compare type of the IIS bus trigger.

`:TRIGger:IIS:COMPare <type>` (guide PDF p. 654)

    <type>:= {EQUal|GREaterthan|LESSthan}
"""
set_trigger_iis_compare!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":TRIGger:IIS:COMPare", type))

"""
    get_trigger_iis_compare(s)

The query returns the current data compare type of the IIS bus trigger.

`:TRIGger:IIS:COMPare?` (guide PDF p. 654)

Returns `String`.

Response format:

    <type>

    <type>:= {EQUal|GREaterthan|LESSthan}
"""
get_trigger_iis_compare(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:COMPare?")

"""
    set_trigger_iis_condition!(s, condition)

The command sets the trigger condition of the IIS bus.

`:TRIGger:IIS:CONDition <condition>` (guide PDF p. 655)

    <condition>:= {DATA|MUTE|CLIP|GLITch|RISing|FALLing}
"""
set_trigger_iis_condition!(s::SiglentScope, condition) =
    scpi_write(s, _cmd(":TRIGger:IIS:CONDition", condition))

"""
    get_trigger_iis_condition(s)

The query returns the current trigger condition of the IIS bus.

`:TRIGger:IIS:CONDition?` (guide PDF p. 655)

Returns `String`.

Response format:

    <condition>

    <condition>:= {DATA|MUTE|CLIP|GLITch|RISing|FALLing}
"""
get_trigger_iis_condition(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:CONDition?")

"""
    set_trigger_iis_dlength!(s, value)

The command sets the data bits of the IIS bus trigger.

`:TRIGger:IIS:DLENgth <value>` (guide PDF p. 656)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    The range of the value is related to the channel bits and the
    start bits. If the channel bits are 32 and the start bit is 2, the
    range is [1,30]
"""
set_trigger_iis_dlength!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIS:DLENgth", value))

"""
    get_trigger_iis_dlength(s)

The query returns the current data bits of the IIS bus trigger.

`:TRIGger:IIS:DLENgth?` (guide PDF p. 656)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iis_dlength(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIS:DLENgth?")

"""
    set_trigger_iis_dsource!(s, source)

The command selects the data source of the IIS bus trigger.

`:TRIGger:IIS:DSource <source>` (guide PDF p. 657)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_iis_dsource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:IIS:DSource", source))

"""
    get_trigger_iis_dsource(s)

The query returns the current data source of the IIS bus trigger

`:TRIGger:IIS:DSource?` (guide PDF p. 657)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_iis_dsource(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:DSource?")

"""
    set_trigger_iis_dthreshold!(s, value)

The command sets the threshold of the data source on IIS bus trigger.

`:TRIGger:IIS:DTHReshold <value>` (guide PDF p. 658)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_iis_dthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIS:DTHReshold", value))

"""
    get_trigger_iis_dthreshold(s)

The query returns the current threshold of the data source on IIS bus trigger.

`:TRIGger:IIS:DTHReshold?` (guide PDF p. 658)

Returns `String`.

Response format:

    <threshold>:= Value in NR3 format.
"""
get_trigger_iis_dthreshold(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:DTHReshold?")

"""
    set_trigger_iis_latchedge!(s, slope)

The command selects the sampling edge of BCLK on IIS bus trigger.

`:TRIGger:IIS:BCLK:EDGE <slope>` (guide PDF p. 659)

    <slope>:= {RISing|FALLing}
"""
set_trigger_iis_latchedge!(s::SiglentScope, slope) =
    scpi_write(s, _cmd(":TRIGger:IIS:LATChedge", slope))

"""
    get_trigger_iis_latchedge(s)

The query returns the sampling edge of BCLK on IIS bus trigger

`:TRIGger:IIS:BCLK:EDGE?` (guide PDF p. 659)

Returns `String`.

Response format:

    <slope>

    <slope>:= {RISing|FALLing}
"""
get_trigger_iis_latchedge(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:LATChedge?")

"""
    set_trigger_iis_lch!(s, level)

The command selects the level of the left channel on IIS bus trigger.

`:TRIGger:IIS:LCH <level>` (guide PDF p. 660)

    <level>:= {LOW|HIGH}
"""
set_trigger_iis_lch!(s::SiglentScope, level) =
    scpi_write(s, _cmd(":TRIGger:IIS:LCH", level))

"""
    get_trigger_iis_lch(s)

The query returns the current level of the left channel on IIS bus trigger.

`:TRIGger:IIS:LCH?` (guide PDF p. 660)

Returns `String`.

Response format:

    <level>

    <level>:= {LOW|HIGH}
"""
get_trigger_iis_lch(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:LCH?")

"""
    set_trigger_iis_value!(s, value)

The command sets the value of the IIS bus trigger.

`:TRIGger:IIS:VALue <value>` (guide PDF p. 661)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    • The range of the value is related to data length by using
    the command :TRIGger:IIS:DLENgth.
    • Use the don’t care data (256, data length is 8) to ignore the
    data value.
"""
set_trigger_iis_value!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIS:VALue", value))

"""
    get_trigger_iis_value(s)

The query returns the current value of the IIS bus trigger.

`:TRIGger:IIS:VALue?` (guide PDF p. 661)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_trigger_iis_value(s::SiglentScope) =
    _query_int(s, ":TRIGger:IIS:VALue?")

"""
    set_trigger_iis_wssource!(s, source)

The command selects the WS source of the IIS bus trigger.

`:TRIGger:IIS:WSSource <source>` (guide PDF p. 662)

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_trigger_iis_wssource!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":TRIGger:IIS:WSSource", source))

"""
    get_trigger_iis_wssource(s)

The query returns the current WS source of the IIS bus trigger.

`:TRIGger:IIS:WSSource?` (guide PDF p. 662)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|D<n>}

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <n>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
get_trigger_iis_wssource(s::SiglentScope) =
    _query_str(s, ":TRIGger:IIS:WSSource?")

"""
    set_trigger_iis_wsthreshold!(s, value)

The command sets the threshold of the WS on IIS bus trigger.

`:TRIGger:IIS:WSThreshold <value>` (guide PDF p. 663)

    <value>:= Value in NR3 format, including a decimal point and
    exponent, like 1.23E+2.

    The range of the value varies by model, see the table below for
    details.
    Model Value Range
    SDS6000 Pro
    SDS6000A
    SDS6000L
    [-4.5*vertical_scale-vertical_offset,
    4.5*vertical_scale-vertical_offset]
    SDS5000X
    SDS2000X Plus
    SDS2000X HD
    [-4.1*vertical_scale-vertical_offset,
    4.1*vertical_scale-vertical_offset]
"""
set_trigger_iis_wsthreshold!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":TRIGger:IIS:WSTHreshold", value))

"""
    get_trigger_iis_wsthreshold(s)

The query returns the current threshold of the WS on IIS bus trigger.

`:TRIGger:IIS:WSThreshold?` (guide PDF p. 663)

Returns `Float64`.

Response format:

    <value>

    <value>:= Value in NR3 format.
"""
get_trigger_iis_wsthreshold(s::SiglentScope) =
    _query_float(s, ":TRIGger:IIS:WSTHreshold?")

# ---------------------------------------------------------------------- #
# WAVeform commands                                                      #
# ---------------------------------------------------------------------- #

export set_waveform_source!, get_waveform_source, set_waveform_start!, get_waveform_start,
      set_waveform_interval!, get_waveform_interval, set_waveform_point!, get_waveform_point,
      get_waveform_maxpoint, set_waveform_width!, get_waveform_width, get_waveform_preamble,
      get_waveform_data, set_waveform_sequence!, get_waveform_sequence

"""
    set_waveform_source!(s, source)

The command specifies the source waveform to be transferred from the oscilloscope using the query :WAVeform:DATA?

`:WAVeform:SOURce <source>` (guide PDF p. 665)

    <source>:= {C<x>|F<x>|D<m>}
    - C denotes an analog input channel. For example, C1 is
    analog input 1.
    - F denotes a math function. For example, F1 is math function
    1. All operators including FFT.
    - D denotes a digital waveform. For example, D1 denotes
    digital input 1.

    <x>:= 1 to (# analog channels) in NR1 format, including an
    integer and no decimal point, like 1.

    <m>:= 0 to (# digital channels - 1) in NR1 format, including an
    integer and no decimal point, like 1.
"""
set_waveform_source!(s::SiglentScope, source) =
    scpi_write(s, _cmd(":WAVeform:SOURce", source))

"""
    get_waveform_source(s)

The query returns the source waveform to be transferred from the oscilloscope.

`:WAVeform:SOURce?` (guide PDF p. 665)

Returns `String`.

Response format:

    <source>

    <source>:= {C<x>|F<x>|D<m>}
"""
get_waveform_source(s::SiglentScope) =
    _query_str(s, ":WAVeform:SOURce?")

"""
    set_waveform_start!(s, value)

The command specifies the starting data point for waveform transfer using the query :WAVeform:DATA?.

`:WAVeform:STARt <value>` (guide PDF p. 666)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    The value range is related to the current waveform point and
    the value set by the command :WAVeform:POINt.
"""
set_waveform_start!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":WAVeform:STARt", value))

"""
    get_waveform_start(s)

The query returns the starting data point for waveform transfer.

`:WAVeform:STARt?` (guide PDF p. 666)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_waveform_start(s::SiglentScope) =
    _query_int(s, ":WAVeform:STARt?")

"""
    set_waveform_interval!(s, value)

The command sets the interval between data points for waveform transfer using the query :WAVeform:DATA?

`:WAVeform:INTerval <value>` (guide PDF p. 667)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    The value range is related to the values set by the
    command :WAVeform:POINt and :WAVeform:STARt.
"""
set_waveform_interval!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":WAVeform:INTerval", value))

"""
    get_waveform_interval(s)

The query returns the interval between data points for waveform transfer.

`:WAVeform:INTerval?` (guide PDF p. 667)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_waveform_interval(s::SiglentScope) =
    _query_int(s, ":WAVeform:INTerval?")

"""
    set_waveform_point!(s, value)

The command sets the number of waveform points to be transferred with the query :WAVeform:DATA?

`:WAVeform:POINt <value>` (guide PDF p. 668)

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    Note:
    The value range is related to the current waveform point.
"""
set_waveform_point!(s::SiglentScope, value) =
    scpi_write(s, _cmd(":WAVeform:POINt", value))

"""
    get_waveform_point(s)

The query returns the number of waveform points to be transferred.

`:WAVeform:POINt?` (guide PDF p. 668)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_waveform_point(s::SiglentScope) =
    _query_int(s, ":WAVeform:POINt?")

"""
    get_waveform_maxpoint(s)

The query returns the maximum points of one piece, when it needs to read the waveform data in pieces.

`:WAVeform:MAXPoint?` (guide PDF p. 669)

Returns `Int`.

Response format:

    <value>

    <value>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_waveform_maxpoint(s::SiglentScope) =
    _query_int(s, ":WAVeform:MAXPoint?")

"""
    set_waveform_width!(s, type)

The command sets the current output format for the transfer of waveform data.

`:WAVeform:WIDTh <type>` (guide PDF p. 670)

    <type>:= {BYTE|WORD}
    WORD formatted data transfers 16-bit data as two bytes, and
    the upper byte is transmitted first.
    BYTE formatted data is transferred as 8-bit bytes.

    Note:
    When the vertical resolution is set to 10 bit or the ADC bit is
    more than 8bit, it must to use the command to set to WORD
    before transferring waveform data.
"""
set_waveform_width!(s::SiglentScope, type) =
    scpi_write(s, _cmd(":WAVeform:WIDTh", type))

"""
    get_waveform_width(s)

The query returns the current output format for the transfer of waveform data.

`:WAVeform:WIDTh?` (guide PDF p. 670)

Returns `String`.

Response format:

    <type>

    <type>:= {BYTE|WORD}
"""
get_waveform_width(s::SiglentScope) =
    _query_str(s, ":WAVeform:WIDTh?")

"""
    get_waveform_preamble(s)

The query returns the parameters of the source using by the command :WAVeform:SOURce.

`:WAVeform:PREamble?` (guide PDF p. 671)

Returns `WaveformPreamble` (decoded; raw bytes in `.raw`).

Response format:

    <bin>

    <bin>:= binary data block headed " #9<9-Digits>”. See the
    table below for details.
"""
get_waveform_preamble(s::SiglentScope) =
    _query_preamble(s, ":WAVeform:PREamble?")

"""
    get_waveform_data(s)

The query returns the waveform data of the source using by the command :WAVeform:SOURce to be transferred from the oscilloscope.

`:WAVeform:DATA?` (guide PDF p. 674)

Returns `Vector{UInt8}` (binary block payload).

Response format:

    <wave_data>

    <wave_data>:=binary data block headed " #N<N-Digits>”
"""
get_waveform_data(s::SiglentScope) =
    scpi_query_block(s, ":WAVeform:DATA?")

"""
    set_waveform_sequence!(s, value1, value2)

This command is used to set the sequence waveform frame to be read. Valid only when sequnce is on.

`:WAVeform:SEQuence <value1>,<value2>` (guide PDF p. 680)

    <value1>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
    It sets the index of sequence frame to be transferred with the
    query :WAVeform:DATA?. When set to 0, all sequence frames
    are returned and the query :WAVeform:DATA? will transfer as
    much sequence frames as it can transfer at once.

    <value2>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
    It sets the start index of sequence frame to be transferred with
    the query :WAVeform:DATA? This value is valid when <value1>
    is set to 0.
    Due to the memory limitation, when the number of all frames
    exceeds the limit, it is necessary to read by slice through
    <value2>. The number of slices can be calculated by
    read_frames (0x90-0x93) and sum_frames(0x94-0x97) in the
    query :WAVeform:PREamble?

    Note:
    • When sequence is enabled, <value1> will be set to 1 by
    default;when sequence and history are enabled, <value1>
    will be set to the current frame by default. In other cases,
    <value1> is set to 4294967295 by default.
    • The value range is related to the current sequence number.
"""
set_waveform_sequence!(s::SiglentScope, value1, value2) =
    scpi_write(s, _cmd(":WAVeform:SEQuence", value1, value2))

"""
    get_waveform_sequence(s)

The query returns the index of sequence frame to be transferred.

`:WAVeform:SEQuence?` (guide PDF p. 680)

Returns `String`.

Response format:

    <value1>,<value2>

    <value1>:= Value in NR1 format, including an integer and no
    decimal point, like 1.

    <value2>:= Value in NR1 format, including an integer and no
    decimal point, like 1.
"""
get_waveform_sequence(s::SiglentScope) =
    _query_str(s, ":WAVeform:SEQuence?")

# ---------------------------------------------------------------------- #
# WGEN commands                                                          #
# ---------------------------------------------------------------------- #

export set_wgen_arbwave_index!, set_wgen_arbwave_name!, get_wgen_arbwave,
      set_wgen_basic_wave!, get_wgen_basic_wave, set_wgen_output!, get_wgen_output,
      get_wgen_storelist, set_wgen_sync!, get_wgen_sync, set_wgen_voltprt!, get_wgen_voltprt

"""
    set_wgen_arbwave_index!(s, index; channel=s.wgen_channel)

This command sets or gets the basic wave parameters.

`<channel>:ARbWaVe INDEX,<index>` (guide PDF p. 683)

    <channel>:={C1}, SAG and the built-in waveform generator
    only support one output channel.

    <index>:= the index of the arbitrary waveform from the table
    below.

    <name>:= the name of the arbitrary waveform from the table
    below.

    Note:
    This table is just an example, the index depends on the specific
    model. The “STL?” query can be used to get the accurate
    mapping relationship between the index and name.

    Index Name Index Name Index Name Index Name
    0 Sine 12 Logfall 24 Gmonopuls 36 Triang
    1 Noise 13 Logrise 25 Tripuls 37 Harris
    2 StairUp 14 Sqrt 26 Cardiac 38 Bartlett
    3 StairDn 15 Root3 27 Quake 39 Tan
    4 Stairud 16 X^2 28 Chirp 40 Cot
    5 Ppulse 17 X^3 29 Twotone 41 Sec
    6 Npulse 18 Sinc 30 Snr 42 Csc
    7 Trapezia 19 Gaussian 31 Hamming 43 Asin
    8 Upramp 20 Dlorentz 32 Hanning 44 Acos
    9 Dnramp 21 Haversine 33 Kaiser 45 Atan
    10 Exp_fall 22 Lorentz 34 Blackman 46 Acot
    11 Exp_rise 23 Gauspuls 35 Gausswin 47 Square
"""
set_wgen_arbwave_index!(s::SiglentScope, index; channel=s.wgen_channel) =
    scpi_write(s, _cmd("$(channel):ARbWaVe", "INDEX", index))

"""
    set_wgen_arbwave_name!(s, name; channel=s.wgen_channel)

This command sets or gets the basic wave parameters.

`<channel>:ARbWaVe NAME,<name>` (guide PDF p. 683)

    <channel>:={C1}, SAG and the built-in waveform generator
    only support one output channel.

    <index>:= the index of the arbitrary waveform from the table
    below.

    <name>:= the name of the arbitrary waveform from the table
    below.

    Note:
    This table is just an example, the index depends on the specific
    model. The “STL?” query can be used to get the accurate
    mapping relationship between the index and name.

    Index Name Index Name Index Name Index Name
    0 Sine 12 Logfall 24 Gmonopuls 36 Triang
    1 Noise 13 Logrise 25 Tripuls 37 Harris
    2 StairUp 14 Sqrt 26 Cardiac 38 Bartlett
    3 StairDn 15 Root3 27 Quake 39 Tan
    4 Stairud 16 X^2 28 Chirp 40 Cot
    5 Ppulse 17 X^3 29 Twotone 41 Sec
    6 Npulse 18 Sinc 30 Snr 42 Csc
    7 Trapezia 19 Gaussian 31 Hamming 43 Asin
    8 Upramp 20 Dlorentz 32 Hanning 44 Acos
    9 Dnramp 21 Haversine 33 Kaiser 45 Atan
    10 Exp_fall 22 Lorentz 34 Blackman 46 Acot
    11 Exp_rise 23 Gauspuls 35 Gausswin 47 Square
"""
set_wgen_arbwave_name!(s::SiglentScope, name; channel=s.wgen_channel) =
    scpi_write(s, _cmd("$(channel):ARbWaVe", "NAME", name))

"""
    get_wgen_arbwave(s; channel=s.wgen_channel)

This command sets or gets the basic wave parameters.

`<channel>:ARbWaVe?` (guide PDF p. 683)

Returns `String`.

    <channel>:= {C1}

Response format:

    <channel>:ARWV

    INDEX,<index>,NAME,<name>
"""
get_wgen_arbwave(s::SiglentScope; channel=s.wgen_channel) =
    _query_str(s, "$(channel):ARbWaVe?")

"""
    set_wgen_basic_wave!(s, parameter, value; channel=s.wgen_channel)

This command sets or gets the basic wave parameters.

`<channel>:BaSic_WaVe <parameter>,<value>` (guide PDF p. 685)

    <channel>:={C1}, SAG and the built-in waveform generator
    only support one output channel.

    <parameter>:= a parameter from the table below.

    <value>:= value of the corresponding parameter.

    Parameters Value Description
    WVTP <type>
    := {SINE, SQUARE, RAMP, PULSE, NOISE, ARB, DC,
    PRBS, IQ}. If the command doesn’t set basic waveform
    type, WVPT will be set to the current waveform.
    FRQ <frequency>
    := frequency. The unit is Hertz “Hz”. Refer to the data sheet
    for the range of valid values. Not valid when WVTP is
    NOISE or DC.
    PERI <period>
    := period. The unit is seconds "s". Refer to the data sheet
    for the range of valid values. Not valid when WVTP is
    NOISE or DC.
    AMP <amplitude>
    := amplitude. The unit is volts, peak-to-peak "Vpp". Refer to
    the data sheet for the range of valid values. Not valid when
    WVTP is NOISE or DC.
    OFST <offset> := offset. The unit is volts "V". Refer to the data sheet for
    the range of valid values. Not valid when WVTP is NOISE.
    SYM <symmetry> := {0 to 100}. Symmetry of RAMP. The unit is "%". Only
    settable when WVTP is RAMP.
    DUTY <duty>
    := {0 to 100}. Duty cycle. The unit is "%". Value depends on
    frequency. Only settable when WVTP is SQUARE or
    PULSE.
    STDEV <stdev>
    := standard deviation of NOISE. The unit is volts "V". Refer
    to the data sheet for the range of valid values. Only
    settable when WVTP is NOISE.
    MEAN <mean>
    := mean of NOISE. The unit is volts "V". Refer to the data
    sheet for the range of valid values. Only settable when
    WVTP is NOISE.
    ...
"""
set_wgen_basic_wave!(s::SiglentScope, parameter, value; channel=s.wgen_channel) =
    scpi_write(s, _cmd("$(channel):BaSic_WaVe", parameter, value))

"""
    get_wgen_basic_wave(s; channel=s.wgen_channel)

This command sets or gets the basic wave parameters.

`<channel>:BaSic_WaVe?` (guide PDF p. 685)

Returns `String`.

    <channel>:= {C1}

Response format:

    <channel>:BSWV <parameter>

    <parameter>:= All the parameters of the current basic
    waveform.
"""
get_wgen_basic_wave(s::SiglentScope; channel=s.wgen_channel) =
    _query_str(s, "$(channel):BaSic_WaVe?")

"""
    set_wgen_output!(s, state, load; channel=s.wgen_channel)

This command enables or disables the output port(s) at the front panel. The query returns “ON” or “OFF” and “LOAD”, “PLRT”, “RATIO” parameters.

`<channel>:OUTPut <state>,LOAD,<load>` (guide PDF p. 687)

    <channel>:= {C1}, SAG and the built-in waveform generator
    only support one output channel.

    <state>:= {ON|OFF}

    <load>:= {50|HZ}. The unit is ohm.
"""
set_wgen_output!(s::SiglentScope, state, load; channel=s.wgen_channel) =
    scpi_write(s, _cmd("$(channel):OUTPut", state, "LOAD", load))

"""
    get_wgen_output(s; channel=s.wgen_channel)

This command enables or disables the output port(s) at the front panel. The query returns “ON” or “OFF” and “LOAD”, “PLRT”, “RATIO” parameters.

`<channel>:OUTPut?` (guide PDF p. 687)

Returns `String`.

Response format:

    <channel>:OUTP <state>,LOAD,<load>,PLRT,<polarity>

    <state>:= {ON|OFF}

    <load>:= {50|HZ}

    <polarity>:= {NOR|INVT}, in which NOR refers to normal, and
    INVT refers to invert. SAG and the built-in waveform generator
    only support to set to NOR.
"""
get_wgen_output(s::SiglentScope; channel=s.wgen_channel) =
    _query_str(s, "$(channel):OUTPut?")

"""
    get_wgen_storelist(s, location=nothing)

This query is used to read the stored waveforms list with indexes and names. If the store unit is empty, the command will return “EMPTY” string.

`SToreList? [<location>]` (guide PDF p. 688)

Returns `String`.

    <location>:= {BUILDIN|USER}
"""
get_wgen_storelist(s::SiglentScope, location=nothing) =
    _query_str(s, _cmd("SToreList?", location))

"""
    set_wgen_sync!(s, state; channel=s.wgen_channel)

This command sets or gets the synchronization signal.

`<channel>:SYNC <state>` (guide PDF p. 691)

    <channel>:= {C1}, SAG and the built-in waveform generator
    only support one output channel.

    <state>:= {ON|OFF}
"""
set_wgen_sync!(s::SiglentScope, state; channel=s.wgen_channel) =
    scpi_write(s, _cmd("$(channel):SYNC", state))

"""
    get_wgen_sync(s; channel=s.wgen_channel)

This command sets or gets the synchronization signal.

`<channel>:SYNC?` (guide PDF p. 691)

Returns `String`.

    <channel>:= {C1}

Response format:

    <channel>:SYNC <state>,TYPE,<TYPE>

    <channel>:= {C1}

    <state>:= {ON|OFF}

    <TYPE>:={CH1}, SAG and the built-in waveform generator
    only support one output channel, so it can only be CH1.
"""
get_wgen_sync(s::SiglentScope; channel=s.wgen_channel) =
    _query_str(s, "$(channel):SYNC?")

"""
    set_wgen_voltprt!(s, state)

This commend sets or gets the state of over-voltage protection.

`VOLTPRT <state>` (guide PDF p. 691)

    <state>:= {ON|OFF}
"""
set_wgen_voltprt!(s::SiglentScope, state) =
    scpi_write(s, _cmd("VOLTPRT", state))

"""
    get_wgen_voltprt(s)

This commend sets or gets the state of over-voltage protection.

`VOLTPRT?` (guide PDF p. 691)

Returns `String`.

Response format:

    VOLTPRT <state>
"""
get_wgen_voltprt(s::SiglentScope) =
    _query_str(s, "VOLTPRT?")

end # module
