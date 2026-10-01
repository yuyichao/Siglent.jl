#

# Synthetic :WAVeform:PREamble? descriptor, laid out as in the guide's table.

function put_le!(buf, off0, x)
    b = reinterpret(UInt8, [htol(x)])
    buf[off0 + 1:off0 + length(b)] .= b
    return buf
end

function put_str!(buf, off0, str)
    b = codeunits(str)
    buf[off0 + 1:off0 + length(b)] .= b
    return buf
end

function make_preamble(; len=346, name="WAVEDESC", coupling=1, bwlimit=1, source=2)
    pre = zeros(UInt8, len)
    put_str!(pre, 0, name)
    put_str!(pre, 16, "WAVEACE")
    put_le!(pre, 32, Int16(1))
    put_le!(pre, 34, Int16(0))
    put_le!(pre, 36, Int32(346))
    put_le!(pre, 60, Int32(2000))
    put_str!(pre, 76, "Siglent SDS")
    put_le!(pre, 116, Int32(1000))
    put_le!(pre, 132, Int32(3))
    put_le!(pre, 136, Int32(2))
    put_le!(pre, 144, Int32(10))
    put_le!(pre, 148, Int32(500))
    put_le!(pre, 156, Float32(0.5))
    put_le!(pre, 160, Float32(-0.25))
    put_le!(pre, 164, Float32(30))
    put_le!(pre, 172, Int16(12))
    put_le!(pre, 174, Int16(4))
    put_le!(pre, 176, Float32(5e-10))
    put_le!(pre, 180, -1.5e-7)
    put_le!(pre, 324, Int16(12))
    put_le!(pre, 326, Int16(coupling))
    put_le!(pre, 328, Float32(10))
    put_le!(pre, 332, Int16(7))
    put_le!(pre, 334, Int16(bwlimit))
    put_le!(pre, 344, Int16(source))
    return pre
end
