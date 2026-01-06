"""
    SerialIO

`IO` wrapper around a PDV device's serial interface.
"""
mutable struct SerialIO <: IO
    device::Device
    open::Bool
end

"""
    serial_io(device; reset=false)

Create a `SerialIO` wrapper for the device. Set `reset=true` to reset the serial interface.
"""
function serial_io(device::Device; reset::Bool=false)
    reset && serial_reset!(device)
    return SerialIO(device, true)
end

"""
    open(SerialIO, device; reset=false)

Create a `SerialIO` wrapper for use with `open`/`do` blocks.
"""
function Base.open(::Type{SerialIO}, device::Device; reset::Bool=false)
    return serial_io(device; reset=reset)
end

"""
    open(f, SerialIO, device; reset=false)

Open a `SerialIO`, call `f(io)`, and ensure it is closed.
"""
function Base.open(f::Function, ::Type{SerialIO}, device::Device; reset::Bool=false)
    io = serial_io(device; reset=reset)
    try
        return f(io)
    finally
        close(io)
    end
end

"""
    isopen(io::SerialIO)

Return `true` if the serial wrapper and device are open.
"""
function Base.isopen(io::SerialIO)
    return io.open && isopen(io.device)
end

"""
    close(io::SerialIO)

Mark the `SerialIO` wrapper as closed.
"""
function Base.close(io::SerialIO)
    io.open = false
    return nothing
end

"""
    bytesavailable(io::SerialIO)

Return the number of bytes available to read.
"""
function Base.bytesavailable(io::SerialIO)
    return serial_get_numbytes(io.device)
end

"""
    read!(io::SerialIO, buf)

Blocking read into `buf`.
"""
function Base.read!(io::SerialIO, buf::AbstractVector{UInt8})
    tmp = Vector{UInt8}(undef, length(buf) + 1)
    nread = serial_read_blocking!(io.device, tmp, length(buf))
    copyto!(buf, 1, tmp, 1, min(nread, length(buf)))
    if nread < length(buf)
        fill!(view(buf, nread + 1:length(buf)), 0x00)
    end
    return buf
end

"""
    read(io::SerialIO, nb)

Blocking read of up to `nb` bytes.
"""
function Base.read(io::SerialIO, nb::Integer)
    nb < 0 && throw(ArgumentError("nb must be non-negative"))
    tmp = Vector{UInt8}(undef, nb + 1)
    nread = serial_read_blocking!(io.device, tmp, nb)
    nread < 0 && return UInt8[]
    buf = Vector{UInt8}(undef, nread)
    copyto!(buf, 1, tmp, 1, nread)
    return buf
end

"""
    readavailable(io::SerialIO)

Read all currently available bytes.
"""
function Base.readavailable(io::SerialIO)
    nbytes = bytesavailable(io)
    nbytes <= 0 && return UInt8[]
    buf = Vector{UInt8}(undef, nbytes + 1)
    nread = serial_read!(io.device, buf, nbytes)
    nread < length(buf) && resize!(buf, nread)
    return buf
end

"""
    write(io::SerialIO, data)

Write a byte vector or string to the serial interface.
"""
function Base.write(io::SerialIO, data::AbstractVector{UInt8})
    return serial_write(io.device, data)
end

function Base.write(io::SerialIO, data::AbstractString)
    return serial_write(io.device, data)
end

"""
    flush(io::SerialIO)

No-op for `SerialIO` (writes are immediate).
"""
function Base.flush(::SerialIO)
    return nothing
end

"""
    readstring(io::SerialIO; maxbytes=4096)

Read up to `maxbytes` and return a `String`.
"""
function readstring(io::SerialIO; maxbytes::Integer=4096)
    maxbytes < 0 && throw(ArgumentError("maxbytes must be non-negative"))
    data = read(io, maxbytes)
    return String(data)
end

"""
    readline(io::SerialIO; maxbytes=4096)

Read up to the current serial terminator (or `maxbytes`) and return a `String`.
"""
function readline(io::SerialIO; maxbytes::Integer=4096)
    maxbytes < 0 && throw(ArgumentError("maxbytes must be non-negative"))
    term = serial_term(io.device)
    if isempty(term)
        return readstring(io; maxbytes=maxbytes)
    end

    delim = UInt8(term[1])
    buf = Vector{UInt8}(undef, 0)
    while length(buf) < maxbytes
        chunk = readavailable(io)
        isempty(chunk) && continue
        for byte in chunk
            byte == delim && return String(buf)
            push!(buf, byte)
            length(buf) >= maxbytes && break
        end
    end
    return String(buf)
end
