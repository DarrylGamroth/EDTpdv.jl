"""
    Device

Wrapper around a `PdvDev` handle with ownership tracking.
"""
mutable struct Device <: PdvObject
    handle::LibEDTpdv.PdvDev
    owns::Bool
end

"""
    Device(handle; owns=false)

Wrap an existing `PdvDev` handle. Set `owns=true` to close on finalizer/`close`.
"""
function Device(handle::LibEDTpdv.PdvDev; owns::Bool=false)
    _check_ptr(handle, "PdvDev")
    obj = Device(handle, owns)
    _register_finalizer!(obj)
    return obj
end

"""
    open_device(dev_name=EDT_INTERFACE; unit=0, channel=0, verbose=true)

Open a PDV device with optional verbose logging.
"""
function open_device(dev_name::AbstractString=EDT_INTERFACE;
                     unit::Integer=0,
                     channel::Integer=0,
                     verbose::Bool=true)
    handle = LibEDTpdv.pdv_open_device(dev_name, unit, channel, verbose ? 1 : 0)
    _check_ptr(handle, "PdvDev")
    return Device(handle; owns=true)
end

"""
    open_channel(dev_name=EDT_INTERFACE; unit=0, channel=0)

Open a specific channel on a PDV device.
"""
function open_channel(dev_name::AbstractString=EDT_INTERFACE;
                      unit::Integer=0,
                      channel::Integer=0)
    handle = LibEDTpdv.pdv_open_channel(dev_name, unit, channel)
    _check_ptr(handle, "PdvDev")
    return Device(handle; owns=true)
end

"""
    open_unit(dev_name=EDT_INTERFACE; unit=0)

Open a PDV device by unit number.
"""
function open_unit(dev_name::AbstractString=EDT_INTERFACE; unit::Integer=0)
    handle = LibEDTpdv.pdv_open(dev_name, unit)
    _check_ptr(handle, "PdvDev")
    return Device(handle; owns=true)
end

"""
    open(Device; dev_name=EDT_INTERFACE, unit=0, channel=0, verbose=true, mode=:channel)

Open a `Device` using `:channel`, `:device`, or `:unit` mode.
"""
function Base.open(::Type{Device};
                   dev_name::AbstractString=EDT_INTERFACE,
                   unit::Integer=0,
                   channel::Integer=0,
                   verbose::Bool=true,
                   mode::Symbol=:channel)
    if mode === :channel
        return open_channel(dev_name; unit=unit, channel=channel)
    elseif mode === :device
        return open_device(dev_name; unit=unit, channel=channel, verbose=verbose)
    elseif mode === :unit
        return open_unit(dev_name; unit=unit)
    end
    throw(ArgumentError("unsupported mode: $mode"))
end

"""
    open(f, Device; kwargs...)

Open a `Device`, call `f(dev)`, and ensure it is closed.
"""
function Base.open(f::Function, ::Type{Device}; kwargs...)
    dev = open(Device; kwargs...)
    try
        return f(dev)
    finally
        close(dev)
    end
end

"""
    flush_fifo!(device)

Flush device FIFOs and reset acquisition state.
"""
function flush_fifo!(device::Device)
    LibEDTpdv.pdv_flush_fifo(device.handle)
    return nothing
end

"""
    multibuf!(device, numbufs)

Allocate ring buffers for acquisition; returns driver status code.
"""
function multibuf!(device::Device, numbufs::Integer)
    return Int(LibEDTpdv.pdv_multibuf(device.handle, numbufs))
end

"""
    set_timeout!(device, value)

Set acquisition timeout in milliseconds; `0` blocks forever, `-1` uses auto timeouts.
"""
function set_timeout!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_timeout(device.handle, value))
end

"""
    get_timeout(device)

Return the current acquisition timeout in milliseconds.
"""
function get_timeout(device::Device)
    return Int(LibEDTpdv.pdv_get_timeout(device.handle))
end

"""
    setup_continuous!(device)

Enable continuous acquisition mode.
"""
function setup_continuous!(device::Device)
    LibEDTpdv.pdv_setup_continuous(device.handle)
    return nothing
end

"""
    stop_continuous!(device)

Disable continuous acquisition mode.
"""
function stop_continuous!(device::Device)
    LibEDTpdv.pdv_stop_continuous(device.handle)
    return nothing
end

"""
    in_continuous(device)

Return `true` if continuous acquisition is enabled.
"""
function in_continuous(device::Device)
    return LibEDTpdv.pdv_in_continuous(device.handle) != 0
end

"""
    force_single(device)

Return `true` if the device forces single-image mode.
"""
function force_single(device::Device)
    return LibEDTpdv.pdv_get_force_single(device.handle) != 0
end

"""
    set_fval_done!(device, enable)

Enable/disable `fval_done` behavior.
"""
function set_fval_done!(device::Device, enable::Bool)
    LibEDTpdv.pdv_set_fval_done(device.handle, enable ? 1 : 0)
    return nothing
end

"""
    get_fval_done(device)

Return `true` if `fval_done` is enabled.
"""
function get_fval_done(device::Device)
    return LibEDTpdv.pdv_get_fval_done(device.handle) != 0
end

"""
    start_images!(device, count)

Start acquisition of `count` images.
"""
function start_images!(device::Device, count::Integer)
    LibEDTpdv.pdv_start_images(device.handle, count)
    return nothing
end

"""
    wait_images_ptr(device, count=1)

Wait for `count` images and return a pointer to the last image buffer.
"""
function wait_images_ptr(device::Device, count::Integer=1)
    return LibEDTpdv.pdv_wait_images(device.handle, count)
end

"""
    get_last_image_ptr(device; raw=false)

Return a pointer to the most recently acquired image (non-blocking).
"""
function get_last_image_ptr(device::Device; raw::Bool=false)
    if raw
        return LibEDTpdv.pdv_get_last_image_raw(device.handle)
    end
    return LibEDTpdv.pdv_get_last_image(device.handle)
end

"""
    get_last_image(device; copy=false, raw=false)

Return the last image as an `UnsafeArray` view or a copied `Vector{UInt8}`.
"""
function get_last_image(device::Device; copy::Bool=false, raw::Bool=false)
    ptr = get_last_image_ptr(device; raw=raw)
    ptr == C_NULL && return nothing
    nbytes = image_bytes(device)
    if copy
        data = Vector{UInt8}(undef, nbytes)
        unsafe_copyto!(pointer(data), ptr, nbytes)
        return data
    end
    return UnsafeArray(ptr, (nbytes,); own=false)
end

"""
    get_last_image_raw(device; copy=false)

Variant of `get_last_image` that skips de-interleaving.
"""
function get_last_image_raw(device::Device; copy::Bool=false)
    return get_last_image(device; copy=copy, raw=true)
end

"""
    image_bytes(device)

Return the size of one image in bytes based on width/height/depth.
"""
function image_bytes(device::Device)
    return width(device) * height(device) * bytes_per_pixel(device)
end

"""
    bytes_per_pixel(device)

Return the minimum byte width for the current bit depth.
"""
function bytes_per_pixel(device::Device)
    return bits_to_bytes(depth(device))
end

"""
    wait_image(device, count=1)

Wait for images and return a copied `Vector{UInt8}` of the last image.
"""
function wait_image(device::Device, count::Integer=1)
    ptr = wait_images_ptr(device, count)
    ptr == C_NULL && return UInt8[]
    nbytes = image_bytes(device)
    data = Vector{UInt8}(undef, nbytes)
    unsafe_copyto!(pointer(data), ptr, nbytes)
    return data
end

"""
    wait_image!(device, dest, count=1)

Wait for images and copy the last image into `dest`.
"""
function wait_image!(device::Device, dest::Vector{UInt8}, count::Integer=1)
    ptr = wait_images_ptr(device, count)
    ptr == C_NULL && return dest
    nbytes = image_bytes(device)
    if length(dest) < nbytes
        resize!(dest, nbytes)
    end
    unsafe_copyto!(pointer(dest), ptr, nbytes)
    return dest
end

"""
    timeouts(device)

Return the number of acquisition timeouts since opening the device.
"""
function timeouts(device::Device)
    return Int(LibEDTpdv.pdv_timeouts(device.handle))
end

"""
    timeout_restart!(device; restart=true)

Recover after a timeout; returns number of buffers left undone.
"""
function timeout_restart!(device::Device; restart::Bool=true)
    return Int(LibEDTpdv.pdv_timeout_restart(device.handle, restart ? 1 : 0))
end

"""
    overrun(device)

Return `true` if the last acquisition overran.
"""
function overrun(device::Device)
    return LibEDTpdv.pdv_overrun(device.handle) != 0
end

"""
    lines_xferred(device)

Return the number of lines transferred for the current frame.
"""
function lines_xferred(device::Device)
    return Int(LibEDTpdv.pdv_get_lines_xferred(device.handle))
end

"""
    width_xferred(device)

Return the number of pixels transferred for the current line.
"""
function width_xferred(device::Device)
    return Int(LibEDTpdv.pdv_get_width_xferred(device.handle))
end

"""
    cl_fval_counter(device)

Return the Camera Link FVAL counter.
"""
function cl_fval_counter(device::Device)
    return Int(LibEDTpdv.pdv_cl_get_fval_counter(device.handle))
end

"""
    reset_cl_fval_counter!(device)

Reset the Camera Link FVAL counter.
"""
function reset_cl_fval_counter!(device::Device)
    LibEDTpdv.pdv_cl_reset_fval_counter(device.handle)
    return nothing
end

"""
    check_framesync(device, image)

Check for frame sync in `image`; returns `(status, framecount)`.
"""
function check_framesync(device::Device, image::Vector{UInt8})
    framecnt = Ref{UInt32}(0)
    result = 0
    GC.@preserve image begin
        result = LibEDTpdv.pdv_check_framesync(device.handle, pointer(image), framecnt)
    end
    return (Int(result), Int(framecnt[]))
end

"""
    enable_framesync!(device, mode)

Enable frame sync with the given mode.
"""
function enable_framesync!(device::Device, mode::Integer)
    return Int(LibEDTpdv.pdv_enable_framesync(device.handle, mode))
end

"""
    framesync_mode(device)

Return the current frame sync mode.
"""
function framesync_mode(device::Device)
    return Int(LibEDTpdv.pdv_framesync_mode(device.handle))
end

"""
    htaps_vtaps(device)

Return `(htaps, vtaps)` tap configuration.
"""
function htaps_vtaps(device::Device)
    htaps = Ref{Cint}(0)
    vtaps = Ref{Cint}(0)
    LibEDTpdv.pdv_get_htaps_vtaps(device.handle, htaps, vtaps)
    return (Int(htaps[]), Int(vtaps[]))
end

"""
    hskip_vskip(device)

Return `(hskip, vskip)` region-of-interest offsets.
"""
function hskip_vskip(device::Device)
    hskip = Ref{Cint}(0)
    vskip = Ref{Cint}(0)
    LibEDTpdv.pdv_get_hskip_vskip(device.handle, hskip, vskip)
    return (Int(hskip[]), Int(vskip[]))
end

"""
    hactv_vactv(device)

Return `(hactv, vactv)` region-of-interest sizes.
"""
function hactv_vactv(device::Device)
    hactv = Ref{Cint}(0)
    vactv = Ref{Cint}(0)
    LibEDTpdv.pdv_get_hactv_vactv(device.handle, hactv, vactv)
    return (Int(hactv[]), Int(vactv[]))
end

"""
    fv_once(device)

Return the frame-valid once setting.
"""
function fv_once(device::Device)
    return Int(LibEDTpdv.pdv_get_fv_once(device.handle))
end

"""
    width!(device, value)

Set the configured image width.
"""
function width!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_width(device.handle, value))
end

"""
    width(device)

Return the configured image width.
"""
function width(device::Device)
    return Int(LibEDTpdv.pdv_get_width(device.handle))
end

"""
    pitch(device)

Return the line pitch in bytes.
"""
function pitch(device::Device)
    return Int(LibEDTpdv.pdv_get_pitch(device.handle))
end

"""
    height!(device, value)

Set the configured image height.
"""
function height!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_height(device.handle, value))
end

"""
    height(device)

Return the configured image height.
"""
function height(device::Device)
    return Int(LibEDTpdv.pdv_get_height(device.handle))
end

"""
    frame_height(device)

Return the total frame height including any blanking.
"""
function frame_height(device::Device)
    return Int(LibEDTpdv.pdv_get_frame_height(device.handle))
end

"""
    depth(device)

Return the configured bit depth.
"""
function depth(device::Device)
    return Int(LibEDTpdv.pdv_get_depth(device.handle))
end

"""
    camera_type(device)

Return the camera type string from the config.
"""
function camera_type(device::Device)
    ptr = LibEDTpdv.pdv_get_camera_type(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    camera_class(device)

Return the camera class string from the config.
"""
function camera_class(device::Device)
    ptr = LibEDTpdv.pdv_get_camera_class(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    camera_model(device)

Return the camera model string from the config.
"""
function camera_model(device::Device)
    ptr = LibEDTpdv.pdv_get_camera_model(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    camera_info(device)

Return the camera info string from the config.
"""
function camera_info(device::Device)
    ptr = LibEDTpdv.pdv_get_camera_info(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    cfgname(device)

Return the active configuration file name.
"""
function cfgname(device::Device)
    ptr = LibEDTpdv.pdv_get_cfgname(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    exposure!(device, value)

Set camera exposure.
"""
function exposure!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_exposure(device.handle, value))
end

"""
    exposure(device)

Return camera exposure.
"""
function exposure(device::Device)
    return Int(LibEDTpdv.pdv_get_exposure(device.handle))
end

"""
    gain!(device, value)

Set camera gain.
"""
function gain!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_gain(device.handle, value))
end

"""
    gain(device)

Return camera gain.
"""
function gain(device::Device)
    return Int(LibEDTpdv.pdv_get_gain(device.handle))
end

"""
    blacklevel!(device, value)

Set camera black level.
"""
function blacklevel!(device::Device, value::Integer)
    return Int(LibEDTpdv.pdv_set_blacklevel(device.handle, value))
end

"""
    blacklevel(device)

Return camera black level.
"""
function blacklevel(device::Device)
    return Int(LibEDTpdv.pdv_get_blacklevel(device.handle))
end

"""
    enable_roi!(device, flag)

Enable or disable ROI mode.
"""
function enable_roi!(device::Device, flag::Bool)
    return Int(LibEDTpdv.pdv_enable_roi(device.handle, flag ? 1 : 0))
end

"""
    roi_enabled(device)

Return `true` if ROI mode is enabled.
"""
function roi_enabled(device::Device)
    return LibEDTpdv.pdv_get_roi_enabled(device.handle) != 0
end

"""
    set_roi!(device, hskip, hactv, vskip, vactv)

Set ROI offsets and sizes.
"""
function set_roi!(device::Device, hskip::Integer, hactv::Integer, vskip::Integer, vactv::Integer)
    return Int(LibEDTpdv.pdv_set_roi(device.handle, hskip, hactv, vskip, vactv))
end

"""
    auto_set_roi!(device)

Auto-detect and set ROI.
"""
function auto_set_roi!(device::Device)
    return Int(LibEDTpdv.pdv_auto_set_roi(device.handle))
end

"""
    frame_period!(device, rate, method)

Set frame period parameters.
"""
function frame_period!(device::Device, rate::Integer, method::Integer)
    return Int(LibEDTpdv.pdv_set_frame_period(device.handle, rate, method))
end

"""
    frame_period(device)

Return the current frame period.
"""
function frame_period(device::Device)
    return Int(LibEDTpdv.pdv_get_frame_period(device.handle))
end

"""
    start_delay!(device, delay_ms)

Set acquisition start delay in milliseconds.
"""
function start_delay!(device::Device, delay_ms::Integer)
    LibEDTpdv.pdv_set_start_delay(device.handle, delay_ms)
    return nothing
end

"""
    start_delay(device)

Return the acquisition start delay in milliseconds.
"""
function start_delay(device::Device)
    return Int(LibEDTpdv.pdv_get_start_delay(device.handle))
end

"""
    invert!(device, value)

Enable or disable image inversion.
"""
function invert!(device::Device, value::Bool)
    LibEDTpdv.pdv_set_invert(device.handle, value ? 1 : 0)
    return nothing
end

"""
    invert(device)

Return `true` if image inversion is enabled.
"""
function invert(device::Device)
    return LibEDTpdv.pdv_get_invert(device.handle) != 0
end

"""
    header_type!(device, header_type; irig_offset=0, irig_raw=false)

Set the header type and IRIG options.
"""
function header_type!(device::Device, header_type::LibEDTpdv.EdtPdvHeaderType, irig_offset::Integer=0, irig_raw::Bool=false)
    return Int(LibEDTpdv.pdv_set_header_type(device.handle, header_type, irig_offset, irig_raw ? 1 : 0))
end

"""
    header_type(device)

Return the current header type.
"""
function header_type(device::Device)
    return LibEDTpdv.pdv_get_header_type(device.handle)
end

"""
    header_size!(device, header_size)

Set the header size in bytes.
"""
function header_size!(device::Device, header_size::Integer)
    LibEDTpdv.pdv_set_header_size(device.handle, header_size)
    return nothing
end

"""
    header_size(device)

Return the header size in bytes.
"""
function header_size(device::Device)
    return Int(LibEDTpdv.pdv_get_header_size(device.handle))
end

"""
    header_position!(device, header_position)

Set header position relative to image data.
"""
function header_position!(device::Device, header_position::LibEDTpdv.EdtPdvHeaderPosition)
    LibEDTpdv.pdv_set_header_position(device.handle, header_position)
    return nothing
end

"""
    header_position(device)

Return the current header position.
"""
function header_position(device::Device)
    return LibEDTpdv.pdv_get_header_position(device.handle)
end

"""
    camera_connected(device)

Return `true` if a Camera Link camera is detected.
"""
function camera_connected(device::Device)
    return LibEDTpdv.pdv_cl_camera_connected(device.handle) != 0
end

"""
    is_cameralink(device)

Return `true` if the device is Camera Link.
"""
function is_cameralink(device::Device)
    return LibEDTpdv.pdv_is_cameralink(device.handle) != 0
end

"""
    is_simulator(device)

Return `true` if the device is running in simulator mode.
"""
function is_simulator(device::Device)
    return LibEDTpdv.pdv_is_simulator(device.handle) != 0
end

"""
    serial_set_baud!(device, baud)

Set the serial baud rate.
"""
function serial_set_baud!(device::Device, baud::Integer)
    return Int(LibEDTpdv.pdv_serial_set_baud(device.handle, baud))
end

"""
    serial_get_baud(device)

Return the current serial baud rate.
"""
function serial_get_baud(device::Device)
    return Int(LibEDTpdv.pdv_serial_get_baud(device.handle))
end

"""
    serial_command(device, cmd)

Send an ASCII serial command.
"""
function serial_command(device::Device, cmd::AbstractString)
    return Int(LibEDTpdv.pdv_serial_command(device.handle, cmd))
end

"""
    serial_binary_command(device, cmd)

Send a binary serial command.
"""
function serial_binary_command(device::Device, cmd::Vector{UInt8})
    GC.@preserve cmd begin
        return Int(LibEDTpdv.pdv_serial_binary_command(device.handle, pointer(cmd), length(cmd)))
    end
end

"""
    serial_write_available(device)

Return available bytes in the serial write buffer.
"""
function serial_write_available(device::Device)
    return Int(LibEDTpdv.pdv_serial_write_available(device.handle))
end

"""
    serial_write(device, buf)

Write a string or byte vector to the serial port.
"""
function serial_write(device::Device, buf::AbstractString)
    return Int(LibEDTpdv.pdv_serial_write(device.handle, buf, sizeof(buf)))
end

function serial_write(device::Device, buf::Vector{UInt8})
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_write(device.handle, pointer(buf), length(buf)))
    end
end

"""
    serial_read!(device, buf, size=length(buf)-1)

Read up to `size` bytes into `buf` (requires `size + 1` capacity).
"""
function serial_read!(device::Device, buf::Vector{UInt8}, size::Integer=length(buf) - 1)
    size < 0 && throw(ArgumentError("size must be non-negative"))
    length(buf) <= size && throw(ArgumentError("buf must be at least size + 1 bytes"))
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_read(device.handle, pointer(buf), size))
    end
end

"""
    serial_read_blocking!(device, buf, size=length(buf)-1)

Blocking read into `buf` (requires `size + 1` capacity).
"""
function serial_read_blocking!(device::Device, buf::Vector{UInt8}, size::Integer=length(buf) - 1)
    size < 0 && throw(ArgumentError("size must be non-negative"))
    length(buf) <= size && throw(ArgumentError("buf must be at least size + 1 bytes"))
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_read_blocking(device.handle, pointer(buf), size))
    end
end

"""
    serial_read_nullterm!(device, buf, size=length(buf)-1; nullterm=false)

Read into `buf` and optionally force a NUL terminator.
"""
function serial_read_nullterm!(device::Device, buf::Vector{UInt8}, size::Integer=length(buf) - 1, nullterm::Bool=false)
    size < 0 && throw(ArgumentError("size must be non-negative"))
    length(buf) <= size && throw(ArgumentError("buf must be at least size + 1 bytes"))
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_read_nullterm(device.handle, pointer(buf), size, nullterm ? 1 : 0))
    end
end

"""
    serial_read_enable(device)

Enable serial reads; returns driver status code.
"""
function serial_read_enable(device::Device)
    return Int(LibEDTpdv.pdv_serial_read_enable(device.handle))
end

"""
    serial_read_disable(device)

Disable serial reads; returns driver status code.
"""
function serial_read_disable(device::Device)
    return Int(LibEDTpdv.pdv_serial_read_disable(device.handle))
end

"""
    serial_set_delimiters!(device, prefix, term)

Set serial prefix and terminator strings.
"""
function serial_set_delimiters!(device::Device, prefix::AbstractString, term::AbstractString)
    LibEDTpdv.pdv_serial_set_delimiters(device.handle, prefix, term)
    return nothing
end

"""
    serial_prefix(device)

Return the current serial prefix string.
"""
function serial_prefix(device::Device)
    ptr = LibEDTpdv.pdv_serial_prefix(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    serial_term(device)

Return the current serial terminator string.
"""
function serial_term(device::Device)
    ptr = LibEDTpdv.pdv_serial_term(device.handle)
    ptr == C_NULL && return ""
    return unsafe_string(ptr)
end

"""
    serial_send_break!(device)

Send a serial break signal.
"""
function serial_send_break!(device::Device)
    LibEDTpdv.pdv_serial_send_break(device.handle)
    return nothing
end

"""
    serial_reset!(device)

Reset the serial interface state.
"""
function serial_reset!(device::Device)
    LibEDTpdv.pdv_serial_reset(device.handle)
    return nothing
end

"""
    serial_read_basler!(device, buf)

Read a Basler binary frame into `buf`.
"""
function serial_read_basler!(device::Device, buf::Vector{UInt8})
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_read_basler(device.handle, pointer(buf), length(buf)))
    end
end

"""
    serial_write_basler(device, buf)

Write a Basler binary frame from `buf`.
"""
function serial_write_basler(device::Device, buf::Vector{UInt8})
    GC.@preserve buf begin
        return Int(LibEDTpdv.pdv_serial_write_basler(device.handle, pointer(buf), length(buf)))
    end
end

"""
    serial_read_duncan!(device, frame)

Read a Duncantech frame into `frame`.
"""
function serial_read_duncan!(device::Device, frame::Vector{UInt8})
    GC.@preserve frame begin
        return Int(LibEDTpdv.pdv_serial_read_duncan(device.handle, pointer(frame)))
    end
end

"""
    serial_write_duncan(device, cmdbuf)

Write a Duncantech frame from `cmdbuf`.
"""
function serial_write_duncan(device::Device, cmdbuf::Vector{UInt8})
    GC.@preserve cmdbuf begin
        return Int(LibEDTpdv.pdv_serial_write_duncan(device.handle, pointer(cmdbuf), length(cmdbuf)))
    end
end

"""
    serial_wait(device, msecs, count)

Wait for serial data; returns number of bytes read or status code.
"""
function serial_wait(device::Device, msecs::Integer, count::Integer)
    return Int(LibEDTpdv.pdv_serial_wait(device.handle, msecs, count))
end

"""
    serial_wait_next(device, msecs, count)

Wait for next serial data; returns number of bytes read or status code.
"""
function serial_wait_next(device::Device, msecs::Integer, count::Integer)
    return Int(LibEDTpdv.pdv_serial_wait_next(device.handle, msecs, count))
end

"""
    serial_set_waitchar!(device, enable, wchar)

Enable/disable waitchar mode and set the wait character.
"""
function serial_set_waitchar!(device::Device, enable::Bool, wchar::UInt8)
    return Int(LibEDTpdv.pdv_serial_set_waitchar(device.handle, enable ? 1 : 0, wchar))
end

"""
    serial_get_waitchar(device)

Return the current wait character.
"""
function serial_get_waitchar(device::Device)
    wchar = Ref{Cuchar}(0)
    LibEDTpdv.pdv_serial_get_waitchar(device.handle, wchar)
    return UInt8(wchar[])
end

"""
    serial_get_numbytes(device)

Return the number of bytes available to read.
"""
function serial_get_numbytes(device::Device)
    return Int(LibEDTpdv.pdv_serial_get_numbytes(device.handle))
end

"""
    serial_get_timeout(device)

Return the serial timeout in milliseconds.
"""
function serial_get_timeout(device::Device)
    return Int(LibEDTpdv.pdv_serial_get_timeout(device.handle))
end

"""
    pause_for_serial(device)

Return the pause-for-serial setting.
"""
function pause_for_serial(device::Device)
    return Int(LibEDTpdv.pdv_get_pause_for_serial(device.handle))
end
