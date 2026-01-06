using Test
using EDTpdv

function _hw_enabled()
    return lowercase(get(ENV, "EDTPDV_TEST_HW", "")) in ("1", "true", "yes")
end

function _env_int(name::AbstractString, default::Int)
    raw = get(ENV, name, "")
    isempty(raw) && return default
    try
        return parse(Int, raw)
    catch
        return default
    end
end

@testset "EDTpdv hardware" begin
    if !_hw_enabled()
        @info "Skipping EDTpdv hardware tests. Set EDTPDV_TEST_HW=1 to enable."
        return
    end

    dev_name = get(ENV, "EDTPDV_DEV_NAME", EDT_INTERFACE)
    unit = _env_int("EDTPDV_UNIT", 0)
    channel = _env_int("EDTPDV_CHANNEL", 0)

    dev = try
        open(Device; mode=:channel, dev_name=dev_name, unit=unit, channel=channel)
    catch err
        @warn "Failed to open EDTpdv device; skipping hardware tests." exception=err
        return
    end

    try
        @test isopen(dev)
        @test width(dev) isa Int
        @test height(dev) isa Int
        @test depth(dev) isa Int
        @test bytes_per_pixel(dev) isa Int
        @test image_bytes(dev) isa Int
        @test camera_type(dev) isa String
        @test cfgname(dev) isa String

        set_timeout!(dev, 10)
        multibuf!(dev, 2)
        start_images!(dev, 1)
        img = wait_image(dev, 1)
        @test img isa Vector{UInt8}

        last = get_last_image(dev)
        @test last === nothing || last isa AbstractVector{UInt8}
    finally
        close(dev)
    end
end
