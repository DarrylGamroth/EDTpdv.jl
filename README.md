# EDTpdv.jl

Julia bindings and convenience wrappers for the EDT PDV (PCI Digital Vision) library.

## Requirements

- EDT PDV runtime installed (default path: `/opt/EDTpdv`).
- The PDV driver and hardware present for acquisition calls.

By default, the wrapper loads `/opt/EDTpdv/libpdv.so`. Override with:

```bash
export EDTPDV_LIB=/path/to/libpdv.so
```

## Basic Usage

```julia
using EDTpdv

dev = open_channel()  # EDT_INTERFACE, unit=0, channel=0
flush_fifo!(dev)

width_px = width(dev)
height_px = height(dev)
depth_bits = depth(dev)
println("size: $(width_px)x$(height_px) depth=$(depth_bits)")

multibuf!(dev, 4)
start_images!(dev, 1)
image = wait_image(dev, 1)

close(dev)
```

### With `open`/do-block

```julia
using EDTpdv

open(Device; mode=:channel, unit=0, channel=0) do dev
    flush_fifo!(dev)
    multibuf!(dev, 4)
    start_images!(dev, 1)
    img = wait_image(dev, 1)
end
```

## Continuous Acquisition

```julia
using EDTpdv

dev = open_channel()
multibuf!(dev, 8)
setup_continuous!(dev)

start_images!(dev, 1)
buf = wait_image(dev, 1)

stop_continuous!(dev)
close(dev)
```

## Camera Controls

```julia
using EDTpdv

dev = open_channel()
println("camera: ", camera_type(dev))

exposure!(dev, 1000)
gain!(dev, 5)
blacklevel!(dev, 0)

close(dev)
```

## Serial IO (IO Interface)

```julia
using EDTpdv

dev = open_channel()
io = serial_io(dev)

write(io, "ID?\r")
resp = readstring(io)
println(resp)

close(io)
close(dev)
```

Note: the lower-level `serial_read*` functions require a buffer of at least `size + 1` bytes
because the EDT API writes a trailing NUL byte. The IO wrappers handle this automatically.

### Serial IO do-block

```julia
using EDTpdv

open(Device) do dev
    open(SerialIO, dev) do io
        write(io, "ID?\r")
        println(readstring(io))
    end
end
```

## Notes

- Most functions require hardware to be present; without hardware you can still load the module and inspect the API.
- See `docs/plan.md` for the wrapper plan and generation notes.
