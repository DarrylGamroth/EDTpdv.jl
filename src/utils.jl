abstract type PdvObject end

"""
    unsafe_handle(obj::PdvObject)

Return the underlying `PdvDev` handle pointer.
"""
function unsafe_handle(obj::PdvObject)
    obj.handle
end

"""
    isopen(obj::PdvObject)

Return `true` if the device handle is non-null.
"""
function isopen(obj::PdvObject)
    obj.handle != C_NULL
end

"""
    close(obj::PdvObject)

Close the device handle if this wrapper owns it.
"""
function Base.close(obj::PdvObject)
    if obj.handle != C_NULL && obj.owns
        LibEDTpdv.pdv_close(obj.handle)
        obj.handle = C_NULL
    end
    return nothing
end

function _register_finalizer!(obj::PdvObject)
    if obj.owns
        finalizer(obj) do o
            if o.handle != C_NULL
                LibEDTpdv.pdv_close(o.handle)
                o.handle = C_NULL
            end
        end
    end
    return obj
end

function _check_ptr(ptr, typename::AbstractString)
    ptr == C_NULL && throw(ArgumentError("$typename is NULL"))
    return ptr
end

"""
    bits_to_bytes(bits)

Convert a bit depth to the minimum whole-byte size.
"""
function bits_to_bytes(bits::Integer)
    return (bits + 7) ÷ 8
end
