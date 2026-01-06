module LibEDTpdv

using CEnum: CEnum, @cenum

const __extension__ = nothing
const __STDC_VERSION__ = 199901
const __has_attribute = 0
const __has_builtin = 0
const __has_feature = 0
const __has_extension = 0
const __has_include = 0
const __has_include_next = 0

const libpdv = get(ENV, "EDTPDV_LIB", "/opt/EDTpdv/libpdv.so")


struct __pthread_internal_list
    __prev::Ptr{__pthread_internal_list}
    __next::Ptr{__pthread_internal_list}
end

const __pthread_list_t = __pthread_internal_list

struct pthread_mutex_t
    data::NTuple{40, UInt8}
end

function Base.getproperty(x::Ptr{pthread_mutex_t}, f::Symbol)
    f === :__data && return Ptr{__pthread_mutex_s}(x + 0)
    f === :__size && return Ptr{NTuple{40, Cchar}}(x + 0)
    f === :__align && return Ptr{Clong}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::pthread_mutex_t, f::Symbol)
    r = Ref{pthread_mutex_t}(x)
    ptr = Base.unsafe_convert(Ptr{pthread_mutex_t}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{pthread_mutex_t}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

function Base.propertynames(x::pthread_mutex_t, private::Bool = false)
    (:__data, :__size, :__align, if private
            fieldnames(typeof(x))
        else
            ()
        end...)
end

const __dirstream = Cvoid

const DIR = __dirstream

const HANDLE = Cint

const uint_t = Cuint

const uchar_t = Cuchar

const ushort_t = Cushort

const edt_critical_section_t = pthread_mutex_t

"""
    edt_dtime()

Return the time in seconds since the last call to [`edt_dtime`](@ref).

Each time [`edt_dtime`](@ref) is called, it returns the time delta since the last time it was called. Therefore the first time [`edt_dtime`](@ref) is called in a process the delta may be meaningless. Time is in seconds and fractions of a second.

This call uses system time, which is system dependent and may be too coarse for some applications. For more accurate time stamping consider the IRIG option and an external IRIG timestamp source, available with some EDT boards.

# Returns
delta time, as a double floating point
### Prototype
```c
EDTAPI double edt_dtime(void);
```
"""
function edt_dtime()
    @ccall libpdv.edt_dtime()::Cdouble
end

"""
    edt_timestamp()

Gets the seconds + fractional seconds since the Epoch, OS independent. Uses system time; accuracy is OS dependent.

# Returns
The timestamp in seconds; a double value with tenths/hundredths/etc on the right of the decimal point.
### Prototype
```c
EDTAPI double edt_timestamp(void);
```
"""
function edt_timestamp()
    @ccall libpdv.edt_timestamp()::Cdouble
end

"""
    edt_msleep(msecs)

Causes the process to sleep for the specified number of milliseconds.

# Arguments
* `msecs`: The number of milliseconds for the process to sleep.
# Returns
0 on success, -1 on failure. If an error occurs, call #edt\\_perror to get the system error message.
### Prototype
```c
EDTAPI void edt_msleep(int msecs);
```
"""
function edt_msleep(msecs)
    @ccall libpdv.edt_msleep(msecs::Cint)::Cvoid
end

"""
    edt_usleep(usecs)

Causes process to sleep for specified number of microseconds.

# Arguments
* `usecs`: Number of microseconds for process to sleep.
# Returns
0 on success; -1 on error
### Prototype
```c
EDTAPI void edt_usleep(int usecs);
```
"""
function edt_usleep(usecs)
    @ccall libpdv.edt_usleep(usecs::Cint)::Cvoid
end

"""
    edt_usec_busywait(usec)

### Prototype
```c
EDTAPI void edt_usec_busywait(uint32_t usec);
```
"""
function edt_usec_busywait(usec)
    @ccall libpdv.edt_usec_busywait(usec::UInt32)::Cvoid
end

"""
    edt_free(ptr)

Convenience routine to free the memory allocated with #[`edt_alloc`](@ref).

# Arguments
* `ptr`: Address of memory buffer to free.
### Prototype
```c
EDTAPI void edt_free(void* ptr);
```
"""
function edt_free(ptr)
    @ccall libpdv.edt_free(ptr::Ptr{Cvoid})::Cvoid
end

"""
    edt_alloc(size)

Convenience routine to allocate memory in a system-independent way. The buffer returned is page aligned. Uses VirtualAlloc on Windows NT systems, posix\\_memalign on UNIX-based systems.

# Arguments
* `size`: Number of bytes of memory to allocate.
# Returns
The address of the allocated memory, or NULL on error. If NULL, use #edt\\_perror to print the error.
### Prototype
```c
EDTAPI void* edt_alloc(size_t size);
```
"""
function edt_alloc(size)
    @ccall libpdv.edt_alloc(size::Csize_t)::Ptr{Cvoid}
end

"""
    edt_closedir(h)

Cross platform function to close a directory handle.

# Arguments
* `h`: Open directory handle.
### Prototype
```c
EDTAPI void edt_closedir(DIRHANDLE h);
```
"""
function edt_closedir(h)
    @ccall libpdv.edt_closedir(h::Ptr{DIR})::Cvoid
end

"""
    edt_opendir(dirname)

Cross platform function to open a directory for reading.

# Arguments
* `dirname`: The name of the directory to open.
# Returns
Directory handle upon success or NULL on failure.
### Prototype
```c
EDTAPI DIRHANDLE edt_opendir(const char* dirname);
```
"""
function edt_opendir(dirname)
    @ccall libpdv.edt_opendir(dirname::Cstring)::Ptr{DIR}
end

"""
    edt_readdir(h, name)

Cross platform function to read the next directory entry.

# Arguments
* `h`: A handle for the directory, returned by #[`edt_opendir`](@ref).
* `name`: A pointer to a string into which the name of the next file in the directory will be written. The string must be allocated by the caller to at least 256 bytes.
# Returns
h if entries remain or NULL if this is the last entry.
### Prototype
```c
EDTAPI int edt_readdir(DIRHANDLE h, char* name);
```
"""
function edt_readdir(h, name)
    @ccall libpdv.edt_readdir(h::Ptr{DIR}, name::Cstring)::Cint
end

"""
    edt_open_datafile(path, name, writing, direct, truncate)

### Prototype
```c
EDTAPI HANDLE edt_open_datafile(const char* path, const char* name, uint8_t writing, uint8_t direct, uint8_t truncate);
```
"""
function edt_open_datafile(path, name, writing, direct, truncate)
    @ccall libpdv.edt_open_datafile(path::Cstring, name::Cstring, writing::UInt8, direct::UInt8, truncate::UInt8)::HANDLE
end

"""
    edt_close_datafile(f)

### Prototype
```c
EDTAPI void edt_close_datafile(HANDLE f);
```
"""
function edt_close_datafile(f)
    @ccall libpdv.edt_close_datafile(f::HANDLE)::Cvoid
end

"""
    edt_write_datafile(f, p, bufsize)

### Prototype
```c
EDTAPI int edt_write_datafile(HANDLE f, void* p, int bufsize);
```
"""
function edt_write_datafile(f, p, bufsize)
    @ccall libpdv.edt_write_datafile(f::HANDLE, p::Ptr{Cvoid}, bufsize::Cint)::Cint
end

"""
    edt_read_datafile(f, p, bufsize)

### Prototype
```c
EDTAPI int edt_read_datafile(HANDLE f, void* p, int bufsize);
```
"""
function edt_read_datafile(f, p, bufsize)
    @ccall libpdv.edt_read_datafile(f::HANDLE, p::Ptr{Cvoid}, bufsize::Cint)::Cint
end

"""
    edt_correct_slashes(str)

Change the slashes in str from forward to back or vice versa depending on whether it's Windows or Unix.

# Arguments
* `str`: String to process.
### Prototype
```c
EDTAPI void edt_correct_slashes(char* str);
```
"""
function edt_correct_slashes(str)
    @ccall libpdv.edt_correct_slashes(str::Cstring)::Cvoid
end

"""
    edt_fwd_to_back(str)

### Prototype
```c
EDTAPI void edt_fwd_to_back(char* str);
```
"""
function edt_fwd_to_back(str)
    @ccall libpdv.edt_fwd_to_back(str::Cstring)::Cvoid
end

"""
    edt_back_to_fwd(str)

### Prototype
```c
EDTAPI void edt_back_to_fwd(char* str);
```
"""
function edt_back_to_fwd(str)
    @ccall libpdv.edt_back_to_fwd(str::Cstring)::Cvoid
end

"""
    edt_get_file_position(f)

### Prototype
```c
EDTAPI uint64_t edt_get_file_position(HANDLE f);
```
"""
function edt_get_file_position(f)
    @ccall libpdv.edt_get_file_position(f::HANDLE)::UInt64
end

"""
    edt_get_file_size(f)

### Prototype
```c
EDTAPI uint64_t edt_get_file_size(HANDLE f);
```
"""
function edt_get_file_size(f)
    @ccall libpdv.edt_get_file_size(f::HANDLE)::UInt64
end

"""
    edt_file_seek(f, pos)

### Prototype
```c
EDTAPI uint64_t edt_file_seek(HANDLE f, uint64_t pos);
```
"""
function edt_file_seek(f, pos)
    @ccall libpdv.edt_file_seek(f::HANDLE, pos::UInt64)::UInt64
end

"""
    edt_access(fname, perm)

Determines file access, independent of operating system.

This a convenience routine that maps to access() on Unix/Linux systems and \\_access() on Windows systems.

# Arguments
* `fname`: Path name of file to check access permissions.
* `perm`: Permission flag(s) to test for. See documentation for access() (Unix/Linux) or \\_access() (Windows) for valid values.
# Returns
0 on success, -1 on failure
### Prototype
```c
EDTAPI int edt_access(const char* fname, int perm);
```
"""
function edt_access(fname, perm)
    @ccall libpdv.edt_access(fname::Cstring, perm::Cint)::Cint
end

"""
    edt_init_critical(cs)

### Prototype
```c
EDTAPI void edt_init_critical(edt_critical_section_t* cs);
```
"""
function edt_init_critical(cs)
    @ccall libpdv.edt_init_critical(cs::Ptr{edt_critical_section_t})::Cvoid
end

"""
    edt_start_critical(cs)

### Prototype
```c
EDTAPI void edt_start_critical(edt_critical_section_t* cs);
```
"""
function edt_start_critical(cs)
    @ccall libpdv.edt_start_critical(cs::Ptr{edt_critical_section_t})::Cvoid
end

"""
    edt_end_critical(cs)

### Prototype
```c
EDTAPI void edt_end_critical(edt_critical_section_t* cs);
```
"""
function edt_end_critical(cs)
    @ccall libpdv.edt_end_critical(cs::Ptr{edt_critical_section_t})::Cvoid
end

const edt_version_string = NTuple{128, Cchar}

"""
Typedef for [`edt_bitpath`](@ref) to send and retrieve bitfile pathnames from the driver.
"""
const edt_bitpath = NTuple{128, Cchar}

"""
    edt_timespec

Structure holding a timestamp broken down into seconds and nanoseconds.
"""
struct edt_timespec
    tv_sec::Int64
    tv_nsec::Int64
end

"""
Structure holding a timestamp broken down into seconds and nanoseconds.
"""
const edt_timespec_t = edt_timespec

"""
    edt_dma_info

Information about active DMA operations.

# See also
edt_get_dma_info().
"""
struct edt_dma_info
    used_dma::UInt32
    alloc_dma::UInt32
    active_dma::UInt32
    interrupts::UInt32
    locks::UInt32
    wait_time::UInt64
    lock_time::UInt64
    lock_array::NTuple{61, UInt32}
    direct_reads::NTuple{256, UInt32}
    direct_writes::NTuple{256, UInt32}
    indirect_reads::NTuple{256, UInt32}
    indirect_writes::NTuple{256, UInt32}
    dma_reads::NTuple{8, UInt32}
    dma_writes::NTuple{8, UInt32}
    active_list_size::UInt32
    free_list_size::UInt32
end

"""
Information about active DMA operations.

# See also
edt_get_dma_info().
"""
const edt_dma_info_t = edt_dma_info

@cenum edt_dma_wait_status::Int32 begin
    EDT_WAIT_UNKNOWN = -1
    EDT_WAIT_OK = 0
    EDT_WAIT_TIMEOUT = 1
    EDT_WAIT_OK_TIMEOUT = 2
end

const EdtDmaWaitStatus = edt_dma_wait_status

"""
    edt_dma_timeout_action

Timeout action.
"""
@cenum edt_dma_timeout_action::UInt32 begin
    EDT_TIMEOUT_NULL = 0
    EDT_TIMEOUT_BIT_STROBE = 1
end

"""
Timeout action.
"""
const EdtDmaTimeoutAction = edt_dma_timeout_action

"""
    edt_kernel_event

Event flag bits, indicating interest in these events.

# See also
edt\\_set\\_event\\_func()
"""
@cenum edt_kernel_event::UInt32 begin
    EDT_EODMA_EVENT = 1
    EDT_EVENT_BUF = 2
    EDT_EVENT_STAT = 3
    EDT_PDV_EVENT_ACQUIRE = 4
    EDT_EVENT_PCD_STAT1 = 5
    EDT_EVENT_PCD_STAT2 = 6
    EDT_EVENT_PCD_STAT3 = 7
    EDT_EVENT_PCD_STAT4 = 8
    EDT_PDV_STROBE_EVENT = 9
    EDT_PDV_EVENT_FVAL = 10
    EDT_PDV_EVENT_TRIGINT = 11
    EDT_EVENT_TEMP = 12
    EDT_MAX_EVENT_TYPES = 13
end

const edt_device = Cvoid

"""
[`PDV`](@ref) device handle, an opaque pointer.

For convenience, we use [`PdvDev`](@ref) in [`PDV`](@ref) applications, and [`EdtDev`](@ref) in applications for all other EDT applications. [`PdvDev`](@ref) and [`EdtDev`](@ref) are interchangable, some functions from edtlib can be called using a [`PdvDev`](@ref) handle.
"""
const PdvDev = Ptr{edt_device}

const EdtDev = PdvDev

"""
    edt_pdv_header_type

Definition for header data for buffers.
"""
@cenum edt_pdv_header_type::UInt32 begin
    PDV_HDR_TYPE_NONE = 0
    PDV_HDR_TYPE_IRIG1 = 1
    PDV_HDR_TYPE_FRAMECOUNT = 2
    PDV_HDR_TYPE_IRIG2 = 3
    PDV_HDR_TYPE_BUFHEADER = 4
end

"""
Definition for header data for buffers.
"""
const EdtPdvHeaderType = edt_pdv_header_type

"""
    edt_pdv_header_position

Position of header relative to the bounds of a DMA buffer.

For disk I/O speed, header memory may be allocated with ring-buffer memory even if the header is not in the DMA stream. This covers all possible relationships of header memory to image data.
"""
@cenum edt_pdv_header_position::UInt32 begin
    PDV_HDR_POS_NONE = 0
    PDV_HDR_POS_BEFORE = 1
    PDV_HDR_POS_BEGIN = 2
    PDV_HDR_POS_MIDDLE = 3
    PDV_HDR_POS_END = 4
    PDV_HDR_POS_AFTER = 5
    PDV_HDR_POS_SEPARATE = 6
end

"""
Position of header relative to the bounds of a DMA buffer.

For disk I/O speed, header memory may be allocated with ring-buffer memory even if the header is not in the DMA stream. This covers all possible relationships of header memory to image data.
"""
const EdtPdvHeaderPosition = edt_pdv_header_position

"""
    pdv_bits_to_bytes(bits)

### Prototype
```c
static inline int pdv_bits_to_bytes(int bits);
```
"""
function pdv_bits_to_bytes(bits)
    @ccall libpdv.pdv_bits_to_bytes(bits::Cint)::Cint
end

"""
    edt_pdv_serial_cmd_flag

Serial command flags. Generally not used in application code.
"""
@cenum edt_pdv_serial_cmd_flag::UInt32 begin
    PDV_SER_CMD_FLAG_NONE = 0
    PDV_SER_CMD_FLAG_NORESP = 1
end

"""
Serial command flags. Generally not used in application code.
"""
const EdtPdvSerCmdFlag = edt_pdv_serial_cmd_flag

"""
    pdv_close(pdv_p)

Closes the specified device and frees the image memory and other resources associated with the device handle.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
0 if successful, -1 if unsuccessful.
### Prototype
```c
EDTAPI int pdv_close(PdvDev pdv_p);
```
"""
function pdv_close(pdv_p)
    @ccall libpdv.pdv_close(pdv_p::PdvDev)::Cint
end

"""
    pdv_open(dev_name, unit)

Opens channel 0 of an EDT Framegrabber for application access.

To open a specific channel on multi-channel device, see pdv_open_channel().

# Arguments
* `dev_name`: The name of the device. For EDT imaging boards use EDT_INTERFACE.
* `unit`: The unit number of the device (board). The first device is 0.
# Returns
The opened [`PDV`](@ref) device handle if successful, or NULL if unsuccessful.
# See also
pdv_open_channel()

### Prototype
```c
EDT_CHECK_RETURN EDT_DEALLOCATE_WITH(pdv_close) EDTAPI PdvDev pdv_open(const char* dev_name, int unit);
```
"""
function pdv_open(dev_name, unit)
    @ccall libpdv.pdv_open(dev_name::Cstring, unit::Cint)::PdvDev
end

"""
    pdv_open_device(dev_name, unit, channel, verbose)

Open the pdv device, with option to suppress non-existent device console output.

# Arguments
* `dev_name`: The name of the device. For EDT imaging boards use EDT_INTERFACE.
* `unit`: The unit number of the device (board). The first device is 0.
* `channel`: The channel of the specified unit to open. The first channel is 0.
* `verbose`: Enable verbose output (1) or not (0).
# Returns
The opened [`PDV`](@ref) device handle if successful, or NULL if unsuccessful.
### Prototype
```c
EDT_CHECK_RETURN EDT_DEALLOCATE_WITH(pdv_close) EDTAPI PdvDev pdv_open_device(const char* dev_name, int unit, int channel, int verbose);
```
"""
function pdv_open_device(dev_name, unit, channel, verbose)
    @ccall libpdv.pdv_open_device(dev_name::Cstring, unit::Cint, channel::Cint, verbose::Cint)::PdvDev
end

"""
    pdv_open_channel(dev_name, unit, channel)

Opens an EDT Framegrabber channel for application access.

# Arguments
* `dev_name`: The name of the device. For EDT imaging boards use EDT_INTERFACE.
* `unit`: The unit number of the device (board). The first device is 0.
* `channel`: The channel of the specified unit to open. The first channel is 0.
# Returns
The opened [`PDV`](@ref) device handle if successful, or NULL if unsuccessful.
# See also
pdv_open()

### Prototype
```c
EDT_CHECK_RETURN EDT_DEALLOCATE_WITH(pdv_close) EDTAPI PdvDev pdv_open_channel(const char* dev_name, int unit, int channel);
```
"""
function pdv_open_channel(dev_name, unit, channel)
    @ccall libpdv.pdv_open_channel(dev_name::Cstring, unit::Cint, channel::Cint)::PdvDev
end

"""
    pdv_set_debug(debug)

Sets the debug level. For values, see the msg.

# Arguments
* `debug`: The debug level.
### Prototype
```c
EDTAPI void pdv_set_debug(int debug);
```
"""
function pdv_set_debug(debug)
    @ccall libpdv.pdv_set_debug(debug::Cint)::Cvoid
end

"""
    pdv_get_debug()

Gets the debug level, as set by [`pdv_set_debug`](@ref) or outside environment variables. For values, see the msg.

# Returns
The debug level.
### Prototype
```c
EDTAPI int pdv_get_debug(void);
```
"""
function pdv_get_debug()
    @ccall libpdv.pdv_get_debug()::Cint
end

"""
    pdv_read(pdv_p, buf, size)

Reads image data from the EDT framegrabber board. [`pdv_read`](@ref) is not supported on all platforms and is included mainly for historical reasons. Consider using pdv_image() or pdv_start_images() instead.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`: The data buffer to receive the image.
* `size`: The size, in bytes, of the data buffer; ordinarily, width * height * bytes per pixel
# Returns
The number of bytes read.
# See also
pdv_image(), pdv_start_images()

### Prototype
```c
EDTAPI int pdv_read(PdvDev pdv_p, unsigned char* buf, unsigned long size);
```
"""
function pdv_read(pdv_p, buf, size)
    @ccall libpdv.pdv_read(pdv_p::PdvDev, buf::Ptr{Cuchar}, size::Culong)::Cint
end

"""
    pdv_image(pdv_p)

Start image acquisition if not already started, then wait for and return the address of the next available image.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The address of the image buffer that has been acquired.
# See also
pdv_start_images(), pdv_wait_images()

### Prototype
```c
EDTAPI uint8_t* pdv_image(PdvDev pdv_p);
```
"""
function pdv_image(pdv_p)
    @ccall libpdv.pdv_image(pdv_p::PdvDev)::Ptr{UInt8}
end

"""
    pdv_multibuf(pdv_p, numbufs)

Sets the number of multiple buffers to use in ring buffer continuous mode, and allocates them. This routine allocates the buffers itself, in kernel or low memory as required by the EDT device driver for optimal DMA.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `numbufs`: The number of buffers to allocate.
# Returns
0 on success, -1 on failure.
# See also
pdv_buffer_addresses()

### Prototype
```c
EDTAPI int pdv_multibuf(PdvDev pdv_p, int numbufs);
```
"""
function pdv_multibuf(pdv_p, numbufs)
    @ccall libpdv.pdv_multibuf(pdv_p::PdvDev, numbufs::Cint)::Cint
end

"""
    pdv_start_images(pdv_p, count)

Starts multiple image acquisition.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `count`: Number of images to start. A value of 0 starts freerun. To stop freerun, call pdv_start_images again with a `count` of 1 or more.
# See also
pdv_wait_images(), pdv_multibuf()

### Prototype
```c
EDTAPI void pdv_start_images(PdvDev pdv_p, int count);
```
"""
function pdv_start_images(pdv_p, count)
    @ccall libpdv.pdv_start_images(pdv_p::PdvDev, count::Cint)::Cvoid
end

"""
    pdv_buffer_addresses(pdv_p)

Returns the addresses of the buffers allocated by the last call to pdv_multibuf(). See pdv_wait_images() for a description and example of use.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
An array of pointers to image buffers. The size of the array is equal to the number of buffers allocated.
# See also
pdv_multibuf()

### Prototype
```c
EDTAPI uint8_t** pdv_buffer_addresses(PdvDev pdv_p);
```
"""
function pdv_buffer_addresses(pdv_p)
    @ccall libpdv.pdv_buffer_addresses(pdv_p::PdvDev)::Ptr{Ptr{UInt8}}
end

"""
    pdv_wait_images(pdv_p, count)

Waits for the images started by pdv_start_images.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `count`: Number of images to wait for before returning. If `count` is greater than the number of buffers set by pdv_multibuf, only the last `count` images will be available when this function returns.
# Returns
The address of the last image.
# See also
pdv_wait_images_timed_raw()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_images(PdvDev pdv_p, int count);
```
"""
function pdv_wait_images(pdv_p, count)
    @ccall libpdv.pdv_wait_images(pdv_p::PdvDev, count::Cint)::Ptr{UInt8}
end

"""
    pdv_wait_images_timed_raw(pdv_p, count, ts, skip_deinterleave)

Waits for one or more images started by pdv_start_images. Optionally controls post-processing and returns a timestamp.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `count`: Number of images to wait for before returning. If `count` is greater than the number of buffers set by pdv_multibuf, only the last `count` images will be available when this function returns.
* `ts`:\\[out\\] If non-null, returns the time when the last image DMA transfer finished. Pass NULL to skip getting the last image's timestamp.
* `skip_deinterleave`: If true, any de-interleaving post pocessing will be skipped.
# Returns
The address of the last image.
# See also
pdv_start_images(), pdv_wait_images(), pdv_decode_timestamp()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_images_timed_raw(PdvDev pdv_p, int count, edt_timespec_t* ts, bool skip_deinterleave);
```
"""
function pdv_wait_images_timed_raw(pdv_p, count, ts, skip_deinterleave)
    @ccall libpdv.pdv_wait_images_timed_raw(pdv_p::PdvDev, count::Cint, ts::Ptr{edt_timespec_t}, skip_deinterleave::Bool)::Ptr{UInt8}
end

"""
    pdv_wait_last_image(pdv_p, num_skipped)

Waits for the last image that has been acquired. A convenience wrapper of pdv_wait_last_image_timed_raw().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `num_skipped`:\\[out\\] Pointer to an integer which will be filled in with number of images skipped, if any. Can be set to NULL to ignore.
# Returns
Address of the image.
# See also
pdv_start_images(), pdv_wait_images(), pdv_wait_last_image_timed_raw()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_last_image(PdvDev pdv_p, int* num_skipped);
```
"""
function pdv_wait_last_image(pdv_p, num_skipped)
    @ccall libpdv.pdv_wait_last_image(pdv_p::PdvDev, num_skipped::Ptr{Cint})::Ptr{UInt8}
end

"""
    pdv_wait_last_image_timed_raw(pdv_p, num_skipped, ts, skip_deinterleave)

Waits for the last image that has been acquired. Optionally controls post-processing and returns a timestamp.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `num_skipped`:\\[out\\] Pointer to an integer which will be filled in with number of images skipped, if any. Can be set to NULL to ignore.
* `ts`:\\[out\\] If non-null, returns the time when the last image DMA transfer finished. Pass NULL to ignore. See pdv_wait_images_timed_raw for a detailed explanation.
* `skip_deinterleave`: If true, any de-interleave post pocessing will be skipped.
# Returns
The address of the last image.
# See also
pdv_start_images(), pdv_wait_images(), pdv_wait_last_image()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_last_image_timed_raw(PdvDev pdv_p, int* num_skipped, edt_timespec_t* ts, bool skip_deinterleave);
```
"""
function pdv_wait_last_image_timed_raw(pdv_p, num_skipped, ts, skip_deinterleave)
    @ccall libpdv.pdv_wait_last_image_timed_raw(pdv_p::PdvDev, num_skipped::Ptr{Cint}, ts::Ptr{edt_timespec_t}, skip_deinterleave::Bool)::Ptr{UInt8}
end

"""
    pdv_get_last_image(pdv_p)

Returns a pointer to the last image that was acquired (non-blocking). It will return a pointer to the same buffer if called a second time with no new images acquired.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The address of the last image acquired.
# See also
pdv_wait_last_image(), pdv_wait_last_image_timed_raw(), pdv_wait_images(), edt_done_count()

### Prototype
```c
EDTAPI uint8_t* pdv_get_last_image(PdvDev pdv_p);
```
"""
function pdv_get_last_image(pdv_p)
    @ccall libpdv.pdv_get_last_image(pdv_p::PdvDev)::Ptr{UInt8}
end

"""
    pdv_get_last_image_raw(pdv_p)

Returns a pointer to the last image that was acquired (non-blocking). Identical to the pdv_get_last_image(), except that it skips any image deinterleave method defined by the **method_interlace** config file directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The address of the last image acquired.
# See also
pdv_get_last_image()

### Prototype
```c
EDTAPI uint8_t* pdv_get_last_image_raw(PdvDev pdv_p);
```
"""
function pdv_get_last_image_raw(pdv_p)
    @ccall libpdv.pdv_get_last_image_raw(pdv_p::PdvDev)::Ptr{UInt8}
end

"""
    pdv_wait_next_image(pdv_p, num_skipped)

Waits for the \\_next\\_ image, skipping any previously started images. A convenience wrapper of pdv_wait_next_image_timed_raw().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `num_skipped`:\\[out\\] Pointer to an integer which will be filled in with number of images skipped, if any. Can be set to NULL to ignore.
# Returns
The address of the image.
# See also
pdv_start_images(), pdv_wait_images(), pdv_wait_next_image_timed_raw()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_next_image(PdvDev pdv_p, int* num_skipped);
```
"""
function pdv_wait_next_image(pdv_p, num_skipped)
    @ccall libpdv.pdv_wait_next_image(pdv_p::PdvDev, num_skipped::Ptr{Cint})::Ptr{UInt8}
end

"""
    pdv_wait_next_image_timed_raw(pdv_p, num_skipped, ts, skip_deinterleave)

Waits for the \\_next\\_ image, skipping any previously started images. Optionally controls post-processing and returns a timestamp.

The format of the returned data and timestamp is described in pdv_wait_images_timed_raw.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `num_skipped`:\\[out\\] Pointer to an integer which will be filled in with number of images skipped, if any. Can be set to NULL to ignore.
* `ts`:\\[out\\] If non-null, returns the time when the last image DMA transfer finished. Pass NULL to ignore. See pdv_wait_images_timed_raw for a detailed explanation.
* `skip_deinterleave`: If true, any de-interleave post processing will be skipped.
# Returns
The address of the image.
# See also
pdv_start_images(), pdv_wait_images(), pdv_wait_next_image()

### Prototype
```c
EDTAPI uint8_t* pdv_wait_next_image_timed_raw(PdvDev pdv_p, int* num_skipped, edt_timespec_t* ts, bool skip_deinterleave);
```
"""
function pdv_wait_next_image_timed_raw(pdv_p, num_skipped, ts, skip_deinterleave)
    @ccall libpdv.pdv_wait_next_image_timed_raw(pdv_p::PdvDev, num_skipped::Ptr{Cint}, ts::Ptr{edt_timespec_t}, skip_deinterleave::Bool)::Ptr{UInt8}
end

"""
    pdv_decode_timestamp(ts)

Decode the timestamp from pdv_wait_images_timed_raw and related functions.

Converts the timestamp returned by the \\_timed subroutines to floating-point, representing the value in seconds.

# Arguments
* `ts`: The timestamp returned from pdv_wait_images_timed_raw or similar function.
# Returns
The timestamp as a double-precision floating-point value.
# See also
pdv_wait_images_timed_raw(), pdv_wait_last_image_timed_raw(), pdv_wait_next_image_timed_raw()

### Prototype
```c
EDTAPI double pdv_decode_timestamp(const edt_timespec_t* ts);
```
"""
function pdv_decode_timestamp(ts)
    @ccall libpdv.pdv_decode_timestamp(ts::Ptr{edt_timespec_t})::Cdouble
end

"""
    pdv_setup_continuous(pdv_p)

Performs setup for continuous transfers.

Shouldn't need to be called by user apps since pdv_start_images(), etc. call it already. But it is in some EDT example applications from before this was the case.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# See also
pdv_stop_continuous()

### Prototype
```c
EDTAPI void pdv_setup_continuous(PdvDev pdv_p);
```
"""
function pdv_setup_continuous(pdv_p)
    @ccall libpdv.pdv_setup_continuous(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_stop_continuous(pdv_p)

Performs un-setup for continuous transfers.

Shouldn't need to be called by user apps since other subroutines (e.g. pdv_timeout_restart()) now call it as needed. But it is still in some EDT example applications from before this was the case.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# See also
pdv_setup_continuous()

### Prototype
```c
EDTAPI void pdv_stop_continuous(PdvDev pdv_p);
```
"""
function pdv_stop_continuous(pdv_p)
    @ccall libpdv.pdv_stop_continuous(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_in_continuous(pdv_p)

Gets the status of the continuous flag.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
1 if continuous, 0 if not.
# See also
pdv_setup_continuous(), pdv_stop_continuous()

### Prototype
```c
EDTAPI int pdv_in_continuous(PdvDev pdv_p);
```
"""
function pdv_in_continuous(pdv_p)
    @ccall libpdv.pdv_in_continuous(pdv_p::PdvDev)::Cint
end

"""
    pdv_flush_fifo(pdv_p)

Flushes the board's input FIFOs, to allow new data transfers to start from a known state.

This subroutine effectively resets the board, so calling it after every image will degrade performance and is not recommended. Additionally, resetting after a timeout, involves more than just flushing the FIFOs -- therefore we recommend using pdv_timeout_restart() to reset (which calls this, among other things).

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
### Prototype
```c
EDTAPI void pdv_flush_fifo(PdvDev pdv_p);
```
"""
function pdv_flush_fifo(pdv_p)
    @ccall libpdv.pdv_flush_fifo(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_set_timeout(pdv_p, value)

Sets the length of time to wait for data on acquisition before timing out.

* `> 0` -- The number of milliseconds to wait for timeout.

* `0` -- Block forever waiting for data.

* `-1` -- Revert to automatic timeouts.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`:
# Returns
0 on success, -1 on failure.
# See also
**user_timeout** directive in the <a>* href="https://edt.com/downloads/camera-configuration-guide.pdf">Camera Configuration Guide</A>

### Prototype
```c
EDTAPI int pdv_set_timeout(PdvDev pdv_p, int value);
```
"""
function pdv_set_timeout(pdv_p, value)
    @ccall libpdv.pdv_set_timeout(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_timeout(pdv_p)

Gets the length of time to wait for data on acquisition before timing out.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The timeout value, in milliseconds.
### Prototype
```c
EDTAPI int pdv_get_timeout(PdvDev pdv_p);
```
"""
function pdv_get_timeout(pdv_p)
    @ccall libpdv.pdv_get_timeout(pdv_p::PdvDev)::Cint
end

"""
    pdv_timeouts(pdv_p)

Returns the number of times the device timed out (frame didn't transfer completely or at all) since the device was opened.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of timeouts since the device was opened.
# See also
pdv_set_timeout(), pdv_get_timeout(), pdv_timeout_restart(), pdv_enable_framesync()

### Prototype
```c
EDTAPI int pdv_timeouts(PdvDev pdv_p);
```
"""
function pdv_timeouts(pdv_p)
    @ccall libpdv.pdv_timeouts(pdv_p::PdvDev)::Cint
end

"""
    pdv_timeout_restart(pdv_p, restart)

Cleans up after a timeout, particularly when you've prestarted multiple buffers or if you've forced a timeout with edt_do_timeout().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `restart`: Whether to immediately restart acquiring.
# Returns
The number of buffers left undone.
# See also
pdv_timeouts()

### Prototype
```c
EDTAPI int pdv_timeout_restart(PdvDev pdv_p, int restart);
```
"""
function pdv_timeout_restart(pdv_p, restart)
    @ccall libpdv.pdv_timeout_restart(pdv_p::PdvDev, restart::Cint)::Cint
end

"""
    pdv_overrun(pdv_p)

Determines whether data overran on the last aquire.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of bytes of data remaining from last aquire. 0 indicates no overrun.
### Prototype
```c
EDTAPI int pdv_overrun(PdvDev pdv_p);
```
"""
function pdv_overrun(pdv_p)
    @ccall libpdv.pdv_overrun(pdv_p::PdvDev)::Cint
end

"""
    pdv_check_frame(pdv_p, image, imagesize, verbose)

Search an image buffer for a valid frame. If the beginning of the image was not at the start of the image buffer, stop and re-start continuous image aquisition.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `image`: Image buffer to inspect.
* `imagesize`: Total size (in bytes) of the image in `imagebuf`.
* `verbose`: Pass `true` (1) to enable more log messages.
### Prototype
```c
EDTAPI void pdv_check_frame(PdvDev pdv_p, uint16_t* image, uint32_t imagesize, int verbose);
```
"""
function pdv_check_frame(pdv_p, image, imagesize, verbose)
    @ccall libpdv.pdv_check_frame(pdv_p::PdvDev, image::Ptr{UInt16}, imagesize::UInt32, verbose::Cint)::Cvoid
end

"""
    pdv_get_interleave_data(pdv_p, buf, bufnum)

Returns the post-processed data corresponding to a given raw buffer. It does not cause post-processing to happen, it only returns the data if it already exists.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`: A pointer to the raw data buffer.
* `bufnum`: The index of the data buffer in the buffer array.
# Returns
A pointer to the post-processed buffer.
### Prototype
```c
EDTAPI uint8_t* pdv_get_interleave_data(PdvDev pdv_p, uint8_t* buf, int bufnum);
```
"""
function pdv_get_interleave_data(pdv_p, buf, bufnum)
    @ccall libpdv.pdv_get_interleave_data(pdv_p::PdvDev, buf::Ptr{UInt8}, bufnum::Cint)::Ptr{UInt8}
end

"""
    pdv_get_force_single(pdv_p)

Returns the value of the **force_single** flag. This flag is 0 by default, and is set by the **force_single** directive in the config file.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Value of the force\\_single flag.
# See also
pdv_multibuf(), **force_single** <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> camera configuration</A> directive.

### Prototype
```c
EDTAPI int pdv_get_force_single(PdvDev pdv_p);
```
"""
function pdv_get_force_single(pdv_p)
    @ccall libpdv.pdv_get_force_single(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_fval_done(pdv_p, enable)

Enables frame valid done functionality on the board.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `enable`: Whether to enable of disable frame valid done.
# See also
pdv_get_lines_xferred(), **fval_done** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</A>

### Prototype
```c
EDTAPI void pdv_set_fval_done(PdvDev pdv_p, int enable);
```
"""
function pdv_set_fval_done(pdv_p, enable)
    @ccall libpdv.pdv_set_fval_done(pdv_p::PdvDev, enable::Cint)::Cvoid
end

"""
    pdv_get_fval_done(pdv_p)

Returns the frame valid done enable state.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Whether the frame valid done feature is enabled.
# See also
pdv_set_fval_done()

### Prototype
```c
EDTAPI int pdv_get_fval_done(PdvDev pdv_p);
```
"""
function pdv_get_fval_done(pdv_p)
    @ccall libpdv.pdv_get_fval_done(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_lines_xferred(pdv_p)

Gets the number of lines transferred during the last acquire. Typically only used in line scan applications where the actual number of lines transferred into a given buffer is unknown at the time of the acquire.

Note that if acquires are continuously being queued (as in [`pdv_start_images`](@ref)(pdv_p, n) where *n* is greater than 1), the number of lines tranferred may not reflect the last finished acquire.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of lines transferred on the last acquire.
# See also
pdv_get_width_xferred()

### Prototype
```c
EDTAPI int pdv_get_lines_xferred(PdvDev pdv_p);
```
"""
function pdv_get_lines_xferred(pdv_p)
    @ccall libpdv.pdv_get_lines_xferred(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_width_xferred(pdv_p)

Gets the number of pixels transferred during the last line transferred. Typically only used in line scan applications where the actual number of pixels transferred per line may not be known.

Note that if lines are continuously being transferred (the normal case), the number of pixels tranferred may not reflect the last finished line.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of pixels transferred on the last line.
# See also
pdv_get_lines_xferred()

### Prototype
```c
EDTAPI int pdv_get_width_xferred(PdvDev pdv_p);
```
"""
function pdv_get_width_xferred(pdv_p)
    @ccall libpdv.pdv_get_width_xferred(pdv_p::PdvDev)::Cint
end

"""
    pdv_cl_get_fval_counter(pdv_p)

Gets the number of frame valid transitions that have been seen by the board since the last time the board/channel was initialized or the last time pdv_cl_reset_fval_counter() was called.

The number returned is NOT the number of frames read in; the counter on the board counts all frame ticks seen whether images are being read in or not. As such this subroutine can be useful in determining whether a camera is connected (among other things), assuming that the camera is a freerun camera or has a continuous external trigger.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of frame valid transitions seen.
# See also
pdv_cl_reset_fval_counter()

### Prototype
```c
EDTAPI int pdv_cl_get_fval_counter(PdvDev pdv_p);
```
"""
function pdv_cl_get_fval_counter(pdv_p)
    @ccall libpdv.pdv_cl_get_fval_counter(pdv_p::PdvDev)::Cint
end

"""
    pdv_cl_reset_fval_counter(pdv_p)

Resets the frame valid counter to zero.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# See also
pdv_cl_get_fval_counter()

### Prototype
```c
EDTAPI void pdv_cl_reset_fval_counter(PdvDev pdv_p);
```
"""
function pdv_cl_reset_fval_counter(pdv_p)
    @ccall libpdv.pdv_cl_reset_fval_counter(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_get_camera_type(pdv_p)

Gets the type of the camera, as set by *initcam* from the camera configuration file's camera description directives. This is a concatenation of **camera_class**, **camera_model**, and **camera_info**, directives.

!!! note

    the camera class, model and info are for information only, and are not used by the driver or library. They are intended for use by applications to allow the user to browse and select a specific camera configuration.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
A string representing the camera type.
# See also
pdv_get_camera_class(), pdv_get_camera_model(), pdv_get_camera_info(), **camera_class**, **camera_model**, **camera_info** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>

### Prototype
```c
EDTAPI const char* pdv_get_camera_type(PdvDev pdv_p);
```
"""
function pdv_get_camera_type(pdv_p)
    @ccall libpdv.pdv_get_camera_type(pdv_p::PdvDev)::Cstring
end

"""
    pdv_get_camera_class(pdv_p)

Gets the class of the camera (usually the manufacturer name), as set by *initcam* from the camera\\_config file **camera_class** directive.

!!! note

    the camera class, model and info are for information only, and are not used by the driver or library. They are intended for use by applications to allow the user to browse and select a specific camera configuration.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
A string representing the camera class.
# See also
pdv_get_camera_type(), **camera_class** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>

### Prototype
```c
EDTAPI const char* pdv_get_camera_class(PdvDev pdv_p);
```
"""
function pdv_get_camera_class(pdv_p)
    @ccall libpdv.pdv_get_camera_class(pdv_p::PdvDev)::Cstring
end

"""
    pdv_get_camera_model(pdv_p)

Gets the model of the camera, as set by initcam from the camera\\_config file **camera_model** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
A string representing the camera model.
# See also
**camera_model** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>

### Prototype
```c
EDTAPI const char* pdv_get_camera_model(PdvDev pdv_p);
```
"""
function pdv_get_camera_model(pdv_p)
    @ccall libpdv.pdv_get_camera_model(pdv_p::PdvDev)::Cstring
end

"""
    pdv_get_camera_info(pdv_p)

Gets the string set by the **camera_info** configuration file directive.

See pdv_get_camera_type() for more information on camera strings.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
A string representing the camera info.
# See also
**camera_info** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>

### Prototype
```c
EDTAPI const char* pdv_get_camera_info(PdvDev pdv_p);
```
"""
function pdv_get_camera_info(pdv_p)
    @ccall libpdv.pdv_get_camera_info(pdv_p::PdvDev)::Cstring
end

"""
    pdv_set_width(pdv_p, value)

Sets width and reallocates buffers accordingly.

Since we rarely ever set width and not height, you should normally just use pdv_set_image_size() to set both.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The new width in pixels.
# Returns
0 on success, -1 on error.
### Prototype
```c
EDTAPI int pdv_set_width(PdvDev pdv_p, int value);
```
"""
function pdv_set_width(pdv_p, value)
    @ccall libpdv.pdv_set_width(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_width(pdv_p)

Gets the width of the image (number of pixels per line), based on the camera in use.

If the width has been changed by setting a region of interest, the modified values are returned; use pdv_get_cam_width() to get the unchanged width.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The width in pixels of images returned from an aquire.
# See also
pdv_get_cam_width(), **width** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_width(PdvDev pdv_p);
```
"""
function pdv_get_width(pdv_p)
    @ccall libpdv.pdv_get_width(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_pitch(pdv_p)

Gets the number of bytes per line.

The result will be different than pdv_get_width() if the pixel depth is not 8 bits.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The pitch in bytes of images returned from an aquire.
### Prototype
```c
EDTAPI int pdv_get_pitch(PdvDev pdv_p);
```
"""
function pdv_get_pitch(pdv_p)
    @ccall libpdv.pdv_get_pitch(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_height(pdv_p, value)

Sets height and reallocates buffers accordingly.

Since we rarely ever set height and not width, you should normally just use pdv_set_image_size() to set both.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The new height in pixels.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_set_height(PdvDev pdv_p, int value);
```
"""
function pdv_set_height(pdv_p, value)
    @ccall libpdv.pdv_set_height(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_height(pdv_p)

Gets the height of the image (number of lines), based on the camera in use.

If the height has been changed by setting a region of interest, the modified values are returned; use pdv_get_cam_height() to get the unchanged height.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The height in pixels of images returned from an acquire.
# See also
pdv_get_cam_height(), **height** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_height(PdvDev pdv_p);
```
"""
function pdv_get_height(pdv_p)
    @ccall libpdv.pdv_get_height(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_frame_height(pdv_p)

Gets the height of a single image when the system is configured for multiple images per buffer.

Unless the frame\\_height directive is specified in the config file this will return zero.

This setting is useful in some cases where special handling of image data by an application is used such as multiple frames per image.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The height of a single image frame in pixels.
# See also
pdv_get_height(), pdv_get_cam_height()

### Prototype
```c
EDTAPI int pdv_get_frame_height(PdvDev pdv_p);
```
"""
function pdv_get_frame_height(pdv_p)
    @ccall libpdv.pdv_get_frame_height(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_depth_extdepth_dpath(pdv_p, depth, extdepth, dpath)

Sets the bit depth, extended depth, and camera link data path.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `depth`: The new depth value.
* `extdepth`: The new extended depth value.
* `dpath`: The new camera link data path value. Passing zero (0) will cause an appropriate value to be calculated automatically.
# Returns
0 on success
# See also
pdv_cl_set_base_channels(), pdv_get_depth(), pdv_get_extdepth(), `depth`, `extdepth`, `CL_DATA_PATH_NORM` directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide </a>

### Prototype
```c
EDTAPI int pdv_set_depth_extdepth_dpath(PdvDev pdv_p, int depth, int extdepth, unsigned int dpath);
```
"""
function pdv_set_depth_extdepth_dpath(pdv_p, depth, extdepth, dpath)
    @ccall libpdv.pdv_set_depth_extdepth_dpath(pdv_p::PdvDev, depth::Cint, extdepth::Cint, dpath::Cuint)::Cint
end

"""
    pdv_get_depth(pdv_p)

Gets the depth of the image (number of bits per pixel), as set in the configuration file for the camera in use.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of bits per pixel in the image.
# See also
pdv_set_depth_extdepth_dpath(), pdv_get_extdepth(), **depth** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_depth(PdvDev pdv_p);
```
"""
function pdv_get_depth(pdv_p)
    @ccall libpdv.pdv_get_depth(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_extdepth(pdv_p)

Gets the extended depth of the camera.

The extended depth is the number of valid bits per pixel that the camera outputs, as set by *initcam* from the configuration file **edtdepth** directive. Note that if **depth** is set differently than **extdepth**, the actual number of bits per pixel passed through by the EDT framegrabber board will be different. For example, if **extdepth** is 10 but **depth** is 8, the board will only transfer one byte per pixel, even though the camera is outputting two bytes per pixel.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The extended depth.
# See also
pdv_get_depth(), **extdepth** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_extdepth(PdvDev pdv_p);
```
"""
function pdv_get_extdepth(pdv_p)
    @ccall libpdv.pdv_get_extdepth(pdv_p::PdvDev)::Cint
end

"""
    pdv_cl_set_base_channels(pdv_p, htaps, vtaps)

Set the number of channels (taps) and horizontal and vertical alignment of the taps.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `htaps`: The number of horizontal taps.
* `vtaps`: The number of vertical taps.
# See also
pdv_set_depth_extdepth_dpath(), **hskip**, **vskip** and **CL_DATA_PATH_NORM** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI void pdv_cl_set_base_channels(PdvDev pdv_p, int htaps, int vtaps);
```
"""
function pdv_cl_set_base_channels(pdv_p, htaps, vtaps)
    @ccall libpdv.pdv_cl_set_base_channels(pdv_p::PdvDev, htaps::Cint, vtaps::Cint)::Cvoid
end

"""
    pdv_set_shutter_method(pdv_p, method, mcl)

Set the device's exposure method and CC line state.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `method`: The exposure method.
* `mcl`: The mode control (CC line) state.
# Returns
0 on success, -1 on failure.
# See also
pdv_get_shutter_method()

### Prototype
```c
EDTAPI int pdv_set_shutter_method(PdvDev pdv_p, int method, unsigned int mcl);
```
"""
function pdv_set_shutter_method(pdv_p, method, mcl)
    @ccall libpdv.pdv_set_shutter_method(pdv_p::PdvDev, method::Cint, mcl::Cuint)::Cint
end

"""
    pdv_get_shutter_method(pdv_p, mcl)

Returns the shutter (expose) timing method and mode control (CC) state.

See pdv_set_shutter_method for an explanation of the return value (shutter method) and `mcl` parameter;

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `mcl`:\\[out\\] Returns mode control (CC line) state. Pass NULL to skip reading the current state.
# Returns
The shutter (expose) timing method.
# See also
pdv_set_shutter_method()

### Prototype
```c
EDTAPI int pdv_get_shutter_method(PdvDev pdv_p, unsigned int* mcl);
```
"""
function pdv_get_shutter_method(pdv_p, mcl)
    @ccall libpdv.pdv_get_shutter_method(pdv_p::PdvDev, mcl::Ptr{Cuint})::Cint
end

"""
    pdv_get_interlace_method(pdv_p)

Returns the interleave method, as set from the **method_interlace** directive in the configuration file (from pdv_initcam()). This method is used to determine how the image data will be rearranged (if at all) before being returned from pdv_wait_images() or pdv_read().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The interleave method.
# See also
**method_interlace** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_interlace_method(PdvDev pdv_p);
```
"""
function pdv_get_interlace_method(pdv_p)
    @ccall libpdv.pdv_get_interlace_method(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_exposure(pdv_p, value)

Sets the exposure time, using the method defined by the directives in the camera configuration file, if set.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The exposure time. For AIA\\_MCL or AIA\\_MCL\\_100US, the valid range is 0-25500. For other methods, valid range and increments are camera-dependent.
# Returns
0 if successful, -1 if unsuccessful.
# See also
pdv_set_shutter_method(), pdv_get_shutter_method(), **MODE_CNTL_NORM**, **serial_exposure** and **method_camera_shutter_timing** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_exposure(PdvDev pdv_p, int value);
```
"""
function pdv_set_exposure(pdv_p, value)
    @ccall libpdv.pdv_set_exposure(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_exposure(pdv_p)

Gets the exposure time on the digital imaging device.

Applies only when using board-controlled shutter timing for which shutter timing methods have been programmed into the library. The valid range is camera-dependent.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Exposure time, in milliseconds.
# See also
pdv_set_exposure(), **method_camera_shutter_timing** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_exposure(PdvDev pdv_p);
```
"""
function pdv_get_exposure(pdv_p)
    @ccall libpdv.pdv_get_exposure(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_gain(pdv_p, value)

Sets the gain on the input device.

Applies only to cameras for which extended control capabilities have been added to the library, or that have a serial command protocol that has been configured using the **serial_gain** configuration directive. Unless you know that one of the above has been implemented for your camera, it is usually safest to just send the specific serial commands via pdv_serial_command() or pdv_serial_write().

The valid range is -128 to 128. The actual range is camera-dependent.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The gain setting.
# Returns
0 on success, -1 on failure.
# See also
pdv_get_gain(), **seriial_gain** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_gain(PdvDev pdv_p, int value);
```
"""
function pdv_set_gain(pdv_p, value)
    @ccall libpdv.pdv_set_gain(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_gain(pdv_p)

Gets the gain on the device.

Applies only to cameras for which extended control capabilities have been written into the library, such as the Kodak Megaplus i series.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The gain value.
# See also
pdv_set_gain(), **seriial_gain** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_gain(PdvDev pdv_p);
```
"""
function pdv_get_gain(pdv_p)
    @ccall libpdv.pdv_get_gain(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_blacklevel(pdv_p, value)

Sets the black level (offset) on the input device.

Applies only to cameras for which extended control capabilities have been added to the library (see the source code), or that have a serial command protocol that has been configured using the **serial_offset** configuration directive. Unless you know that one of the above has been implemented for your camera, it is usually safest to just send the specific serial commands via pdv_serial_command() or pdv_serial_write().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The black level value. The valid range is camera-dependent.
# Returns
0 on success, -1 on failure.
# See also
pdv_get_blacklevel(), **serial_offset** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_blacklevel(PdvDev pdv_p, int value);
```
"""
function pdv_set_blacklevel(pdv_p, value)
    @ccall libpdv.pdv_set_blacklevel(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_blacklevel(pdv_p)

Gets the black level (offset) on the imaging device.

Applies only to cameras for which extended control capabilities have been written into the library, such as the Kodak Megaplus i series.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The black level value.
### Prototype
```c
EDTAPI int pdv_get_blacklevel(PdvDev pdv_p);
```
"""
function pdv_get_blacklevel(pdv_p)
    @ccall libpdv.pdv_get_blacklevel(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_binning(pdv_p, xval, yval)

Set binning on the camera to the specified values, and recalculate the values that will be returned by pdv_get_width(), pdv_get_height(), and pdv_get_image_size().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `xval`: The x binning value. Usually 1, 2, 4 or 8. Default is 1.
* `yval`: The y binning value. Usually 1, 2, 4 or 8. Default is 1.
# Returns
0 on success, -1 on failure.
# See also
**serial_binning** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_binning(PdvDev pdv_p, int xval, int yval);
```
"""
function pdv_set_binning(pdv_p, xval, yval)
    @ccall libpdv.pdv_set_binning(pdv_p::PdvDev, xval::Cint, yval::Cint)::Cint
end

"""
    pdv_set_start_delay(pdv_p, delay_ms)

Set the time to wait between acquiring images.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `delay_ms`: The delay in milliseconds.
### Prototype
```c
EDTAPI void pdv_set_start_delay(PdvDev pdv_p, int delay_ms);
```
"""
function pdv_set_start_delay(pdv_p, delay_ms)
    @ccall libpdv.pdv_set_start_delay(pdv_p::PdvDev, delay_ms::Cint)::Cvoid
end

"""
    pdv_get_start_delay(pdv_p)

Get the time to wait between acquiring images.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The delay in milliseconds.
### Prototype
```c
EDTAPI int pdv_get_start_delay(PdvDev pdv_p);
```
"""
function pdv_get_start_delay(pdv_p)
    @ccall libpdv.pdv_get_start_delay(pdv_p::PdvDev)::Cint
end

"""
    pdv_enable_roi(pdv_p, flag)

Enables on-board region of interest.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `flag`: Nonzero to enable region of interest; 0 to disable it.
# Returns
0 on success, -1 on failure.
# See also
pdv_set_roi() for an example.

### Prototype
```c
EDTAPI int pdv_enable_roi(PdvDev pdv_p, int flag);
```
"""
function pdv_enable_roi(pdv_p, flag)
    @ccall libpdv.pdv_enable_roi(pdv_p::PdvDev, flag::Cint)::Cint
end

"""
    pdv_get_roi_enabled(pdv_p)

Read if region of interest (ROI) is enabled.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Nonzero if the region of interest is enabled.
### Prototype
```c
EDTAPI int pdv_get_roi_enabled(PdvDev pdv_p);
```
"""
function pdv_get_roi_enabled(pdv_p)
    @ccall libpdv.pdv_get_roi_enabled(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_roi(pdv_p, hskip, hactv, vskip, vactv)

Sets a rectangular region of interest, supporting cropping.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `hskip`: The X coordinate of the upper left corner of the region of interest.
* `hactv`: The width (number of pixels per line) of the region of interest.
* `vskip`: The Y coordinate of the upper left corner of the region of interest.
* `vactv`: The height (number of lines per frame) of the region of interest.
# Returns
0 on success, -1 on failure.
# See also
pdv_enable_roi(), **vskip**, **vactv**, **hskip**, **hactv** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_roi(PdvDev pdv_p, int hskip, int hactv, int vskip, int vactv);
```
"""
function pdv_set_roi(pdv_p, hskip, hactv, vskip, vactv)
    @ccall libpdv.pdv_set_roi(pdv_p::PdvDev, hskip::Cint, hactv::Cint, vskip::Cint, vactv::Cint)::Cint
end

"""
    pdv_auto_set_roi(pdv_p)

Set ROI to camera width/height; adjust ROI width to be a multiple of 4, and enable ROI.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_auto_set_roi(PdvDev pdv_p);
```
"""
function pdv_auto_set_roi(pdv_p)
    @ccall libpdv.pdv_auto_set_roi(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_cam_width(pdv_p, value)

Sets placeholder for original full camera frame width, unaffected by ROI changes and usually only called by pdv\\_initcam.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The width of the camera's sensor in pixels.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_set_cam_width(PdvDev pdv_p, int value);
```
"""
function pdv_set_cam_width(pdv_p, value)
    @ccall libpdv.pdv_set_cam_width(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_set_cam_height(pdv_p, value)

Sets placeholder for original full camera frame height, unaffected by ROI changes and usually only called by pdv\\_initcam.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `value`: The height of the camera's sensor in pixels.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_set_cam_height(PdvDev pdv_p, int value);
```
"""
function pdv_set_cam_height(pdv_p, value)
    @ccall libpdv.pdv_set_cam_height(pdv_p::PdvDev, value::Cint)::Cint
end

"""
    pdv_get_min_shutter(pdv_p)

Gets the minimum allowable exposure value for this camera, as set by *initcam* from the camera\\_config file **shutter_speed_min** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The minimum exposure value.
# See also
**shutter_speed_min** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_min_shutter(PdvDev pdv_p);
```
"""
function pdv_get_min_shutter(pdv_p)
    @ccall libpdv.pdv_get_min_shutter(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_max_shutter(pdv_p)

Gets the maximum allowable exposure value for this camera, as set by *initcam* from the camera\\_config file **shutter_speed_max** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The maximum exposure value.
# See also
**shutter_speed_max** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_max_shutter(PdvDev pdv_p);
```
"""
function pdv_get_max_shutter(pdv_p)
    @ccall libpdv.pdv_get_max_shutter(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_min_gain(pdv_p)

Gets the minimum allowable gain value for this camera, as set by *initcam* from the camera configuration file **gain_min** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The minimum gain value.
# See also
**gain_min** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_min_gain(PdvDev pdv_p);
```
"""
function pdv_get_min_gain(pdv_p)
    @ccall libpdv.pdv_get_min_gain(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_max_gain(pdv_p)

Gets the maximum allowable gain value for this camera, as set by *initcam* from the camera configuration file **gain_max** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The maximum gain value.
# See also
**gain_max** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_max_gain(PdvDev pdv_p);
```
"""
function pdv_get_max_gain(pdv_p)
    @ccall libpdv.pdv_get_max_gain(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_min_offset(pdv_p)

Gets the minimum allowable offset (black level) value for this camera, as set by *initcam* from the camera configuration file **offset_min** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The minimum offset value.
# See also
**offset_min** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_min_offset(PdvDev pdv_p);
```
"""
function pdv_get_min_offset(pdv_p)
    @ccall libpdv.pdv_get_min_offset(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_max_offset(pdv_p)

Gets the maximum allowable offset (black level) value for this camera, as set by *initcam* from the camera configuration file **offset_max** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The maximum offset value.
# See also
**offset_max** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_max_offset(PdvDev pdv_p);
```
"""
function pdv_get_max_offset(pdv_p)
    @ccall libpdv.pdv_get_max_offset(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_invert(pdv_p, val)

Tell the EDT framegrabber hardware to invert each pixel before transferring it to the host computer's memory.

This is implemented in firmware and has no impact on performance.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `val`: Nonzero to enable invert mode; 0 to disable it.
### Prototype
```c
EDTAPI void pdv_set_invert(PdvDev pdv_p, int val);
```
"""
function pdv_set_invert(pdv_p, val)
    @ccall libpdv.pdv_set_invert(pdv_p::PdvDev, val::Cint)::Cvoid
end

"""
    pdv_get_invert(pdv_p)

Get the state of the hardware invert register enable bit. See pdv_set_invert for details on this feature.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Nonzero if invert mode is enabled.
### Prototype
```c
EDTAPI int pdv_get_invert(PdvDev pdv_p);
```
"""
function pdv_get_invert(pdv_p)
    @ccall libpdv.pdv_get_invert(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_firstpixel_counter(pdv_p, val)

Enable hardware overwrite of first two bytes of the frame with a counter.

The counter increments by one for every frame received by the framegrabber. Disabling this also resets the counter to zero, unless framesync mode is also enabled (see pdv_enable_framesync()).

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `val`: Nonzero to enable the counter; 0 to disable it.
### Prototype
```c
EDTAPI void pdv_set_firstpixel_counter(PdvDev pdv_p, int val);
```
"""
function pdv_set_firstpixel_counter(pdv_p, val)
    @ccall libpdv.pdv_set_firstpixel_counter(pdv_p::PdvDev, val::Cint)::Cvoid
end

"""
    pdv_get_firstpixel_counter(pdv_p)

Query state of the hardware first pixel counter register enable bit. See pdv_set_firstpixel_counter() for details on this feature.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Nonzero if the counter is enabled.
### Prototype
```c
EDTAPI int pdv_get_firstpixel_counter(PdvDev pdv_p);
```
"""
function pdv_get_firstpixel_counter(pdv_p)
    @ccall libpdv.pdv_get_firstpixel_counter(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_header_type(pdv_p, header_type, irig_offset, irig_raw)

Sets the header (or footer) type.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `header_type`: Header type, from the edt_pdv_header_type enum.
* `irig_offset`: Timecode offset in seconds. Typically set to 2 seconds. Ignored if header\\_type is PDV_HDR_TYPE_NONE.
* `irig_raw`: 0 = Enable UNIX timecode format. 1 = Enable raw timecode format. Ignored if header\\_type is PDV_HDR_TYPE_NONE.
# Returns
0 in success, -1 on failure.
# See also
pdv_set_header_size(), **method_header_type** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>, and the Timestamp appendix in the User's Guide.

### Prototype
```c
EDTAPI int pdv_set_header_type(PdvDev pdv_p, EdtPdvHeaderType header_type, int irig_offset, int irig_raw);
```
"""
function pdv_set_header_type(pdv_p, header_type, irig_offset, irig_raw)
    @ccall libpdv.pdv_set_header_type(pdv_p::PdvDev, header_type::EdtPdvHeaderType, irig_offset::Cint, irig_raw::Cint)::Cint
end

"""
    pdv_get_header_type(pdv_p)

Get the header (or footer) type.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The header type as an [`edt_pdv_header_type`](@ref) enum.
# See also
pdv_set_header_type(), **method_header_type** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI EdtPdvHeaderType pdv_get_header_type(PdvDev pdv_p);
```
"""
function pdv_get_header_type(pdv_p)
    @ccall libpdv.pdv_get_header_type(pdv_p::PdvDev)::EdtPdvHeaderType
end

"""
    pdv_set_header_size(pdv_p, header_size)

Sets the header (or footer) size, in bytes, for the device.

This can also be done by using the **header_size** directive in the camera configuration file.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `header_size`: The new header size in bytes.
# See also
pdv_get_header_size(), **header_size** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI void pdv_set_header_size(PdvDev pdv_p, int header_size);
```
"""
function pdv_set_header_size(pdv_p, header_size)
    @ccall libpdv.pdv_set_header_size(pdv_p::PdvDev, header_size::Cint)::Cvoid
end

"""
    pdv_get_header_size(pdv_p)

Returns the currently defined header or footer size.

This is usually set in the configuration file with the directive **header_size**. It can also be set by calling pdv_set_header_size().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
Current header size.
# See also
pdv_set_header_size(), **header_size** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_header_size(PdvDev pdv_p);
```
"""
function pdv_get_header_size(pdv_p)
    @ccall libpdv.pdv_get_header_size(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_header_position(pdv_p, header_position)

Sets the header (or footer) position.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `header_position`: The starting point for the header.
# See also
pdv_get_header_offset(), pdv_set_header_offset(), EdtPdvHeaderPosition.

### Prototype
```c
EDTAPI void pdv_set_header_position(PdvDev pdv_p, EdtPdvHeaderPosition header_position);
```
"""
function pdv_set_header_position(pdv_p, header_position)
    @ccall libpdv.pdv_set_header_position(pdv_p::PdvDev, header_position::EdtPdvHeaderPosition)::Cvoid
end

"""
    pdv_get_header_position(pdv_p)

Returns the header or footer position value.

These values can be set in the configuration file with the **method_header_position** directive.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The header position.
# See also
pdv_get_header_offset(), **method_header_position** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI EdtPdvHeaderPosition pdv_get_header_position(PdvDev pdv_p);
```
"""
function pdv_get_header_position(pdv_p)
    @ccall libpdv.pdv_get_header_position(pdv_p::PdvDev)::EdtPdvHeaderPosition
end

"""
    pdv_set_header_offset(pdv_p, header_offset)

Sets the byte offset of the header data in the allocated buffer.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `header_offset`: The header offset.
### Prototype
```c
EDTAPI void pdv_set_header_offset(PdvDev pdv_p, int header_offset);
```
"""
function pdv_set_header_offset(pdv_p, header_offset)
    @ccall libpdv.pdv_set_header_offset(pdv_p::PdvDev, header_offset::Cint)::Cvoid
end

"""
    pdv_get_header_offset(pdv_p)

Returns the byte offset of the header in the buffer.

The byte offset is determined by the header position value. If header\\_position is PDV\\_HEADER\\_BEFORE, the offset is 0; if header\\_position is PDV\\_HEADER\\_AFTER (i.e. not really a header but a footer), the offset is the image size. If header\\_position is PDV\\_HEADER\\_MIDDLE, the header offset can be set using the **header_offset** directive in the camera\\_configuration file, or by calling pdv_set_header_offset().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
A byte offset from the beginning of the buffer.
# See also
pdv_get_header_position(), pdv_set_header_offset()

### Prototype
```c
EDTAPI int pdv_get_header_offset(PdvDev pdv_p);
```
"""
function pdv_get_header_offset(pdv_p)
    @ccall libpdv.pdv_get_header_offset(pdv_p::PdvDev)::Cint
end

"""
    pdv_set_header_dma(pdv_p, header_dma)

Sets the boolean value for whether the image header is included in the DMA from the camera.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `header_dma`: True to include the header, false if not.
# See also
pdv_get_header_dma()

### Prototype
```c
EDTAPI void pdv_set_header_dma(PdvDev pdv_p, bool header_dma);
```
"""
function pdv_set_header_dma(pdv_p, header_dma)
    @ccall libpdv.pdv_set_header_dma(pdv_p::PdvDev, header_dma::Bool)::Cvoid
end

"""
    pdv_get_header_dma(pdv_p)

Returns the current setting for flag which determines whether the header (or footer) size is to be added to the DMA size. This is true if the camera/device returns header information at the beginning or end of its transfer.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
True if header is included in DMA size.
### Prototype
```c
EDTAPI bool pdv_get_header_dma(PdvDev pdv_p);
```
"""
function pdv_get_header_dma(pdv_p)
    @ccall libpdv.pdv_get_header_dma(pdv_p::PdvDev)::Bool
end

"""
    pdv_set_image_size(pdv_p, width, height)

Sets the width and height of the image. Tells the driver what width and height (in pixels) to expect from the camera.

This call is ordinarily unnecessary in an application program, because the width and height are set automatically when *initcam* runs. Exceptions can occur, however; for example, if the camera's output size can be changed while running, or if the application performs setup that supersedes initcam. This routine is provided for these special cases.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `width`: The width of the image in pixels.
* `height`: The height of the image in pixels.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_set_image_size(PdvDev pdv_p, int width, int height);
```
"""
function pdv_set_image_size(pdv_p, width, height)
    @ccall libpdv.pdv_set_image_size(pdv_p::PdvDev, width::Cint, height::Cint)::Cint
end

"""
    pdv_get_image_size(pdv_p)

Returns the size of the image in bytes, absent any padding or header data. Since padding and header data are usually absent, the value returned from this is usually the same as that returned by pdv_get_imghdr_size().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The image size, in bytes.
# See also
pdv_get_imghdr_size()

### Prototype
```c
EDTAPI int pdv_get_image_size(PdvDev pdv_p);
```
"""
function pdv_get_image_size(pdv_p)
    @ccall libpdv.pdv_get_image_size(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_imghdr_size(pdv_p)

Returns the size of the image buffer in bytes, based on its width, height, and depth. The size returned includes allowance for buffer headers.

Enabling a region of interest changes this value. To obtain the actual size of the image data without any optional header or other padding, see pdv_get_dma_size().

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The total number of bytes in the image, including buffer header overhead.
# See also
pdv_set_roi()

### Prototype
```c
EDTAPI int pdv_get_imghdr_size(PdvDev pdv_p);
```
"""
function pdv_get_imghdr_size(pdv_p)
    @ccall libpdv.pdv_get_imghdr_size(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_dma_size(pdv_p)

Returns the actual amount of image data for DMA.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The DMA size in bytes.
# See also
pdv_get_imghdr_size(), pdv_get_header_position(), pdv_extra_headersize().

### Prototype
```c
EDTAPI int pdv_get_dma_size(PdvDev pdv_p);
```
"""
function pdv_get_dma_size(pdv_p)
    @ccall libpdv.pdv_get_dma_size(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_fulldma_size(pdv_p, extrasizep)

### Prototype
```c
EDTAPI int pdv_get_fulldma_size(PdvDev pdv_p, int* extrasizep);
```
"""
function pdv_get_fulldma_size(pdv_p, extrasizep)
    @ccall libpdv.pdv_get_fulldma_size(pdv_p::PdvDev, extrasizep::Ptr{Cint})::Cint
end

"""
    pdv_get_cam_width(pdv_p)

Returns the camera image width, in pixels, as set by the configuration file directive **width**.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The image width in pixels.
# See also
pdv_get_width(), **width** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_cam_width(PdvDev pdv_p);
```
"""
function pdv_get_cam_width(pdv_p)
    @ccall libpdv.pdv_get_cam_width(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_cam_height(pdv_p)

Returns the camera image height, in pixels, as set by the configuration file directive **height**, unaffected by changes made by setting a region of interest. See pdv_set_roi() for more information.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The image height in pixels.
# See also
pdv_get_height(), **height** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_cam_height(PdvDev pdv_p);
```
"""
function pdv_get_cam_height(pdv_p)
    @ccall libpdv.pdv_get_cam_height(pdv_p::PdvDev)::Cint
end

"""
    pdv_check_framesync(pdv_p, image_p, framecnt)

Checks for frame sync and frame count.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `image_p`: A pointer to previously acquired image (from pdv_wait_images() for example) for which you want the framesync to be checked.
* `framecnt`: A pointer to location to put frame counter from this frame.
# Returns
1 if out of sync is detected, 0 if no out of sync detected, -1 on error.
# See also
pdv_enable_framesync(), pdv_framesync_mode()

### Prototype
```c
EDTAPI int pdv_check_framesync(PdvDev pdv_p, uint8_t* image_p, uint32_t* framecnt);
```
"""
function pdv_check_framesync(pdv_p, image_p, framecnt)
    @ccall libpdv.pdv_check_framesync(pdv_p::PdvDev, image_p::Ptr{UInt8}, framecnt::Ptr{UInt32})::Cint
end

"""
    pdv_enable_framesync(pdv_p, mode)

Enables frame sync footer and frame out-of-synch detection.

* `PDV_FRAMESYNC_OFF`: Framesync functionality disabled.

* `PDV_FRAMESYNC_ON`: Framesync functionality enabled.

* `PDV_FRAMESYNC_EMULATE_TIMEOUT`: Framesync errors will be reflected as timeouts.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `mode`: The frame sync mode should be one of:
# Returns
0 on success, -1 if not supported by the device in use.
# See also
pdv_check_framesync(), pdv_framesync_mode(), pdv_timeouts()

### Prototype
```c
EDTAPI int pdv_enable_framesync(PdvDev pdv_p, int mode);
```
"""
function pdv_enable_framesync(pdv_p, mode)
    @ccall libpdv.pdv_enable_framesync(pdv_p::PdvDev, mode::Cint)::Cint
end

"""
    pdv_framesync_mode(pdv_p)

Returns the frame sync mode.

Can be one of: - `PDV_FRAMESYNC_OFF`: Framesync functionality disabled. - `PDV_FRAMESYNC_ON`: Framesync functionality enabled. - `PDV_FRAMESYNC_EMULATE_TIMEOUT`: Framesync functionality enabled, and framesync errors will be reflected as timeouts.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
1 if enabled, 0 if not enabled.
# See also
pdv_enable_framesync(), pdv_check_framesync()

### Prototype
```c
EDTAPI int pdv_framesync_mode(PdvDev pdv_p);
```
"""
function pdv_framesync_mode(pdv_p)
    @ccall libpdv.pdv_framesync_mode(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_cfgname(pdv_p)

Get the configuration file name.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The configuration file name.
### Prototype
```c
EDTAPI const char* pdv_get_cfgname(PdvDev pdv_p);
```
"""
function pdv_get_cfgname(pdv_p)
    @ccall libpdv.pdv_get_cfgname(pdv_p::PdvDev)::Cstring
end

"""
    pdv_set_defaults(pdv_p)

Set exposure, gain, and blacklevel to default values.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
### Prototype
```c
EDTAPI void pdv_set_defaults(PdvDev pdv_p);
```
"""
function pdv_set_defaults(pdv_p)
    @ccall libpdv.pdv_set_defaults(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_enable_external_trigger(pdv_p, flag)

Enables external triggering.

* 0 -- turn off trigger

* 1 -- turn on photo trigger

* 2 -- turn on field ID trigger (through camera or cable). Does not apply to PCI C-Link.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `flag`: One of:
### Prototype
```c
EDTAPI void pdv_enable_external_trigger(PdvDev pdv_p, int flag);
```
"""
function pdv_enable_external_trigger(pdv_p, flag)
    @ccall libpdv.pdv_enable_external_trigger(pdv_p::PdvDev, flag::Cint)::Cvoid
end

"""
    pdv_set_frame_period(pdv_p, rate, method)

Set the frame period counter and enable/disable frame timing.

* 0 -- disable frame counter

* `PDV_FMRATE_ENABLE` -- continuous frame counter

* `PDV_FVAL_ADJUST` -- frame counter extends every frame valid by 'period' microseconds

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `period`: The frame period in microseconds-2, range 0-16777215
* `method`: One of:
# Returns
-1 on error, 0 on success.
# See also
pdv_get_frame_period(), **frame_period**, **method_frame_timing** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_set_frame_period(PdvDev pdv_p, int rate, int method);
```
"""
function pdv_set_frame_period(pdv_p, rate, method)
    @ccall libpdv.pdv_set_frame_period(pdv_p::PdvDev, rate::Cint, method::Cint)::Cint
end

"""
    pdv_get_frame_period(pdv_p)

Get the frame period.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The frame period in microseconds.
# See also
pdv_set_frame_period(), **frame_period** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_frame_period(PdvDev pdv_p);
```
"""
function pdv_get_frame_period(pdv_p)
    @ccall libpdv.pdv_get_frame_period(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_htaps_vtaps(pdv_p, htaps, vtaps)

Get `htaps` and `vtaps` setting values.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `htaps`:\\[out\\] On success, the current horizontal taps value is returned here. Pass NULL to ignore.
* `vtaps`:\\[out\\] On success, the current vertical taps value is returned here. Pass NULL to ignore.
# Returns
0 on success.
### Prototype
```c
EDTAPI int pdv_get_htaps_vtaps(PdvDev pdv_p, int* htaps, int* vtaps);
```
"""
function pdv_get_htaps_vtaps(pdv_p, htaps, vtaps)
    @ccall libpdv.pdv_get_htaps_vtaps(pdv_p::PdvDev, htaps::Ptr{Cint}, vtaps::Ptr{Cint})::Cint
end

"""
    pdv_get_hskip_vskip(pdv_p, hskip, vskip)

Get `hskip` and `vskip` setting values.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `hskip`:\\[out\\] On success, the current horizontal skip value is returned here. Pass NULL to ignore.
* `vskip`:\\[out\\] On success, the current vertical skip value is returned here. Pass NULL to ignore.
# Returns
0 on success.
### Prototype
```c
EDTAPI int pdv_get_hskip_vskip(PdvDev pdv_p, int* hskip, int* vskip);
```
"""
function pdv_get_hskip_vskip(pdv_p, hskip, vskip)
    @ccall libpdv.pdv_get_hskip_vskip(pdv_p::PdvDev, hskip::Ptr{Cint}, vskip::Ptr{Cint})::Cint
end

"""
    pdv_get_hactv_vactv(pdv_p, hactv, vactv)

Get `hactv` and `vactv` setting values.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `hactv`:\\[out\\] On success, the current horizontal active pixels value is returned here. Pass NULL to ignore.
* `vactv`:\\[out\\] On success, the current vertical active pixels value is returned here. Pass NULL to ignore.
# Returns
0 on success.
### Prototype
```c
EDTAPI int pdv_get_hactv_vactv(PdvDev pdv_p, int* hactv, int* vactv);
```
"""
function pdv_get_hactv_vactv(pdv_p, hactv, vactv)
    @ccall libpdv.pdv_get_hactv_vactv(pdv_p::PdvDev, hactv::Ptr{Cint}, vactv::Ptr{Cint})::Cint
end

"""
    pdv_get_fv_once(pdv_p)

Get value of `fv_once` (frame valid once) flag.

pdv\\_p The open [`PDV`](@ref) device handle.

# Returns
1 if `fv_once` is true, 0 if false or on error.
### Prototype
```c
EDTAPI int pdv_get_fv_once(PdvDev pdv_p);
```
"""
function pdv_get_fv_once(pdv_p)
    @ccall libpdv.pdv_get_fv_once(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_set_baud(pdv_p, baud)

Sets the baud rate on the serial lines; applies only to cameras with serial control. Valid values are 9600, 19200, 38500, 57500, and 115200.

!!! note

    The baud rate is ordinarily initialized using the value of the **serial_baud** directive in the configuration file, and defaults to 9600 if the directive is not present. Under most circumstances, applications do not need to set the baud rate explicitly.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `baud`: The desired baud rate.
# Returns
0 on success, -1 on error.
# See also
**serial_baud** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_set_baud(PdvDev pdv_p, int baud);
```
"""
function pdv_serial_set_baud(pdv_p, baud)
    @ccall libpdv.pdv_serial_set_baud(pdv_p::PdvDev, baud::Cint)::Cint
end

"""
    pdv_serial_get_baud(pdv_p)

Get the baud rate, typically initialized by the **serial_baud** directive in the config file.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The baud rate in bits/sec, or 0 on error.
# See also
**serial_baud** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_get_baud(PdvDev pdv_p);
```
"""
function pdv_serial_get_baud(pdv_p)
    @ccall libpdv.pdv_serial_get_baud(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_command(pdv_p, cmd)

Sends an ASCII serial command to the camera, with ASCII camera command formatting. Applies only to cameras that use a serial control method for camera-computer communications.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `cmd`: The command to send to the camera.
# Returns
0 on success, -1 on failure
# See also
pdv_serial_term(), pdv_serial_prefix(), pdv_serial_set_delimiters(), pdv_serial_write()

### Prototype
```c
EDTAPI int pdv_serial_command(PdvDev pdv_p, const char* cmd);
```
"""
function pdv_serial_command(pdv_p, cmd)
    @ccall libpdv.pdv_serial_command(pdv_p::PdvDev, cmd::Cstring)::Cint
end

"""
    pdv_serial_binary_command(pdv_p, cmd, len)

Sends a binary serial command to the camera. Applies only to cameras that use a serial control method for for camera-computer communications.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `cmd`: A buffer containing the serial command.
* `len`: The number of bytes to send.
# Returns
0 on success, -1 on failure.
# See also
pdv_serial_command(), pdv_serial_read(), pdv_serial_wait()

### Prototype
```c
EDTAPI int pdv_serial_binary_command(PdvDev pdv_p, const uint8_t* cmd, int len);
```
"""
function pdv_serial_binary_command(pdv_p, cmd, len)
    @ccall libpdv.pdv_serial_binary_command(pdv_p::PdvDev, cmd::Ptr{UInt8}, len::Cint)::Cint
end

"""
    pdv_serial_write_available(pdv_p)

Get the number of bytes available in the driver's serial write buffer.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of bytes available in the driver's write buffer.
### Prototype
```c
EDTAPI int pdv_serial_write_available(PdvDev pdv_p);
```
"""
function pdv_serial_write_available(pdv_p)
    @ccall libpdv.pdv_serial_write_available(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_write(pdv_p, buf, size)

Performs a serial write over the serial lines. Not recommended for use in new code, consider using pdv_serial_command() or pdv_serial_binary_command() instead.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`: The buffer containing the serial command.
* `size`: The number of bytes to send.
# Returns
0 on success, -1 on failure.
# See also
pdv_serial_command(), pdv_serial_binary_command()

### Prototype
```c
EDTAPI int pdv_serial_write(PdvDev pdv_p, const char* buf, int size);
```
"""
function pdv_serial_write(pdv_p, buf, size)
    @ccall libpdv.pdv_serial_write(pdv_p::PdvDev, buf::Cstring, size::Cint)::Cint
end

"""
    pdv_serial_read(pdv_p, buf, size)

Performs a serial read over the serial control lines.

The serial data read will be stored in a user supplied buffer. That buffer will be NULL-terminated. Use [`pdv_serial_read_nullterm`](@ref)(pdv\\_p, [`FALSE`](@ref)) if you don't want that behavior.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`:\\[out\\] The buffer to receive the data. It must be at least `size` + 1 bytes (`size` bytes of data plus a one byte NULL terminator).
* `size`: The number of bytes to be read.
# Returns
The number of bytes read into the buffer.
# See also
pdv_serial_wait()

### Prototype
```c
EDTAPI int pdv_serial_read(PdvDev pdv_p, char* buf, int size);
```
"""
function pdv_serial_read(pdv_p, buf, size)
    @ccall libpdv.pdv_serial_read(pdv_p::PdvDev, buf::Cstring, size::Cint)::Cint
end

"""
    pdv_serial_read_blocking(pdv_p, buf, size)

Performs a serial read over the serial control lines, blocks until all requested data is read.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`:\\[out\\] The buffer to receive the data. It must be at least `size` + 1 bytes (`size` bytes of data plus a one byte NULL terminator).
* `size`: The number of bytes to be read.
# Returns
The number of bytes read into the buffer.
# See also
pdv_serial_wait(), pdv_serial_read()

### Prototype
```c
EDTAPI int pdv_serial_read_blocking(PdvDev pdv_p, char* buf, int size);
```
"""
function pdv_serial_read_blocking(pdv_p, buf, size)
    @ccall libpdv.pdv_serial_read_blocking(pdv_p::PdvDev, buf::Cstring, size::Cint)::Cint
end

"""
    pdv_serial_read_nullterm(pdv_p, buf, size, nullterm)

Performs a serial read over the serial control lines. The buffer passed in will be NULL-terminated if nullterm is true.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `buf`:\\[out\\] The buffer to receive the data. It must be at least `size` + 1 bytes (`size` bytes of data plus a one byte NULL terminator).
* `size`: The number of bytes to be read.
* `nullterm`: True to null terminate the buffer read in, false to disable that.
# Returns
The number of bytes read into buf.
# See also
pdv_serial_read()

### Prototype
```c
EDTAPI int pdv_serial_read_nullterm(PdvDev pdv_p, char* buf, int size, int nullterm);
```
"""
function pdv_serial_read_nullterm(pdv_p, buf, size, nullterm)
    @ccall libpdv.pdv_serial_read_nullterm(pdv_p::PdvDev, buf::Cstring, size::Cint, nullterm::Cint)::Cint
end

"""
    pdv_serial_read_enable(pdv_p)

### Prototype
```c
EDTAPI int pdv_serial_read_enable(PdvDev pdv_p);
```
"""
function pdv_serial_read_enable(pdv_p)
    @ccall libpdv.pdv_serial_read_enable(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_read_disable(pdv_p)

### Prototype
```c
EDTAPI int pdv_serial_read_disable(PdvDev pdv_p);
```
"""
function pdv_serial_read_disable(pdv_p)
    @ccall libpdv.pdv_serial_read_disable(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_set_delimiters(pdv_p, newprefix, newterm)

Set the prefix and terminator for serial commands.

The serial prefix and terminator are typically set through the **serial_term** and **serial_prefix** config file directives.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `newprefix`: A string to prefix to serial commands, or NULL.
* `newterm`: A string to append to serial commands, or NULL.
# See also
pdv_serial_command(), **serial_prefix** and **serial_term** directives in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI void pdv_serial_set_delimiters(PdvDev pdv_p, const char* newprefix, const char* newterm);
```
"""
function pdv_serial_set_delimiters(pdv_p, newprefix, newterm)
    @ccall libpdv.pdv_serial_set_delimiters(pdv_p::PdvDev, newprefix::Cstring, newterm::Cstring)::Cvoid
end

"""
    pdv_serial_prefix(pdv_p)

Get the serial prefix.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The serial prefix string.
# See also
pdv_serial_command(), **serial_prefix** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI const char* pdv_serial_prefix(PdvDev pdv_p);
```
"""
function pdv_serial_prefix(pdv_p)
    @ccall libpdv.pdv_serial_prefix(pdv_p::PdvDev)::Cstring
end

"""
    pdv_serial_term(pdv_p)

Get the serial terminator.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The serial terminator string.
# See also
pdv_serial_command(), **serial_term** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI const char* pdv_serial_term(PdvDev pdv_p);
```
"""
function pdv_serial_term(pdv_p)
    @ccall libpdv.pdv_serial_term(pdv_p::PdvDev)::Cstring
end

"""
    pdv_serial_send_break(pdv_p)

Send a break condition on the serial connection.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
### Prototype
```c
EDTAPI void pdv_serial_send_break(PdvDev pdv_p);
```
"""
function pdv_serial_send_break(pdv_p)
    @ccall libpdv.pdv_serial_send_break(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_serial_reset(pdv_p)

Resets the serial interface. Clears any outstanding reads and writes and puts the serial interface in a known idle state.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
### Prototype
```c
EDTAPI void pdv_serial_reset(PdvDev pdv_p);
```
"""
function pdv_serial_reset(pdv_p)
    @ccall libpdv.pdv_serial_reset(pdv_p::PdvDev)::Cvoid
end

"""
    pdv_serial_read_basler(pdv_p, cmd, len)

Reads a Basler binary frame command. Checks the framing and BCC.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `cmd`:\\[out\\] The buffer to receive the data.
* `len`: The maximum number of bytes to be read.
# Returns
The number of characters read back, or 0 if none or failure.
# See also
BASLER A202K Camera Manual Doc. ID number DA044003

### Prototype
```c
EDTAPI int pdv_serial_read_basler(PdvDev pdv_p, uint8_t* cmd, int len);
```
"""
function pdv_serial_read_basler(pdv_p, cmd, len)
    @ccall libpdv.pdv_serial_read_basler(pdv_p::PdvDev, cmd::Ptr{UInt8}, len::Cint)::Cint
end

"""
    pdv_serial_write_basler(pdv_p, cmd, len)

Send a Basler formatted serial frame. Adds the framing and BCC.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `cmd`: The serial data to send.
* `len`: The number of bytes to send.
# Returns
0 on success, -1 on failure.
# See also
BASLER A202K Camera Manual Doc. ID number DA044003

### Prototype
```c
EDTAPI int pdv_serial_write_basler(PdvDev pdv_p, uint8_t* cmd, int len);
```
"""
function pdv_serial_write_basler(pdv_p, cmd, len)
    @ccall libpdv.pdv_serial_write_basler(pdv_p::PdvDev, cmd::Ptr{UInt8}, len::Cint)::Cint
end

"""
    pdv_serial_read_duncan(pdv_p, frame)

Read a binary serial response from a Duncantech MS and DT series camera -- checks for STX and size, then waits for size+1 more bytes.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `frame`:\\[out\\] The buffer to receive the data.
# Returns
The number of characters read back, or 0 if none or failure.
# See also
pdv_serial_write_duncan(), DuncanTech User Manual Doc # 9000-0001-05.

### Prototype
```c
EDTAPI int pdv_serial_read_duncan(PdvDev pdv_p, uint8_t* frame);
```
"""
function pdv_serial_read_duncan(pdv_p, frame)
    @ccall libpdv.pdv_serial_read_duncan(pdv_p::PdvDev, frame::Ptr{UInt8})::Cint
end

"""
    pdv_serial_write_duncan(pdv_p, cmdbuf, size)

Send a Duncantech MS / DT series camera frame -- adds the framing and checksum, then sends the command.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `cmdbuf`: A buffer containing the command, minus framing information.
* `size`: The number of bytes in the cmdbuf.
# Returns
0 on success, -1 on failure.
# See also
pdv_serial_read_duncan(), DuncanTech User Manual Doc # 9000-0001-05.

### Prototype
```c
EDTAPI int pdv_serial_write_duncan(PdvDev pdv_p, uint8_t* cmdbuf, int size);
```
"""
function pdv_serial_write_duncan(pdv_p, cmdbuf, size)
    @ccall libpdv.pdv_serial_write_duncan(pdv_p::PdvDev, cmdbuf::Ptr{UInt8}, size::Cint)::Cint
end

"""
    pdv_serial_wait(pdv_p, msecs, count)

Waits for a response from the camera as a result of a pdv_serial_write() or pdv_serial_command().

After calling this function, use pdv_serial_read() to get the data. For a detailed example of serial communications, see the serial\\_cmd.c example program.

If the timeout value is zero, the default from the **serial_timeout** directive in the config file is used. If no default timeout value was specified, the timeout will be 1000 milliseconds.

If two threads attempt to call [`pdv_serial_wait`](@ref)() at the same time, one will return immediately with a result of -EBUSY.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `msecs`: Number of milliseconds to wait before timing out.
* `count`: The maximum number of bytes to wait for before returning.
# Returns
On success or timeout, the number of bytes of serial data returned from the camera. Negative error code on error.
# See also
pdv_serial_read(), **serial_timeout** directive in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_wait(PdvDev pdv_p, int msecs, int count);
```
"""
function pdv_serial_wait(pdv_p, msecs, count)
    @ccall libpdv.pdv_serial_wait(pdv_p::PdvDev, msecs::Cint, count::Cint)::Cint
end

"""
    pdv_serial_wait_next(pdv_p, msecs, count)

Wait for new serial data to come in. Ignore any previously received data.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `msecs`: Number of milliseconds to wait before timing out.
* `count`: The maximum number of bytes to wait for before returning.
# Returns
The number of bytes of serial data returned from the camera.
### Prototype
```c
EDTAPI int pdv_serial_wait_next(PdvDev pdv_p, int msecs, int count);
```
"""
function pdv_serial_wait_next(pdv_p, msecs, count)
    @ccall libpdv.pdv_serial_wait_next(pdv_p::PdvDev, msecs::Cint, count::Cint)::Cint
end

"""
    pdv_serial_set_waitchar(pdv_p, enable, wchar)

Set the serial wait character.

Normally pdv_serial_wait() will wait until the serial\\_timeout period expires before returning (unless the max number of characters is seen). This is the most general purpose and robust method since there's no other way of knowing all different camera response formats. However if each response can be expected to be 1 line terminated by the same character(such as a newline) every time, then setting the serial\\_waitchar to that character can greatly shorten the time it takes for a pdv_serial_wait() call to return.

This character can also be initialized in the <a href="https://edt.com/downloads/camera-configuration-guide.pdf">camera configuration</A> directive **serial_waitchar**.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `enable`: Nonzero to enable wait character detection.
* `wchar`: The end of message character to detect.
# Returns
0 in success, -1 on failure
# See also
pdv_serial_get_waitchar() and **serial_waitchar** directive in the <a>* href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_set_waitchar(PdvDev pdv_p, int enable, unsigned char wchar);
```
"""
function pdv_serial_set_waitchar(pdv_p, enable, wchar)
    @ccall libpdv.pdv_serial_set_waitchar(pdv_p::PdvDev, enable::Cint, wchar::Cuchar)::Cint
end

"""
    pdv_serial_get_waitchar(pdv_p, wchar)

Get serial wait character.

If wait character detection is enabled, pdv_serial_wait() will return immediately after the wait character is detected.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
* `wchar`:\\[out\\] The character wait character detection will wait for.
# Returns
1 if waitchar enabled, 0 if disabled.
# See also
pdv_serial_set_waitchar() and **serial_waitchar** directive in the <a>* href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_get_waitchar(PdvDev pdv_p, unsigned char* wchar);
```
"""
function pdv_serial_get_waitchar(pdv_p, wchar)
    @ccall libpdv.pdv_serial_get_waitchar(pdv_p::PdvDev, wchar::Ptr{Cuchar})::Cint
end

"""
    pdv_serial_get_numbytes(pdv_p)

Returns the number of bytes of unread data in the serial response buffer.

Similar to pdv_serial_wait() but doesn't wait for any timeout period, nor does it have any minimum count parameter.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The number of bytes of serial data returned from the camera.
### Prototype
```c
EDTAPI int pdv_serial_get_numbytes(PdvDev pdv_p);
```
"""
function pdv_serial_get_numbytes(pdv_p)
    @ccall libpdv.pdv_serial_get_numbytes(pdv_p::PdvDev)::Cint
end

"""
    pdv_serial_get_block_size()

Returns the block size for serial writes.

# Returns
The serial block size in bytes.
### Prototype
```c
EDTAPI int pdv_serial_get_block_size(void);
```
"""
function pdv_serial_get_block_size()
    @ccall libpdv.pdv_serial_get_block_size()::Cint
end

"""
    pdv_serial_set_block_size(newsize)

Sets the block size for serial writes.

The default size is 1024.

# Arguments
* `newsize`: The new serial block size in bytes.
### Prototype
```c
EDTAPI void pdv_serial_set_block_size(int newsize);
```
"""
function pdv_serial_set_block_size(newsize)
    @ccall libpdv.pdv_serial_set_block_size(newsize::Cint)::Cvoid
end

"""
    pdv_serial_get_timeout(pdv_p)

Get the value of the serial timeout.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The timeout in milliseconds.
# See also
**serial_timeout** directive in the <a>* href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_serial_get_timeout(PdvDev pdv_p);
```
"""
function pdv_serial_get_timeout(pdv_p)
    @ccall libpdv.pdv_serial_get_timeout(pdv_p::PdvDev)::Cint
end

"""
    pdv_get_pause_for_serial(pdv_p)

Get the serial pause time, in milliseconds.

Set by the `pause_for_serial` directive in a camera configuration file.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
The serial pause time in milliseconds.
# See also
**pause_for_serial** directive in the <a>* href="https://edt.com/downloads/camera-configuration-guide.pdf"> Camera Configuration Guide</a>.

### Prototype
```c
EDTAPI int pdv_get_pause_for_serial(PdvDev pdv_p);
```
"""
function pdv_get_pause_for_serial(pdv_p)
    @ccall libpdv.pdv_get_pause_for_serial(pdv_p::PdvDev)::Cint
end

"""
    pdv_free(ptr)

Convenience routine to free the memory allocated with pdv_alloc().

# Arguments
* `ptr`: Address of memory buffer to free.
### Prototype
```c
EDTAPI void pdv_free(void* ptr);
```
"""
function pdv_free(ptr)
    @ccall libpdv.pdv_free(ptr::Ptr{Cvoid})::Cvoid
end

"""
    pdv_alloc(size)

Convenience routine to allocate memory in a system-independent way.

The buffer returned is page aligned. Page alignment is required for some EDT image routines and always preferred. This function uses VirtualAlloc on Windows NT/2000/XP systems, or posix\\_memalign on Linux/Unix systems.

# Arguments
* `size`: The number of bytes of memory to allocate
# Returns
The address of the allocated memory, or NULL on error. If NULL, use edt_msg_perror() to print the error.
# See also
pdv_free()

### Prototype
```c
EDTAPI void* pdv_alloc(int size);
```
"""
function pdv_alloc(size)
    @ccall libpdv.pdv_alloc(size::Cint)::Ptr{Cvoid}
end

"""
    pdv_access(fname, perm)

Determines file access independent of operating system. This a convenience routine that maps to `access`() on Unix/Linux systems, and `_access` on Windows systems.

# Arguments
* `fname`: The path name of the file to check access of.
* `perm`: The permission flag(s) to test for. See `access`() (Unix/Linux) or `_access` (Windows) for valid arguments.
# Returns
0 on success, -1 on failure.
### Prototype
```c
EDTAPI int pdv_access(const char* fname, int perm);
```
"""
function pdv_access(fname, perm)
    @ccall libpdv.pdv_access(fname::Cstring, perm::Cint)::Cint
end

"""
    pdv_mark_ras_depth(addr, n, width, height, x, y, depth, fg)

Draws the digits of a number into an image buffer.

# Arguments
* `addr`: The address of the image to be marked.
* `n`: The number to be marked into the image.
* `width`: The width of the image.
* `height`: The height of the image.
* `x`: The position in the image where the number will be drawn.
* `y`: The position in the image where the number will be drawn.
* `depth`: The number of bits per pixel of the image.
* `fg`: The color value to use for drawing the digits.
### Prototype
```c
EDTAPI void pdv_mark_ras_depth(void* addr, int n, int width, int height, int x, int y, int depth, int fg);
```
"""
function pdv_mark_ras_depth(addr, n, width, height, x, y, depth, fg)
    @ccall libpdv.pdv_mark_ras_depth(addr::Ptr{Cvoid}, n::Cint, width::Cint, height::Cint, x::Cint, y::Cint, depth::Cint, fg::Cint)::Cvoid
end

"""
    pdv_bytes_per_line(width, depth)

Returns bytes per line based on width and bit depth, including depth < 8.

# Arguments
* `width`: Pixels per line.
* `depth`: Bits per pixel.
# Returns
Bytes per line.
### Prototype
```c
EDTAPI int pdv_bytes_per_line(int width, int depth);
```
"""
function pdv_bytes_per_line(width, depth)
    @ccall libpdv.pdv_bytes_per_line(width::Cint, depth::Cint)::Cint
end

"""
    pdv_cl_camera_connected(pdv_p)

Checks whether a camera is connected and turned on.

Looks for an active (changing) pixel clock from the camera, and returns 1 if detected.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
1 if a pixel clock is detected, 0 if not detected.
### Prototype
```c
EDTAPI int pdv_cl_camera_connected(PdvDev pdv_p);
```
"""
function pdv_cl_camera_connected(pdv_p)
    @ccall libpdv.pdv_cl_camera_connected(pdv_p::PdvDev)::Cint
end

"""
    pdv_is_cameralink(pdv_p)

Infers that this device is connected to is a Camera Link camera (as opposed to RS-422 or LVDS parallel), based on settings from the loaded camera config file.

Specifically for framegrabbers, will return false (0) for simulators.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
1 if Camera Link framegrabber, 0 otherwise.
### Prototype
```c
EDTAPI int pdv_is_cameralink(PdvDev pdv_p);
```
"""
function pdv_is_cameralink(pdv_p)
    @ccall libpdv.pdv_is_cameralink(pdv_p::PdvDev)::Cint
end

"""
    pdv_is_simulator(pdv_p)

Infers that this device is a simulator, e.g. VisionLink CLS.

# Arguments
* `pdv_p`: The open [`PDV`](@ref) device handle.
# Returns
1 if a simulator, 0 otherwise.
### Prototype
```c
EDTAPI int pdv_is_simulator(PdvDev pdv_p);
```
"""
function pdv_is_simulator(pdv_p)
    @ccall libpdv.pdv_is_simulator(pdv_p::PdvDev)::Cint
end

struct __pthread_mutex_s
    __lock::Cint
    __count::Cuint
    __owner::Cint
    __nusers::Cuint
    __kind::Cint
    __spins::Cint
    __list::__pthread_list_t
end

# exports
const PREFIXES = ["pdv_", "edt_", "EDT_", "PDV_"]
for name in names(@__MODULE__; all=true), prefix in PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
end

end # module
