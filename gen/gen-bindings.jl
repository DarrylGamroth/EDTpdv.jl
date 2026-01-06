using Clang.Generators

cd(@__DIR__)

include_root = "/opt/EDTpdv"

options = load_options(joinpath(@__DIR__, "generator.toml"))
args = get_default_args()
push!(args, "-I$include_root")
push!(args, "-I$(joinpath(include_root, "driver"))")

headers = [
    joinpath(include_root, "libpdv.h"),
]

ctx = create_context(headers, args, options)
build!(ctx)
