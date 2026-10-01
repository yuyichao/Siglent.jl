#!/usr/bin/julia

using Distributed

const test_files = ["format",
                    "preamble",
                    "transport",
                    "api",
                    ]

addprocs(min(Sys.CPU_THREADS, length(test_files)))

pmap(test_files) do file
    println("Start testing $file")
    @eval module $(Symbol("Test_$(file)_mod"))
    include($(joinpath(@__DIR__, "$(file).jl")))
    end
    println("Done testing $file")
    # So that we do not try to bring the worker-only module back to the main process
    return
end
