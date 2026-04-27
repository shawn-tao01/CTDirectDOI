# MOI Optimizer Attributes

MOI.get(::Optimizer, ::MOI.SolverName)    = "CTDirect"
MOI.get(::Optimizer, ::MOI.SolverVersion) = "0.0.0"

## Name
MOI.supports(::Optimizer, ::MOI.Name) = true
function MOI.set(model::Optimizer, ::MOI.Name, name::String)
    model.name = name
    return nothing
end
MOI.get(model::Optimizer, ::MOI.Name) = model.name

## Silent
MOI.supports(::Optimizer, ::MOI.Silent) = true
function MOI.set(model::Optimizer, ::MOI.Silent, flag::Bool)
    model.silent = flag
    return nothing
end
MOI.get(model::Optimizer, ::MOI.Silent) = model.silent

## ObjectiveSense
MOI.supports(::Optimizer, ::MOI.ObjectiveSense) = true
function MOI.set(model::Optimizer, ::MOI.ObjectiveSense, sense::MOI.OptimizationSense)
    model.objective_sense = sense
    return nothing
end
MOI.get(model::Optimizer, ::MOI.ObjectiveSense) = model.objective_sense

MOI.supports_incremental_interface(::Optimizer) = true


## CTDirect Optimizer Attributes

struct GridSize <: MOI.AbstractOptimizerAttribute end

MOI.supports(::Optimizer, ::GridSize) = true

function MOI.set(model::Optimizer, ::GridSize, n::Integer)
    model.n_steps = Int(n)
    return nothing
end

MOI.get(model::Optimizer, ::GridSize) = model.n_steps


struct DiscMethod <: MOI.AbstractOptimizerAttribute end

MOI.supports(::Optimizer, ::DiscMethod) = true

function MOI.set(model::Optimizer, ::DiscMethod, method::Symbol)
    model.disc_method = method
    return nothing
end

MOI.get(model::Optimizer, ::DiscMethod) = model.disc_method
