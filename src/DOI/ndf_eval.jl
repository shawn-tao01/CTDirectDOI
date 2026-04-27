
function eval_nbf(node, x0_map::Dict, xf_map::Dict)
    node isa Real                          && return Float64(node)
    node isa DOI.Initial{DYN_VAR}          && return x0_map[node.dyn_fun]
    node isa DOI.Final{DYN_VAR}            && return xf_map[node.dyn_fun]
    node isa DOI.NonlinearBoundaryFunction || error("Unknown NBF node: $(typeof(node))")

    args = [eval_nbf(a, x0_map, xf_map) for a in node.args]
    op = node.head
    op == :+   && return sum(args)
    op == :-   && return length(args) == 1 ? -args[1] : args[1] - args[2]
    op == :*   && return prod(args)
    op == :/   && return args[1] / args[2]
    op == :^   && return args[1] ^ args[2]
    op == :sin && return sin(args[1])
    op == :cos && return cos(args[1])
    op == :exp && return exp(args[1])
    op == :sqrt && return sqrt(args[1])
    error("Unsupported NBF op: $op")
end

function eval_ndf(node, var_map::Dict, t::Real)
    node isa Real                          && return Float64(node)
    node isa DOI.DynamicVariableIndex      && return var_map[node]
    node isa DOI.PhaseIndex                && return t
    node isa DOI.NonlinearDynamicFunction  || error("Unknown node: $(typeof(node))")

    args = [eval_ndf(a, var_map, t) for a in node.args]
    op = node.head
    op == :+   && return sum(args)
    op == :-   && return length(args) == 1 ? -args[1] : args[1] - args[2]
    op == :*   && return prod(args)
    op == :/   && return args[1] / args[2]
    op == :^   && return args[1] ^ args[2]
    op == :sin && return sin(args[1])
    op == :cos && return cos(args[1])
    op == :exp && return exp(args[1])
    op == :sqrt && return sqrt(args[1])
    error("Unsupported NDF op: $op")
end
