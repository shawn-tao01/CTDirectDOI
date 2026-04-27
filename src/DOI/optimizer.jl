
mutable struct Optimizer <: MOI.AbstractOptimizer
    name::String
    silent::Bool
    phases::OrderedSet{PHS}
    dyn_vars::OrderedDict{PHS, OrderedSet{DYN_VAR}}
    dif_cons::OrderedDict{PHS, DIF_CONS}
    alg_cons::OrderedDict{PHS, ALG_CONS}
    bou_cons::BOU_CONS
    linkages::LINKAGES
    dyn_var_bounds::OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}
    dyn_var_initials::OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}
    dyn_var_finals::OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}
    phase_initials::OrderedDict{PHS, LC64}
    phase_finals::OrderedDict{PHS, LC64}
    objective_sense::MOI.OptimizationSense
    objective::Union{OBJ, Nothing}
    dif_dyn_vars::OrderedSet{DYN_VAR}
    dyn_var_names::OrderedDict{DYN_VAR, String}
    start_dyn_vars::OrderedDict{PHS, STARTS}
    last_index_phases::Int64
    last_index_dyn_vars::Int64
    last_index_dif_cons::Int64
    last_index_alg_cons::Int64
    last_index_bou_cons::Int64
    last_index_linkages::Int64
    # CTDirect-specific
    n_steps::Int
    disc_method::Symbol
    state_idx::Dict{DYN_VAR, Int}
    control_idx::Dict{DYN_VAR, Int}
    ctdirect_solution::Any
end

function Optimizer()
    return Optimizer(
        "",                                                  # name
        false,                                               # silent
        OrderedSet{PHS}(),                                   # phases
        OrderedDict{PHS, OrderedSet{DYN_VAR}}(),             # dyn_vars
        OrderedDict{PHS, DIF_CONS}(),                        # dif_cons
        OrderedDict{PHS, ALG_CONS}(),                        # alg_cons
        BOU_CONS(),                                          # bou_cons
        LINKAGES(),                                          # linkages
        OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}(),      # dyn_var_bounds
        OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}(),      # dyn_var_initials
        OrderedDict{PHS, OrderedDict{DYN_VAR, LC64}}(),      # dyn_var_finals
        OrderedDict{PHS, LC64}(),                            # phase_initials
        OrderedDict{PHS, LC64}(),                            # phase_finals
        MOI.MIN_SENSE,                                       # objective_sense
        nothing,                                             # objective
        OrderedSet{DYN_VAR}(),                               # dif_dyn_vars
        OrderedDict{DYN_VAR, String}(),                      # dyn_var_names
        OrderedDict{PHS, STARTS}(),                          # start_dyn_vars
        0,                                                   # last_index_phases
        0,                                                   # last_index_dyn_vars
        0,                                                   # last_index_dif_cons
        0,                                                   # last_index_alg_cons
        0,                                                   # last_index_bou_cons
        0,                                                   # last_index_linkages
        100,                                                 # n_steps
        :trapeze,                                            # disc_method
        Dict{DYN_VAR, Int}(),                                # state_idx
        Dict{DYN_VAR, Int}(),                                # control_idx
        nothing,                                             # ctdirect_solution
    )
end

function MOI.empty!(model::Optimizer)
    model.name = ""
    empty!(model.phases)
    empty!(model.dyn_vars)
    empty!(model.dif_cons)
    empty!(model.alg_cons)
    empty!(model.bou_cons)
    empty!(model.linkages)
    empty!(model.dyn_var_bounds)
    empty!(model.dyn_var_initials)
    empty!(model.dyn_var_finals)
    empty!(model.phase_initials)
    empty!(model.phase_finals)
    model.objective_sense = MOI.MIN_SENSE
    model.objective = nothing
    empty!(model.dif_dyn_vars)
    empty!(model.dyn_var_names)
    empty!(model.start_dyn_vars)
    model.last_index_phases   = 0
    model.last_index_dyn_vars = 0
    model.last_index_dif_cons = 0
    model.last_index_alg_cons = 0
    model.last_index_bou_cons = 0
    model.last_index_linkages = 0
    empty!(model.state_idx)
    empty!(model.control_idx)
    model.ctdirect_solution = nothing
    return nothing
end

MOI.is_empty(model::Optimizer) = isempty(model.phases)

_lc64_value(set::EQ64) = set.value
_lc64_lb(set::GE64)    = set.lower
_lc64_lb(set::IV64)    = set.lower
_lc64_lb(set::EQ64)    = set.value
_lc64_ub(set::LE64)    = set.upper
_lc64_ub(set::IV64)    = set.upper
_lc64_ub(set::EQ64)    = set.value

function _extract_time(set::LC64, default::Float64)
    set isa EQ64 && return set.value
    set isa GE64 && return set.lower
    set isa LE64 && return set.upper
    set isa IV64 && return set.lower
    return default
end

function _push_lc64_bounds!(lb::Vector{Float64}, ub::Vector{Float64}, set::LC64)
    if set isa EQ64
        push!(lb, set.value); push!(ub, set.value)
    elseif set isa IV64
        push!(lb, set.lower); push!(ub, set.upper)
    elseif set isa GE64
        push!(lb, set.lower); push!(ub, Inf)
    elseif set isa LE64
        push!(lb, -Inf);      push!(ub, set.upper)
    end
    return nothing
end

function MOI.optimize!(model::Optimizer)

    phase = first(model.phases)

    # Identify state vs control variables
    state_vars   = [v for v in model.dyn_vars[phase] if v in model.dif_dyn_vars]
    control_vars = [v for v in model.dyn_vars[phase] if !(v in model.dif_dyn_vars)]
    n_x = length(state_vars)
    n_u = length(control_vars)

    # Build and store index maps for post-solve retrieval
    state_idx   = Dict(v => i for (i, v) in enumerate(state_vars))
    control_idx = Dict(v => i for (i, v) in enumerate(control_vars))
    merge!(model.state_idx,   state_idx)
    merge!(model.control_idx, control_idx)

    function make_var_map(x, u)
        d = Dict{DYN_VAR, Any}()
        for (v, i) in state_idx;   d[v] = x[i]; end
        for (v, i) in control_idx; d[v] = u[i]; end
        return d
    end

    # Non-autonomous in-place dynamics: (dx, t, x, u, v) -> nothing
    function f_dynamics(dx, t, x, u, v)
        var_map = make_var_map(x, u)
        for (i, sv) in enumerate(state_vars)
            for (_, (dif_fun, _)) in model.dif_cons[phase]
                if dif_fun.dyn_var == sv
                    dx[i] = eval_ndf(dif_fun.dyn_fun, var_map, t)
                    break
                end
            end
        end
        return nothing
    end

    # Extract phase time bounds
    t0 = haskey(model.phase_initials, phase) ?
        _extract_time(model.phase_initials[phase], 0.0) : 0.0
    tf = haskey(model.phase_finals, phase) ?
        _extract_time(model.phase_finals[phase], 1.0) : 1.0

    # ── Build CTModels OCP ──────────────────────────────────────────────────────
    ocp = CTModels.PreModel()
    CTModels.time_dependence!(ocp; autonomous=false)
    CTModels.state!(ocp, n_x)
    n_u > 0 && CTModels.control!(ocp, n_u)
    CTModels.time!(ocp; t0=t0, tf=tf)
    CTModels.dynamics!(ocp, f_dynamics)

    # Objective: mayer=(x0,xf,v)->scalar  lagrange=(t,x,u,v)->scalar
    sense = model.objective_sense == MOI.MIN_SENSE ? :min : :max
    obj   = model.objective
    if obj isa NBF
        mayer_fun = (x0, xf, v) -> eval_nbf(obj,
            Dict(state_vars[i] => x0[i] for i in 1:n_x),
            Dict(state_vars[i] => xf[i] for i in 1:n_x),
        )
        CTModels.objective!(ocp, sense; mayer=mayer_fun)
    elseif obj isa DOI.MultiPhaseIntegral{NDF}
        lag_ndf      = obj.dyn_funs[1]
        lagrange_fun = (t, x, u, v) -> eval_ndf(lag_ndf, make_var_map(x, u), t)
        CTModels.objective!(ocp, sense; lagrange=lagrange_fun)
    elseif obj isa BOLZA
        mayer_fun = (x0, xf, v) -> eval_nbf(obj.bou_fun,
            Dict(state_vars[i] => x0[i] for i in 1:n_x),
            Dict(state_vars[i] => xf[i] for i in 1:n_x),
        )
        lag_ndf      = obj.integral.dyn_funs[1]
        lagrange_fun = (t, x, u, v) -> eval_ndf(lag_ndf, make_var_map(x, u), t)
        CTModels.objective!(ocp, sense; mayer=mayer_fun, lagrange=lagrange_fun)
    end

    # Initial state boundary constraints
    ic_idx = Int[]; ic_lb = Float64[]; ic_ub = Float64[]
    for (v_var, set) in model.dyn_var_initials[phase]
        haskey(state_idx, v_var) || continue
        push!(ic_idx, state_idx[v_var])
        _push_lc64_bounds!(ic_lb, ic_ub, set)
    end
    if !isempty(ic_idx)
        let _idx = copy(ic_idx)
            CTModels.constraint!(ocp, :boundary;
                f     = (val, x0, xf, v) -> (val .= [x0[i] for i in _idx]),
                lb    = ic_lb, ub = ic_ub,
                label = :initial_state,
            )
        end
    end

    # Final state boundary constraints
    fc_idx = Int[]; fc_lb = Float64[]; fc_ub = Float64[]
    for (v_var, set) in model.dyn_var_finals[phase]
        haskey(state_idx, v_var) || continue
        push!(fc_idx, state_idx[v_var])
        _push_lc64_bounds!(fc_lb, fc_ub, set)
    end
    if !isempty(fc_idx)
        let _idx = copy(fc_idx)
            CTModels.constraint!(ocp, :boundary;
                f     = (val, x0, xf, v) -> (val .= [xf[i] for i in _idx]),
                lb    = fc_lb, ub = fc_ub,
                label = :final_state,
            )
        end
    end

    # State and control box bounds from dyn_var_bounds
    for (j, (v_var, set)) in enumerate(model.dyn_var_bounds[phase])
        bnd_lb = Float64[]; bnd_ub = Float64[]
        _push_lc64_bounds!(bnd_lb, bnd_ub, set)
        if haskey(state_idx, v_var)
            i = state_idx[v_var]
            CTModels.constraint!(ocp, :state;
                rg=i:i, lb=bnd_lb, ub=bnd_ub,
                label=Symbol("state_bound_$(j)"),
            )
        elseif haskey(control_idx, v_var)
            i = control_idx[v_var]
            CTModels.constraint!(ocp, :control;
                rg=i:i, lb=bnd_lb, ub=bnd_ub,
                label=Symbol("control_bound_$(j)"),
            )
        end
    end

    CTModels.definition!(ocp, :(ocp = nothing))
    ct_model = CTModels.build_model(ocp)

    # ── Discretize and solve ────────────────────────────────────────────────────
    discretizer = CTDirect.Collocation(grid_size=model.n_steps, scheme=model.disc_method)
    docp        = CTDirect.discretize(ct_model, discretizer)
    init        = CTModels.build_initial_guess(ct_model, nothing)
    modeler     = CTSolvers.Modelers.ADNLP()
    solver      = CTSolvers.Solvers.Ipopt(print_level=model.silent ? 0 : 5)

    sol = CommonSolve.solve(docp, init, modeler, solver; display=!model.silent)

    model.ctdirect_solution = sol
end
