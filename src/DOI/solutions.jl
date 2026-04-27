
function MOI.get(model::Optimizer, ::DOI.DynamicVariableSolution, dyn_var::DYN_VAR)
    sol = model.ctdirect_solution

    if dyn_var in model.dif_dyn_vars
        idx = model.state_idx[dyn_var]
        x_fn = CTModels.state(sol)
        return t -> x_fn(t)[idx]
    else
        idx = model.control_idx[dyn_var]
        u_fn = CTModels.control(sol)
        return t -> u_fn(t)[idx]
    end
end
