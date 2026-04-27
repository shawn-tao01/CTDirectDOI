module CTDirectDOI

import MathOptInterface as MOI
import DynOptInterface  as DOI
import CTDirect, CTModels, CTSolvers, CommonSolve
import NLPModelsIpopt
using  OrderedCollections

include("DOI/ndf_eval.jl")
include("DOI/aliases.jl")
include("DOI/optimizer.jl")
include("DOI/attributes.jl")
include("DOI/ingredients.jl")
include("DOI/solutions.jl")

export  Optimizer

end 
