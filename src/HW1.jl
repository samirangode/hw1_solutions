module HW1

using NBInclude

function studentinfo()
    info = Dict(
        "name" => "Brian Jackson",
        "Andrew ID" => "bjackso2"
    )
    return info
end

function notebook() 
    @nbinclude(joinpath(@__DIR__,"hw1.ipynb"))
end

end # module
