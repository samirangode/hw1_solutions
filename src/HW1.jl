module HW1

using NBInclude

function studentinfo()
    info = Dict(
        "name" => "Brian Jackson",
        "Andrew ID" => "bjackso2"
    )
    return info
end

notebook() = @nbinclude(joinpath(@__DIR__,"hw1_NewtonsMethod.ipynb"))
function test_benchmarktools()
    @btime 1+2 samples=2 evals=2
end

end # module
