# example julia script to be run for unit tests

using iLQGames: SVector 

s = "Hi! I'm a Julia script (even though you might be calling me from python)!"

function foo(x)
    return 2*x
end

function foo_array_sum(x::AbstractArray)
    return sum(x)
end

return s
