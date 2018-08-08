# Filters using Nullable types to represent missing measurements

function update{T}(kf::KalmanFilter,y::Union{Vector{T}, Nothing})
    if y.y === nothing
        return kf
    else
        return update(kf,Observation(get(y)))
    end
end

function update!{T}(kf::KalmanFilter,y::Union{Vector{T}, Nothing})
    if y === nothing
        return kf
    else
        update!(kf,Observation(get(y)))
        return kf
    end
end

function predictupdate{T}(kf::BasicKalmanFilter,y::Union{Vector{T}, Nothing})
    update(predict(kf),y)
end

function predictupdate!{T}(kf::BasicKalmanFilter,y::Union{Vector{T}, Nothing})
    update!(predict!(kf),y)
end


