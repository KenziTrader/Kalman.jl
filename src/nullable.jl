# Filters using Nullable types to represent missing measurements

function update(kf::KalmanFilter,y::Union{Vector{T}, Nothing}) where T
    if y.y == nothing
        return kf
    else
        return update(kf,Observation(y))
    end
end

function update!(kf::KalmanFilter,y::Union{Vector{T}, Nothing}) where T
    if y == nothing
        return kf
    else
        update!(kf,Observation(y))
        return kf
    end
end

function predictupdate(kf::BasicKalmanFilter,y::Union{Vector{T}, Nothing}) where T
    update(predict(kf),y)
end

function predictupdate!(kf::BasicKalmanFilter,y::Union{Vector{T}, Nothing}) where T
    update!(predict!(kf),y)
end


