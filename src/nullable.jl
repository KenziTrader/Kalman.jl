# Filters using Nullable types to represent missing measurements

using Nullables

function update(kf::KalmanFilter,y::Nullable{Vector{T}}) where T
    if isnull(y.y)
        return kf
    else
        return update(kf,Observation(get(y)))
    end
end

function update!(kf::KalmanFilter,y::Nullable{Vector{T}}) where T
    if isnull(y)
        return kf
    else
        update!(kf,Observation(get(y)))
        return kf
    end
end

function predictupdate(kf::BasicKalmanFilter,y::Nullable{Vector{T}}) where T
    update(predict(kf),y)
end

function predictupdate!(kf::BasicKalmanFilter,y::Nullable{Vector{T}}) where T
    update!(predict!(kf),y)
end