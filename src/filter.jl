#######
# Universal Kalman filtering methods

function predict!(kf::KalmanFilter)
    kf.x = ap(kf.f,kf.x)
    kf
end

function predict(kf::KalmanFilter)
    predict!(copy(kf))
end

function update(kf::KalmanFilter,y::Observation)
    update!(copy(kf),y)
end

function update!(kf::KalmanFilter,y::Observation)
    (res,ph,s) = covs(kf,y)
	handleMissingMeasurements!(y, res, ph, s)
    xn = kf.x.x + ph * (s\res)
    pn = kf.x.p - ph * (s'\ph')
    
    # This is an ugly hack which works for now
    if typeof(kf.x) <: AbstractUnscentedState
        kf.x = UnscentedState(xn,pn,kf.x.α,kf.x.β,kf.x.κ)
    else
        kf.x = State(xn,pn)
    end
    kf
end

function predictupdate(kf::KalmanFilter,y::Observation)
    update(predict(kf),y)
end

function predictupdate!(kf::KalmanFilter,y::Observation)
    update!(predict!(kf),y)
end


function handleMissingMeasurements!(y::Observation,
									res::Vector, ph::Matrix, s::Matrix)
	nm = length(y.y)
	ndim = size(ph, 1)
    for j=1:nm
    	if y.y[j]==0 || isnan(y.y[j])
    		res[j] = 0
			for k=1:nm
				s[j, k] = 0
				s[k, j] = 0
			end
			s[j, j] = 1
			for i=1:ndim
				ph[i, j] = 0
			end
    	end
    end
end
