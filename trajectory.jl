import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics, RigidBodySim
using StaticArrays, LinearAlgebra
using MeshCat, MeshCatMechanisms
using Gadfly, Cairo, Fontconfig


###############################################################################

q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

qdiff = qd - q0
T = 10

function Traj(t)

   theta = q0 + ((3*t^2)/(T^2) - (2*t^3/T^3) )*qdiff
   dtheta = ((6*t)/(T^2) - (6*t^2/T^3) )*qdiff
   ddtheta = (6/(T^2) - (12*t)/(T^3) )*qdiff


   print("\n\n q: \n\n")
   print(theta)
   print("\n\n qdot: \n\n")
   print(dtheta)
   print("\n\n qddot: \n\n")
   print(ddtheta)

end

################################################################################

function display_urdf(urdfPath,vis)
    mechanism = parse_urdf(Float64,urdfPath)
    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    for bd in bodies(mechanism)
        setelement!(mvis,default_frame(bd),0.5,"$bd")
    end
    return mvis, mechanism
end

################################################################################

function Control_PD!(τ, t, state)
    kp = 100
    kd = 50

    τ .= -diagm(kd*[1,1,1,1,1,1,1,1,1])*velocity(state) - diagm(kp*[1,1,1,1,1,1,1,1,1])*(configuration(state) - [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01])
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

#################################################################################




function Control_CTC!(τ, t, state)
    time = 0
    dt = .01
    qprev = configuration(state)
    i = 0

    kp = 50
    kd = 20

    qdes = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

    M = mass_matrix(state)

    for i = 0:dt:10

      qd = qprev + ((3*t^2)/(T^2) - (2*t^3/T^3) )*(qdes-qprev)
      dqd = ((6*t)/(T^2) - (6*t^2/T^3) )*(qdes-qprev)
      ddqd = (6/(T^2) - (12*t)/(T^3) )*(qdes-qprev)

      q = configuration(state)

      dq = velocity(state)

      e = qd-q
      edot = dqd - dq

      τ .= M*(ddqd + kp * e + kd * edot)

      act_sat = 50; # Actuator limits
      τ .= map( x -> x > act_sat ? act_sat : x,τ)
      τ .= map( x -> x < -act_sat ? -act_sat : x,τ)

      time = time + dt
      i = i + dt

    end

end



#################################################################################

function controller(controller)

    vis = Visualizer();open(vis)

    delete!(vis)

    urdfPath = "panda.urdf"
    mvis, mechanism = display_urdf(urdfPath,vis)

    state = MechanismState(mechanism)

    set_configuration!(state,[0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1])
    zero_velocity!(state)
    set_configuration!(mvis, configuration(state))

    problem = ODEProblem(Dynamics(mechanism,controller), state, (0., 10.));
    sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
    setanimation!(mvis, sol; realtime_rate = 1.0);

end
