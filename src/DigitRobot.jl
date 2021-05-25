__precompile__()

module DigitRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

packagepath() = joinpath(@__DIR__,"..")
urdfpath() = joinpath(packagepath(), "urdf", "digit_model.urdf")

function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end


function mechanism(::Type{T} = Float64;
        floating = true,
        contactmodel = default_contact_model(),
        remove_fixed_tree_joints = true, add_flat_ground=false) where {T}

    mechanism = RigidBodyDynamics.parse_urdf(urdfpath(); scalar_type=T, floating=floating, remove_fixed_tree_joints=remove_fixed_tree_joints)

    if contactmodel != nothing
        for side in (:left, :right)
            foot = findbody(mechanism, "$(string(side))_toe_roll")
            frame = default_frame(foot)

            #point of contact of foot
            x = 0.000559
            y = -0.050998
            z = -0.030691

            add_contact_point!(foot, ContactPoint(Point3D(frame, x,y,z), contactmodel))
        end
    end

    if add_flat_ground
        frame = root_frame(mechanism)
        ground = HalfSpace3D(Point3D(frame, 0.,0.,0.), FreeVector3D(frame, 0.,0.,1.))
        add_environment_primitive!(mechanism, ground)
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end


function setnominal!(digitstate::MechanismState)
    mechanism = digitstate.mechanism
    zero!(digitstate)

    hip_joint_left = findjoint(mechanism, "hip_abduction_left")
    hip_joint_right = findjoint(mechanism, "hip_abduction_right")
    toe_joint_left = findjoint(mechanism, "toe_pitch_joint_left")
    toe_joint_right = findjoint(mechanism, "toe_pitch_joint_right")

    set_configuration!(digitstate, hip_joint_left, 0.337)
    set_configuration!(digitstate, hip_joint_right, -0.337)
    set_configuration!(digitstate, toe_joint_left, -0.126)
    set_configuration!(digitstate, toe_joint_right, 0.126)

    floatingjoint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(digitstate, floatingjoint, [1; 0; 0; 0; 0; 0; 0.85])
    digitstate
end

function __init__()
    if !isfile(urdfpath())
        error("Could not find $(urdfpath())")
    end
end

end #module
