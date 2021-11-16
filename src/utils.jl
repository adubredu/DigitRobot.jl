function load_digit()
    mech  = mechanism(add_flat_ground=true)
    mvis = MechanismVisualizer(mech, URDFVisuals(DigitRobot.urdfpath())) 
    state = MechanismState(mech)
    setnominal!(state)
    set_configuration!(mvis, configuration(state))
    default_background!(mvis)
    settransform!(mvis.visualizer["/Cameras/default"],
            compose(Translation(0.0, 0.0, -1.0), LinearMap(RotZ(-pi / 2.0))))

    return mvis, mech, state
end 

function set_limb_configuration!(qs::Vector, limb::String)
    if limb == "left_leg"
        set_configuration!()
