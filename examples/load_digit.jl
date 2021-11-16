using Revise
using DigitRobot 
using RigidBodyDynamics
using MeshCatMechanisms
using CoordinateTransformations
using Rotations
using MeshCat

mech  = DigitRobot.mechanism(add_flat_ground=true)
mvis = MechanismVisualizer(mech, URDFVisuals(DigitRobot.urdfpath()))
# open(mvis)
state = MechanismState(mech)
DigitRobot.setnominal!(state)
set_configuration!(mvis, configuration(state))
default_background!(mvis)
settransform!(mvis.visualizer["/Cameras/default"],
	    compose(Translation(0.0, 0.0, -1.0), LinearMap(RotZ(-pi / 2.0))))
render(mvis)