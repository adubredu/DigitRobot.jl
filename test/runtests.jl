using Test
using Main.DigitRobot
using RigidBodyDynamics
using RigidBodyDynamics.Contact: location

@testset "digit" begin
    mechanism = DigitRobot.mechanism()
    @test num_velocities(mechanism) == 28
    @test num_positions(mechanism) == 29
end


@testset "setnominal!" begin
    mechanism = DigitRobot.mechanism(add_flat_ground=true)
    state = MechanismState(mechanism)
    DigitRobot.setnominal!(state)

    for side in (:left, :right)
        foot = findbody(mechanism, "$(string(side))_toe_roll")
        for point in contact_points(foot)
            contact_location_world  = transform(state, location(point), root_frame(mechanism))
            @test contact_location_world.v[3] ≈ 0 atol=7e-2
        end
    end
    @test center_of_mass(state).v[3] > 0.8
end
