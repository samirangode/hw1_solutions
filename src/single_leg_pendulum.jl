load_a1_urdf(urdf_path) = parse_urdf(urdf_path,floating=true,remove_fixed_tree_joints=false)

abstract type PendulumModel end
const joint_indices = (4:15)  # non fix joint position
const hip_indices = (4:7)
const thigh_indices = (8:11)
const calf_indices = (12:15)

mutable struct SingleLegPendulum <: PendulumModel
	mechanism::Mechanism
	state::MechanismState
	SingleLegPendulum(a::Mechanism) = new(a,MechanismState(a))
end

hip_joints(model::PendulumModel) = joints(model.mechanism)[hip_indices]
thigh_joints(model::PendulumModel) = joints(model.mechanism)[thigh_indices]
calf_joints(model::PendulumModel)= joints(model.mechanism)[calf_indices]
world_joint(model::PendulumModel) = joints(model.mechanism)[1]

function build_rear_foot_constraints!(model::SingleLegPendulum)
	mechanism = model.mechanism
	state = model.state
	RR_foot = findbody(mechanism, "RR_foot")
	trunk = findbody(mechanism, "trunk")
	world_body = bodies(mechanism)[1]
	RR_foot_joint = Joint("RR_foot_joint", QuaternionSpherical{Float64}())
	RR_trans = translation(relative_transform(state,
										default_frame(trunk),
										default_frame(RR_foot)))

	RR_foot_origin = SVector(RR_trans[1], RR_trans[2], 0)

	RR_foot_to_foot_joint_pose = Transform3D(
                frame_before(RR_foot_joint),
                default_frame(RR_foot),
                SVector(0, 0, 0),
    )
	RR_foot_joint_to_world_pose = Transform3D(
                default_frame(world_body),
                frame_after(RR_foot_joint),
                RR_foot_origin,
    )
    attach!(mechanism,
            RR_foot,
            world_body,
            RR_foot_joint,
            joint_pose = RR_foot_to_foot_joint_pose,
            successor_pose = RR_foot_joint_to_world_pose,
    )
	model.state = MechanismState(model.mechanism)

end

function build_rear_foot_constraints_revolute!(model::SingleLegPendulum)
	build_rear_foot_constraints_revolute!(model.mechanism)
	model.state = MechanismState(model.mechanism)
end
function build_rear_foot_constraints_revolute!(mechanism::Mechanism)
	state = MechanismState(mechanism)
	RR_foot = findbody(mechanism, "RR_foot")
	trunk = findbody(mechanism, "trunk")
	world_body = bodies(mechanism)[1]

	RR_foot_joint_z = Joint("RR_foot_joint_z", Revolute([0.,0.,1.]))
	RR_foot_joint_x = Joint("RR_foot_joint_x", Revolute([1.,0.,0.]))
	RR_foot_joint_y = Joint("RR_foot_joint_y", Revolute([0.,1.,0.]))


	RR_trans = translation(relative_transform(state,
	                                    default_frame(trunk),
	                                    default_frame(RR_foot)))

	RR_foot_origin = SVector(RR_trans[1], RR_trans[2], 0)

	Rz_to_foot_joint_pose = Transform3D(
	            frame_before(RR_foot_joint_z),
	            default_frame(RR_foot),
	            SVector(0, 0, 0),
	)

	dummy_z = RigidBody{Float64}("dummy_z")
	dummy_x = RigidBody{Float64}("dummy_x")

	axis1 = SVector(0., 0., 1.) # joint axis
	axis2 = SVector(1., 0., 0.) # joint axis

	I_1 = 0.0 # moment of inertia about joint axis
	c_1 = 0# center of mass location with respect to joint axis
	m_1 = 0.0 # mass
	inertia1 = SpatialInertia(default_frame(dummy_z),
	    moment=I_1 * axis1 * axis1',
	    com=SVector(0, 0, c_1),
	    mass=m_1)
	inertia2 = SpatialInertia(default_frame(dummy_x),
	    moment=I_1 * axis2 * axis2',
	    com=SVector(0, 0, c_1),
	    mass=m_1)
	spatial_inertia!(dummy_z,inertia1)
	spatial_inertia!(dummy_x,inertia2)

	RZ_to_dummy = Transform3D(
	                frame_before(RR_foot_joint_z),
	                default_frame(dummy_z),
	                SVector(0,0,0)
	)

	attach!(mechanism,RR_foot,
	                  dummy_z,
	                  RR_foot_joint_z,
	                  joint_pose = Rz_to_foot_joint_pose)

	Rx_to_dummy = Transform3D(
	                frame_before(RR_foot_joint_x),
	                default_frame(dummy_z),
	                SVector(0,0,0)
	)
	attach!(mechanism,dummy_z,
	                  dummy_x,
	                  RR_foot_joint_x,
	                  joint_pose = Rx_to_dummy)

	Ry_to_world = Transform3D(
	                frame_before(RR_foot_joint_y),
	                default_frame(dummy_x),
	                SVector(0,0,0)
	)
	attach!(mechanism,dummy_x,
	                  world_body,
	                  RR_foot_joint_y,
	                  joint_pose = Ry_to_world)

	remove_joint!(mechanism,findjoint(mechanism,"base_to_world"))
	# model.state = MechanismState(mechanism)

end

function set_standing_config!(model::PendulumModel, hip_configs::SVector{4},
							 thigh_configs::SVector{4}, calf_configs::SVector{4}, world_configs::SVector{7})
	state = model.state
	for i in 1:4
		set_configuration!(state, hip_joints(model)[i], hip_configs[i])
		set_configuration!(state, thigh_joints(model)[i], thigh_configs[i])
		set_configuration!(state, calf_joints(model)[i], calf_configs[i])
	end

	# set body pose
	# set_configuration!(state, world_joint,[1.0,0,-0.6,0,0,0,0.465])
    set_configuration!(state, world_joint(model), world_configs)

end

# function control!(torques::AbstractVector, t, state::MechanismState)
# 	# sample control function to pass into simulate
#     rand!(torques) # for example
# end


# function ev_dynamics(q_and_torques::AbstractVector{T}) where T
#     state = MechanismState{T}(a1)
#     q = q_and_torques[1:19]
# 	torques = q_and_torques[20:end]
#
#     # set the state variables:
#     set_configuration!(state, q)
# 	dynamics_result = DynamicsResult(a1)
# 	dynamics!(dynamics_result,state,torques)
#     # return results converted to an `SVector`(as ForwardDiff expects an `AbstractVector`)
#     Vector(SVector{18}(dynamics_result.v̇))
# end
