int16 drone_id
int32 traj_id
time start_time
float32 des_clearance

int16 drone_type        # 0: copter, 1: omni-drive, 2: diff-drive
float32 body_height     # a terrestrial drone is modeled as a cylinder with its body frame attached to the bottom
                        # so we need its body_height to do swarm control
float32 body_radius     # radius for the sphere (copter, aerial) and radius for the cylinder (terrestrial)
uint8 order
float32[3] start_p
float32[3] start_v
float32[3] start_a
float32[3] end_p
float32[3] end_v
float32[3] end_a
float32[] inner_x
float32[] inner_y
float32[] inner_z
float32[] duration