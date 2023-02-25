
vec3 solve_throttle_for_xz(t_rocket_control *control, double delta_time)
{
	vec3 targetxz = control->landing_target;
	vec3 posxz = control->pos;
	vec3 velxz = control->vel;
	targetxz.y = 0;
	posxz.y = 0;
	velxz.y = 0;

	//target xz velocity 0ms at 1km
	double height_at_zero = 1000;
	if (control->pos.y < height_at_zero)
		height_at_zero = 0;

	double t_to_land = max(control->pos.y - height_at_zero, 0) / -control->vel.y;
	vec3 predicted_land = VEC_ZERO;
	predicted_land.x = control->pos.x + t_to_land * velxz.x;
	predicted_land.z = control->pos.z + t_to_land * velxz.z;

	vec3 res = vec_sub(targetxz, predicted_land);
	vec_mul(&res, 0.003);
	return vec_invert(res);
}

double	solve_throttle_for_y(t_rocket_control *control, double thrust_force)
{
	double burn_duration = -control->vel.y / (thrust_force / 8.0);
	double burn_dist = -control->vel.y * burn_duration;
	double dist_to_light = burn_dist + control->landing_target.y;

	double throttle_change = dist_to_light - fabsf(control->pos.y);
	double throttle_control_range = 0.002;//to minimize landing velocity with high accuracy
	double range_half = throttle_control_range / 2.0;
	throttle_change = clamp(throttle_change, -range_half, range_half);
	//map to range 0 - 1
	double res = throttle_change / throttle_control_range + 0.5;
	return res;
}

void	landing_burn_controller(t_rocket_control *control, double thrust_force, double delta_time)
{
	//dont throttle xz at very low velocity
	bool xz = true;
	if (control->vel.y > -4.0)
		xz = false;

	vec3 throttle_vec = VEC_ZERO;
	if (xz)
		throttle_vec = solve_throttle_for_xz(control, delta_time);
	//the difference is y cant go negative
	throttle_vec.y = -solve_throttle_for_y(control, thrust_force);

	if (xz)
	{
		quat retro = quat_look_at(vec_invert(throttle_vec));
		quat_mul_this(&retro, quat_from_x(M_PI_2));
		control->rot = slerp(control->rot, retro, 2.0 * delta_time);
	}
	else
		control->rot = slerp(control->rot, QUAT_IDENTITY, 2.0 * delta_time);

	double len = vec_length(throttle_vec);
	clamp01(len);
	control->throttle_target = len;

	//to improve landing velocity beyond 0.02ms should take throttle change speed into account

	//smooth out throttle control
	double throttle_vel = float_difference(control->throttle_target, control->throttle);
	clamp01(throttle_vel);
	if (control->throttle_target > control->throttle)
		control->throttle += delta_time * control->throttle_delta * throttle_vel;
	else
		control->throttle -= delta_time * control->throttle_delta * throttle_vel;
}

void	rocket_simulate(t_rocket_control *control, float thrust_force, double delta_time)
{
	clamp01(control->throttle);
	if (control->throttle)
	{
		vec3 up = quat_get_up(control->rot);
		vec_normalize(&up);
		vec_mul(&up, delta_time * thrust_force * control->throttle);
		control->vel = vec_add(control->vel, up);
	}
	float gravity = -9.81;//earth
	// float gravity = -1.625;//moon
	// float gravity = -3.72;//mars
	control->vel.y += gravity * delta_time;
	control->pos = vec_add(control->pos, vec_mul_copy(control->vel, delta_time));
	control->rot = quat_mul(control->rot, control->angular_vel);

	if (control->pos.y < 0)
	{
		//check if landed
		if (control->auto_state != AUTO_LANDED)
			control->auto_state = AUTO_LANDED;
		control->pos.y = 0;
		control->vel = VEC_ZERO;
	}
}

void tick()
{
	double thrust_force = control->thrust_kg / control->weight_kg;
	landing_burn_controller(control, thrust_force, dt);
	rocket_simulate(control, thrust_force, dt);
}
