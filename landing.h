typedef enum			e_auto_pilot_state
{
	AUTO_DISABLED,

	AUTO_DIVE,
	AUTO_LANDING_BURN,
	AUTO_LANDED,
}						e_auto_pilot_state;

typedef struct			t_rocket_control
{
	vec3				pos;
	vec3				vel;
	quat				angular_vel;
	double				throttle;
	double				throttle_delta;
	double				thrust_kg;
	double				weight_kg;

	e_auto_pilot_state	auto_state;
	double				throttle_target;
	vec3				landing_target;
}						t_rocket_control;
