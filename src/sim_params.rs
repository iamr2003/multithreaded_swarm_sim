static total_time: f64 = 5.0; //s
static dt: f64 = 0.01;
static steps: i32 = (total_time / dt) as i32;
static n_agents: i32 = 1000;
static enclosure_edge: f64 = 10.; //m
static max_init_vel: f64 = 10.; //m/s
static max_vel: f64 = 10.;
static neighbor_radius: f64 = 1.0; //m
