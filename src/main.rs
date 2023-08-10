use ndarray::{array, s, Array, Array1, Array2, Axis};
use ndarray_linalg::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;
use std::time::{Duration, Instant};
use three_d::*;

// spits out velocity
fn local_controller(
    self_agent: usize,
    neighbors: &Vec<usize>,
    agents_pos: &Array2<f32>,
    agents_vel: &Array2<f32>,
) -> Array1<f32> {
    //let's start with basic boids
    let curr_pos = agents_pos.slice(s![self_agent, ..]);
    let pos_centroid = agents_pos.mean_axis(Axis(0)).unwrap();
    let vel_centroid = agents_pos.mean_axis(Axis(0)).unwrap();

    let sep: Array1<f32> = Array::zeros(2);
    //need to do some manipulation
    // let rel_pos;

    let coh_gain = 1.0;
    let ali_gain = 1.0;
    let sep_gain = 1.0;

    let out = coh_gain * (pos_centroid - curr_pos) + ali_gain * vel_centroid + sep_gain * sep;

    return out;
}

//0 to swarm sim is the goal
fn main() {
    println!("Hello, world!");

    // let a1 = array!([1,2,3,4]);
    // let a2 = array!([4,3,2,1]);
    // println!("{}",a1+a2);
    //
    // let a3 = array!([[1,2],[3,4]]);
    // let a4 = array!([[2,2],[3,4]]);
    // println!("{}",a3+a4);

    // let r = Array::random((2,5),Uniform::new(0.,10.));
    // println!("{}",r);

    let total_time = 5.0; //s
    let dt = 0.01;
    let steps = (total_time / dt) as i32;
    let n_agents = 10;
    let enclosure_edge = 10.; //m
    let neighbor_radius = 5.; //m

    let mut agents_pos: Array2<f32> =
        Array::random((n_agents, 2), Uniform::new(-enclosure_edge, enclosure_edge));
    let mut agents_vel: Array2<f32> = Array::zeros((n_agents, 2));

    let mut pos_history: Vec<Array2<f32>> = vec![];
    let mut vel_history: Vec<Array2<f32>> = vec![];

    // println!("{}", agents_pos.mean_axis(Axis(0)).unwrap());

    // println!("{}", agents_pos.slice(s![0, ..]));

    // let mut top_agent = agents_pos.slice_mut(s![0, ..]);

    for mut agent in agents_pos.rows_mut() {
        // println!("{}", agent);
        //need to understand a little more how to handle this, I just want to reassign some values
        //in bulk
        let new: Array<f32, _> = array![agent[1], agent[0]];

        agent.assign(&new);
        // let temp = agent.to_owned();
        // agent[0] = temp[1];
        // agent[1] = temp[0];
    }

    // println!("{}", agents_pos);
    //to_owned needs to be used to clone the subview properly
    // for i in 0..n_agents {
    //     for j in 0..n_agents {
    //         let diff = agents_pos.slice(s![j, ..]).to_owned()  -agents_pos.slice(s![i, ..]).to_owned();
    //         println!("{}", diff);
    //     }
    // }

    for _ in 0..steps {
        let mut next_agents_pos: Array2<f32> = agents_pos.clone();
        let mut next_agents_vel: Array2<f32> = agents_vel.clone();
        //find neighbors, the slow way
        for i in 0..n_agents {
            let mut neighbors: Vec<usize> = vec![];
            let agent = agents_pos.slice(s![i, ..]).to_owned();
            for j in 0..n_agents {
                if i != j {
                    let other = agents_pos.slice(s![j, ..]).to_owned();
                    let diff = agent.to_owned() - other;
                    if diff.norm() > neighbor_radius {
                        //is neighbor, now do stuff
                        neighbors.push(j);
                    }
                }
                //some kind of handle neighbors call
                let mut agent_next_vel = local_controller(i, &neighbors, &agents_pos, &agents_vel);
                let agent_next_pos = agent_next_vel.clone() * dt + agent.to_owned();

                //eventually boundary conditions and velocity limits
                //simple bounce condition by dimension
                if agent_next_pos[0] > enclosure_edge && agent_next_vel[0] > 0.0 {
                    agent_next_vel[0] *= -1.0;
                }

                if agent_next_pos[0] < enclosure_edge && agent_next_vel[0] < 0.0 {
                    agent_next_vel[0] *= -1.0;
                }

                if agent_next_pos[1] > enclosure_edge && agent_next_vel[1] > 0.0 {
                    agent_next_vel[1] *= -1.0;
                }

                if agent_next_pos[1] < enclosure_edge && agent_next_vel[1] < 0.0 {
                    agent_next_vel[1] *= -1.0;
                }

                //updating again after boundary update
                let agent_next_pos = agent_next_vel.clone() * dt + agent.to_owned();

                //  assign for specific agent
                next_agents_pos.slice_mut(s![i, ..]).assign(&agent_next_pos);
                next_agents_vel.slice_mut(s![i, ..]).assign(&agent_next_vel);
            }
        }
        //log current iteration and overwrite
        pos_history.push(agents_pos.clone());
        vel_history.push(agents_vel.clone());

        agents_pos = next_agents_pos;
        agents_vel = next_agents_vel;
    }

    // for pos in pos_history.iter(){
    //     println!("{}",pos);
    // }

    //some rendering of the agents, would like to keep separate from the engine aspect
    let window = Window::new(WindowSettings {
        title: "Shapes 2D!".to_string(),
        max_size: Some((720, 720)),
        ..Default::default()
    })
    .unwrap();
    let context = window.gl();
    let scale_factor = 1.0;
    let (width, height) = window.size();
    let agent_visual_radius = 15.0 * scale_factor;
    let agent_material = ColorMaterial {
        color: Color::BLUE,
        ..Default::default()
    };

    //let's do it on a timer first
    let start = Instant::now();

    window.render_loop(move |frame_input| {
        // let width = window.viewport().width;
        // let height = window.viewport().height;

        let mut circles: Vec<Gm<Circle, ColorMaterial>> = vec![];

        // need to put some scaling logic here
        // positions will be (-enc_size,enc_size)
        // true center will be
        let (x_center, y_center) = ((width / 2) as f32, (height / 2) as f32);

        let shorter_radius = (height / 2) as f32;
        let scalar = shorter_radius / enclosure_edge;

        //run a replay because simpler style for now
        //
        let elapsed_s = start.elapsed().as_millis() as f64 / 1000.0;
        println!("{}", elapsed_s);
        let current_step = (((elapsed_s as f32) / (dt as f32)).round() as i32) % steps;
        println!("{}", current_step);

        for agent_pos in pos_history[current_step as usize].rows() {
            let agent_x = (agent_pos[0] * scalar) + x_center;
            let agent_y = (agent_pos[1] * scalar) + y_center;
            circles.push(Gm::new(
                Circle::new(
                    &context,
                    vec2(agent_x, agent_y) * scale_factor,
                    agent_visual_radius,
                ),
                agent_material.clone(),
            ));
        }

        // calibration point for the center
        //
        //
        let edge_marker_radius = 3.0 * scale_factor;

        // circles.push(Gm::new(
        //     Circle::new(
        //         &context,
        //         vec2(x_center, y_center) * scale_factor,
        //         edge_marker_radius,
        //     ),
        //     ColorMaterial {
        //         color: Color::RED,
        //         ..Default::default()
        //     },
        // ));

        //edge markers
        for i in [-1.0, 0.0, 1.0].iter() {
            for j in [-1.0, 0.0, 1.0] {
                circles.push(Gm::new(
                    Circle::new(
                        &context,
                        vec2(x_center + shorter_radius * i, y_center + shorter_radius * j)
                            * scale_factor,
                        edge_marker_radius,
                    ),
                    ColorMaterial {
                        color: Color::RED,
                        ..Default::default()
                    },
                ));
            }
        }
        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.0, 0.0, 0.0, 1.0, 1.0))
            .render(&camera2d(frame_input.viewport), circles.iter(), &[]);
        FrameOutput::default()
    });
    // camera2d(fram_input.viewport)
}
