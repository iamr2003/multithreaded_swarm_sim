use ndarray::{array, s, Array, Array1, Array2, Axis};
use ndarray_linalg::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;
use std::time::Instant;
use three_d::*;

// spits out velocity
fn local_controller(
    neighbors_pos: &Array2<f32>,
    neighbors_vel: &Array2<f32>,
    self_pos: &Array1<f32>,
    self_vel: &Array1<f32>,
) -> Array1<f32> {
    //let's start with basic boids

    let pos_centroid = match neighbors_pos.mean_axis(Axis(0)) {
        Some(res) => res,
        None => Array::zeros(2),
    };
    let vel_centroid = match neighbors_vel.mean_axis(Axis(0)) {
        Some(res) => res,
        None => Array::zeros(2),
    };

    let mut sep: Array1<f32> = Array::zeros(2);

    //might be able to write more efficiently with nd ops, but can't figure out rn
    for n_pos in neighbors_pos.rows() {
        let rel_pos = n_pos.to_owned() - self_pos;
        let dist_weighted = -1.0 / rel_pos.norm().pow(2.0);
        let unit_rel_pos = rel_pos.to_owned() / rel_pos.norm();
        sep = sep + (dist_weighted * unit_rel_pos);
    }

    let coh_gain = 1.0;
    let ali_gain = 1.0;
    let sep_gain = 5.0;
    let inertia = 1.0;
    let out = inertia * self_vel
        + coh_gain * (pos_centroid - self_pos)
        + ali_gain * vel_centroid
        + sep_gain * sep;

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
    let n_agents = 1000;
    let enclosure_edge = 10.; //m
    let max_init_vel = 10.; //m/s
    let max_vel = 10.;
    let neighbor_radius = 1.0; //m
                               //
                               //already feelings slowness trying 0.01 dt

    let mut agents_pos: Array2<f32> =
        Array::random((n_agents, 2), Uniform::new(-enclosure_edge, enclosure_edge));

    //let's give some initial vel
    //consider using a different distribution
    let mut agents_vel: Array2<f32> =
        Array::random((n_agents, 2), Uniform::new(-max_init_vel, max_init_vel));
    // Array::zeros((n_agents, 2));

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

    //
    for _ in 0..steps {
        let mut next_agents_pos: Array2<f32> = agents_pos.clone();
        let mut next_agents_vel: Array2<f32> = agents_vel.clone();
        //find neighbors, the slow way

        //spatial hash, create occupancy grid
        // row,index,agent
        // REPHRASE AS IN TERMS OF NUMBER OF ENTRIES, MAKE IT EASY TO RECOMPUTE

        let grid_size: usize = f32::ceil((enclosure_edge * 2.0) / neighbor_radius) as usize;
        let mut occupancy: Vec<Vec<Vec<usize>>> = vec![vec![vec![]; grid_size]; grid_size];
        for i in 0..n_agents {
            let agent_pos = agents_pos.slice(s![i, ..]).to_owned();
            let x_hash = f32::floor((agent_pos[0] + enclosure_edge) / neighbor_radius) as usize;
            let y_hash = f32::floor((agent_pos[1] + enclosure_edge) / neighbor_radius) as usize;
            occupancy[x_hash][y_hash].push(i);
        }

        for i in 0..n_agents {
            let mut neighbors_pos: Vec<Array1<f32>> = vec![];
            let mut neighbors_vel: Vec<Array1<f32>> = vec![];

            let agent_pos = agents_pos.slice(s![i, ..]).to_owned();
            let x_hash = f32::floor((agent_pos[0] + enclosure_edge) / neighbor_radius) as i32;
            let y_hash = f32::floor((agent_pos[1] + enclosure_edge) / neighbor_radius) as i32;

            let agent_vel = agents_vel.slice(s![i, ..]).to_owned();

            // calculate neighbors
            // check all adjacent cells including the one within
            // not allowing me to start with -1 for some reason

            //inelegant, would like a better way
            for r in -1..=1 {
                for c in -1..=1 {
                    //NEED TO MAKE SURE GET HAS SAFETIES I WANT, usize wrap might be a problem
                    match occupancy.get((&x_hash + r) as usize) {
                        Some(row) => match row.get((&y_hash + c) as usize) {
                            Some(cell) => {
                                for j in cell.iter() {
                                    if i != *j {
                                        let other = agents_pos.slice(s![*j, ..]).to_owned();
                                        let diff = agent_pos.to_owned() - other.to_owned();
                                        if diff.norm() > neighbor_radius {
                                            //is neighbor, now do stuff
                                            neighbors_pos.push(other);
                                            neighbors_vel
                                                .push(agents_vel.slice(s![*j, ..]).to_owned());
                                        }
                                    }
                                }
                            }
                            None => {}
                        },
                        None => {}
                    }
                }
            }
            //I think I was doing wayyy too many recomputations for no good reason
            //some kind of handle neighbors call

            //some fun conversions from vec to array, might have to build my own implem
            let neighbors_pos_flat: Vec<f32> = neighbors_pos.iter().flatten().cloned().collect();
            let neighbors_vel_flat: Vec<f32> = neighbors_vel.iter().flatten().cloned().collect();

            let neighbors_pos_nd =
                Array2::from_shape_vec((neighbors_pos.len(), 2), neighbors_pos_flat);
            let neighbors_vel_nd =
                Array2::from_shape_vec((neighbors_vel.len(), 2), neighbors_vel_flat);

            //something weird is happening RELATED TO UNWRAPS-- they shouldn't still
            //when no interaction, continue same direction
            let mut agent_next_vel = match neighbors_pos_nd {
                Ok(n_pos) => match neighbors_vel_nd {
                    Ok(n_vel) => local_controller(&n_pos, &n_vel, &agent_pos, &agent_vel),
                    Err(..) => agent_vel.to_owned(),
                },
                Err(..) => agent_vel.to_owned(),
            };

            //should not be needed, but it is
            if neighbors_pos.len() == 0 {
                agent_next_vel = agent_vel.to_owned();
            }

            //vel magnitude clipping
            if agent_next_vel.norm() > max_vel {
                agent_next_vel = agent_next_vel.clone() * max_vel / agent_next_vel.norm();
            }

            let agent_next_pos = agent_next_vel.clone() * dt + agent_pos.to_owned();

            //eventually boundary conditions and velocity limits

            //simple bounce condition by dimension
            if agent_next_pos[0] > enclosure_edge && agent_next_vel[0] > 0.0 {
                agent_next_vel[0] *= -1.0;
            }

            if agent_next_pos[0] < -enclosure_edge && agent_next_vel[0] < 0.0 {
                agent_next_vel[0] *= -1.0;
            }

            if agent_next_pos[1] > enclosure_edge && agent_next_vel[1] > 0.0 {
                agent_next_vel[1] *= -1.0;
            }

            if agent_next_pos[1] < -enclosure_edge && agent_next_vel[1] < 0.0 {
                agent_next_vel[1] *= -1.0;
            }

            //updating again after boundary update
            let agent_next_pos = agent_next_vel.clone() * dt + agent_pos.to_owned();

            //  assign for specific agent
            next_agents_pos.slice_mut(s![i, ..]).assign(&agent_next_pos);
            next_agents_vel.slice_mut(s![i, ..]).assign(&agent_next_vel);
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
    let agent_visual_radius = 10.0 * scale_factor;
    let agent_material = ColorMaterial {
        color: Color::BLUE,
        ..Default::default()
    };

    let start = Instant::now();
    // I would like a ring around each circle as the neighbor radius, so I can verify what's
    // happening
    // there is still some weird merging that occurs

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
        // println!("{}", elapsed_s);
        let current_step = (((elapsed_s as f32) / (dt as f32)).round() as i32) % steps;
        // println!("{}", current_step);

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
