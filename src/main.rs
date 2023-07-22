use ndarray::{array, s, Array, Array1, Array2, Axis};
use ndarray_linalg::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;

// spits out velocity
fn local_controller(
    self_agent: usize,
    neighbors: &Vec<usize>,
    agents_pos: &Array2<f64>,
    agents_vel: &Array2<f64>,
) -> Array1<f64> {
    //let's start with basic boids
    let curr_pos = agents_pos.slice(s![self_agent, ..]);
    let pos_centroid = agents_pos.mean_axis(Axis(0)).unwrap();
    let vel_centroid = agents_pos.mean_axis(Axis(0)).unwrap();

    let sep: Array1<f64> = Array::zeros(2);
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

    let total_time = 10.0; //s
    let dt = 0.1;
    let steps = (total_time / dt) as i32;
    let n_agents = 10;
    let enclosure_edge = 10.; //m
    let neighbor_radius = 5.; //m

    let mut agents_pos: Array2<f64> =
        Array::random((n_agents, 2), Uniform::new(-enclosure_edge, enclosure_edge));
    let mut agents_vel: Array2<f64> = Array::zeros((n_agents, 2));

    println!("{}", agents_pos.mean_axis(Axis(0)).unwrap());

    println!("{}", agents_pos.slice(s![0, ..]));

    // let mut top_agent = agents_pos.slice_mut(s![0, ..]);

    for mut agent in agents_pos.rows_mut() {
        // println!("{}", agent);
        //need to understand a little more how to handle this, I just want to reassign some values
        //in bulk
        let new: Array<f64, _> = array![agent[1], agent[0]];

        agent.assign(&new);
        // let temp = agent.to_owned();
        // agent[0] = temp[1];
        // agent[1] = temp[0];
    }

    println!("{}", agents_pos);
    //to_owned needs to be used to clone the subview properly
    // for i in 0..n_agents {
    //     for j in 0..n_agents {
    //         let diff = agents_pos.slice(s![j, ..]).to_owned()  -agents_pos.slice(s![i, ..]).to_owned();
    //         println!("{}", diff);
    //     }
    // }

    for _ in 0..steps {
        //find neighbors, the slow way
        for i in 0..n_agents {
            let mut neighbors: Vec<usize> = vec![];
            for j in 0..n_agents {
                if i != j {
                    let agent = agents_pos.slice(s![i, ..]).to_owned();
                    let other = agents_pos.slice(s![j, ..]).to_owned();
                    let diff = agent - other;
                    if diff.norm() > neighbor_radius {
                        //is neighbor, now do stuff
                        neighbors.push(j);
                    }
                }
                //some kind of handle neighbors call
                let agent_vel = local_controller(i, &neighbors, &agents_pos, &agents_vel);
            }
        }
        //some kind of logic to handle between state transitions, time changing, etc.
    }
}
