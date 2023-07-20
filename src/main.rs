use ndarray::{array, s, Array,Array2};
use ndarray_linalg::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;

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
    let steps = (total_time / dt) as i64;
    let n_agents = 10;
    let enclosure_edge = 10.; //m
    let neighbor_radius = 5.; //m

    let mut agents_pos:Array2<f64> =
        Array::random((n_agents, 2), Uniform::new(-enclosure_edge, enclosure_edge));

    println!("{}", agents_pos);

    println!("{}", agents_pos.slice(s![0, ..]));
    
    // let mut top_agent = agents_pos.slice_mut(s![0, ..]);

    for mut agent in agents_pos.rows_mut() {
        // println!("{}", agent);
        //need to understand a little more how to handle this, I just want to reassign some values
        //in bulk
        let new:Array<f64,_> = array![agent[1],agent[0]];

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
            let mut neighbors:Vec<usize> = vec![];
            for j in 0..n_agents {
                if i != j {
                    let agent = agents_pos.slice(s![i, ..]).to_owned();
                    let other = agents_pos.slice(s![j, ..]).to_owned();
                    let diff = agent - other;
                    if diff.norm() > neighbor_radius{
                        //is neighbor, now do stuff
                        neighbors.push(j);
                    }
                }
                //some kind of handle neighbors call
            }
        }
        //some kind of logic to handle between state transitions, time changing, etc.
    }
}
