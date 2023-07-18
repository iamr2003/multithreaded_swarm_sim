use ndarray::Array;
use ndarray::array;
use ndarray_rand::RandomExt;
use ndarray_rand::rand_distr::Uniform;

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
    let steps = (total_time/dt) as i64;
    let n_agents = 10;
    let enclosure_edge = 10; //m

    let agents_pos = Array::random((n_agents,2),Uniform::new(-enclosure_edge,enclosure_edge));
    println!("{}",agents_pos);

    for agent in agents_pos.rows(){
        println!("{}",agent);
    }


        println!("{}",agents_pos[1]);

    // for i in 0..n_agents{
    //     println!("{}",agents_pos);
    // }

    // for step in 0..steps{
    //             
    // }

}
