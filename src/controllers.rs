use ndarray::{array, s, Array, Array1, Array2, Axis};
use ndarray_linalg::*;

pub trait AgentController {
    fn local_controller(
        neighbors_pos: &Array2<f32>,
        neighbors_vel: &Array2<f32>,
        self_pos: &Array1<f32>,
        self_vel: &Array1<f32>,
    ) -> Array1<f32> {
        Array::zeros(2)
    }
}

pub struct Boids;
impl AgentController for Boids {
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
        let sep_gain = 3.0;
        let inertia = 1.0;
        let out = inertia * self_vel
            + coh_gain * (pos_centroid - self_pos)
            + ali_gain * vel_centroid
            + sep_gain * sep;

        return out;
    }
}
