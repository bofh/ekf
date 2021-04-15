extern crate nalgebra as na;
use na::Vector3;

mod ekf {
    use na::{SVector, Vector4, Matrix4x3, Vector3};

    type StateVector = SVector<f32, 7>;

    #[derive(Debug)]
    pub struct Filter {
        x: StateVector
    }

    impl Filter {
        pub fn quaternion(self) -> Vector4<f32> {
            Vector4::from(self.x.fixed_rows::<4>(0))
        }

        fn quaternion_matrix(self) -> Matrix4x3<f32> {
            let q = self.quaternion();
            Matrix4x3::new(-q[1], -q[2], -q[3],
                            q[0], -q[3],  q[2],
                            q[3],  q[0], -q[1],
                           -q[2],  q[1],  q[0])
        }

        pub fn predict(self, _w: Vector3<f32>, _dt: f32) {
        }

    }

    pub fn new() -> Filter {
        Filter {
            x: StateVector::ith(0, 1.0)
        }
    }
}

fn main() {
    let x = ekf::new();
    let w: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
    x.predict(w, 0.02);
}
