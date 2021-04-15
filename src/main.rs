extern crate nalgebra as na;
use na::Vector3;

mod ekf {
    use na::{Matrix3, Matrix3x4, Matrix4, Matrix4x3, SVector, Vector3, Vector4, SMatrix};

    type Matrix7 = SMatrix<f32, 7, 7>;

    trait BlockMatrix {
        fn from_four(m00: Matrix4<f32>, m01: SMatrix<f32, 4, 3>, m10: Matrix3x4<f32>, m11: Matrix3<f32>) -> Matrix7;
    }

    impl BlockMatrix for SMatrix<f32, 7, 7> {
        fn from_four(m00: Matrix4<f32>, m01: SMatrix<f32, 4, 3>, m10: Matrix3x4<f32>, m11: Matrix3<f32>) -> Matrix7 {
            // Fix this code, especially default branch
            Matrix7::from_fn(|r, c|
                match r {
                    0..=3 => match c {
                        0..=3 => m00[(r, c)],
                        4..=6 => m01[(r, c - 4)],
                        _ => 1000.0
                    }
                    4..=6 => match c {
                        0..=3 => m10[(r - 4, c)],
                        4..=6 => m11[(r - 4, c - 4)],
                        _ => 2000.0
                    }
                    _ => 3000.0
                }
            )
        }
    }

    type StateVector = SVector<f32, 7>;

    #[derive(Debug)]
    pub struct Filter {
        x: StateVector,
    }

    impl Filter {
        pub fn quaternion(self) -> Vector4<f32> {
            Vector4::from(self.x.fixed_rows::<4>(0))
        }

        fn quaternion_matrix(self) -> Matrix4x3<f32> {
            let q = self.quaternion();
            Matrix4x3::new(
                -q[1], -q[2], -q[3],
                 q[0], -q[3],  q[2],
                 q[3],  q[0], -q[1],
                -q[2],  q[1],  q[0],
            )
        }

        pub fn predict(self, _w: Vector3<f32>, dt: f32) -> Matrix7 {
            let m00: Matrix4<f32> = Matrix4::identity();
            let m01 = (-dt / 2.0) * self.quaternion_matrix();
            let m10: Matrix3x4<f32> = Matrix3x4::zeros();
            let m11: Matrix3<f32> = Matrix3::identity();
            Matrix7::from_four(m00, m01, m10, m11)
        }
    }

    pub fn new() -> Filter {
        Filter {
            x: StateVector::ith(0, 1.0),
        }
    }
}

fn main() {
    let x = ekf::new();
    let w: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);
    let test = x.predict(w, 0.02);
    println!("{}", test);
}
