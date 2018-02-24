extern crate imageproc;
extern crate image;
extern crate nalgebra;

use std::fs::File;
use std::path::Path;
use nalgebra::Vector2;

use imageproc::drawing::*;

fn rk4_integrate(r: &Vector2<f64>, f: (&Fn(Vector2<f64>) -> f64, &Fn(Vector2<f64>) -> f64), h: f64) -> Vector2<f64> {
    let k1 = Vector2::new(f.0(*r), f.1(*r));
    let k2 = Vector2::new(f.0(*r+(h/2.0)*k1), f.1(*r+(h/2.0)*k1));
    let k3 = Vector2::new(f.0(*r+(h/2.0)*k2), f.1(*r+(h/2.0)*k2));
    let k4 = Vector2::new(f.0(*r+h*k3), f.1(*r+h*k3));
    r+(h/6.0)*(k1+2.0*k2+2.0*k3+k4)
}

struct PhasePos {
    pos: Vector2<f64>,
    bounds: Option<(f64, f64)>,
    bounds_l: f64,
}

impl PhasePos {
    fn new(p: Vector2<f64>) -> PhasePos {
        PhasePos{pos: p, bounds: None, bounds_l: 0.0}
    }
    fn new_bounded(p: Vector2(f64), b: (f64, f64)) -> PhasePos {
        PhasePos{pos: p, bounds: Some(b), bounds_l: b.1-b.0}
    }
}

impl Iterator for PhasePos {
    type Item = Vector2<f64>;
    fn next(&mut self) -> Option<Vector2<f64>> {
        Some(Vector2::new(1.0, 2.0))
    }
}

fn main() {
    let imgx=800;
    let imgy=800;

    let mut img = image::DynamicImage::new_rgb8(imgx, imgy);
    let ref mut fout = File::create("phase.png").unwrap();

    //draw_line_segment_mut(&mut img, (0.0, 0.0), (100.0, 100.0), image::Rgba([1000, 10, 10, 10]));
    //img.save(fout, image::PNG);
    println!("{}", (-1.23)%0.23);
}
