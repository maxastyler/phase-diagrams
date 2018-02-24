extern crate imageproc;
extern crate image;
extern crate nalgebra;

use std::fs::File;
use std::path::Path;
use nalgebra::Vector2;
use std::f64::consts::PI;

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
    fn new_bounded(p: Vector2<f64>, b: (f64, f64)) -> PhasePos {
        PhasePos{pos: p, bounds: Some(b), bounds_l: b.1-b.0}
    } 
    fn wrap(&mut self, x: f64) -> f64 {
        match self.bounds {
            Some(b) => {
                let mut y = x;
                loop {
                    if y<b.0 { y+=self.bounds_l; }
                    else if y>b.1 { y-=self.bounds_l; }
                    else { break; }
                }
                y
            },
            None => x,
        }
    }
}

impl Iterator for PhasePos {
    type Item = Vector2<f64>;
    fn next(&mut self) -> Option<Vector2<f64>> {
        let mut new_pos = rk4_integrate(&self.pos, (&|x| x[1], &|x| -(x[0].sin())), 0.01);
        new_pos[0] = self.wrap(new_pos[0]);
        self.pos = new_pos;
        Some(new_pos)
    }
}

fn main() {
    let imgx=800;
    let imgy=800;

    let mut img = image::DynamicImage::new_rgb8(imgx, imgy);
    let ref mut fout = File::create("phase.png").unwrap();
    let system = PhasePos::new_bounded(Vector2::new(1.0, 0.0), (0.0, 2.0*PI));
    for i in system.take(100) {
        println!("{}", i);
    }

    //draw_line_segment_mut(&mut img, (0.0, 0.0), (100.0, 100.0), image::Rgba([1000, 10, 10, 10]));
    //img.save(fout, image::PNG);
}
