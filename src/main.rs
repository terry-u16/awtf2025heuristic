#[allow(dead_code)]
mod annealing;
#[allow(dead_code)]
mod grid;
mod problem;
#[allow(dead_code)]
mod random;
mod solver;
#[allow(dead_code)]
mod util;

fn main() {
    let input = problem::Input::read();
    let output = solver::solve(&input);
    output.write();
}
