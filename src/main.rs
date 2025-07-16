#[allow(dead_code)]
mod annealing;
#[allow(dead_code)]
mod grid;
mod problem;
mod solver;
#[allow(dead_code)]
mod util;
#[allow(dead_code)]
mod random;

fn main() {
    let input = problem::Input::read();
    let output = solver::solve(&input);
    output.write();
}
