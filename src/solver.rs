mod naive;
mod rand_greedy;

use crate::problem::{Input, Output};

pub fn solve(input: &Input) -> Output {
    rand_greedy::solve(input)
}
