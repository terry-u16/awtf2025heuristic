mod anneling;
mod clustering;
mod naive;
mod rand_greedy;

use crate::problem::{Input, Output};

pub fn solve(input: &Input) -> Output {
    let clusters = clustering::make_clusters(input);
    self::anneling::solve(input, clusters)
}
