use crate::{
    grid::{D, L, R, U},
    problem::{Action, Input, Move, Output},
    solver::naive,
};
use rand::Rng;
use rand_pcg::Pcg64Mcg;
use std::cmp::Reverse;

pub(super) fn solve(input: &Input) -> Output {
    let mut rng = Pcg64Mcg::new(42);
    let mut best_output = Output::new(
        input.init_walls_v.clone(),
        input.init_walls_h.clone(),
        (0..input.robot_count).collect(),
        vec![],
        u32::MAX,
    );

    while input.since.elapsed().as_millis() < 1900 {
        let mut input = input.clone();
        let group_count = rng.gen_range(1..=10);
        let groups = (0..input.robot_count)
            .map(|_| rng.gen_range(0..group_count))
            .collect::<Vec<_>>();

        let mut actions = vec![];

        for _ in 0..30 {
            let group = rng.gen_range(0..group_count);
            let dir = rng.gen_range(0..4);

            if move_group(&mut input, &groups, group, dir) {
                actions.push(Action::Group(Move::new(group, dir)));
            }
        }

        let mut output = naive::solve(&input);

        output.groups = groups;
        output.walls_v = input.init_walls_v.clone();
        output.walls_h = input.init_walls_h.clone();

        actions.extend(output.actions);
        output.actions = actions;

        if output.score() < best_output.score() {
            best_output = output;
            break;
        }
    }

    eprintln!("Best score: {}", best_output.score());

    best_output
}

fn move_group(input: &mut Input, groups: &[usize], group_id: usize, dir: usize) -> bool {
    let mut robots = (0..input.robot_count)
        .filter(|&i| groups[i] == group_id)
        .collect::<Vec<_>>();

    if robots.is_empty() {
        return false;
    }

    match dir {
        U => {
            robots.sort_unstable_by_key(|&i| input.init_robots[i].row());
        }
        R => {
            robots.sort_unstable_by_key(|&i| Reverse(input.init_robots[i].col()));
        }
        D => {
            robots.sort_unstable_by_key(|&i| Reverse(input.init_robots[i].row()));
        }
        L => {
            robots.sort_unstable_by_key(|&i| input.init_robots[i].col());
        }
        _ => unreachable!(),
    }

    for &robot_id in &robots {
        let current_pos = input.init_robots[robot_id];

        if let Some(new_pos) = input.init_graph[current_pos][dir] {
            if input.robot_maps[new_pos].is_none() {
                input.robot_maps[current_pos] = None;
                input.robot_maps[new_pos] = Some(robot_id);
                input.init_robots[robot_id] = new_pos;
            }
        }
    }

    true
}
