use crate::{
    annealing::{self, run_annealing, SimdSelector, SingleScore},
    grid::{Coord, Map2d, D, L, R, U},
    problem::{Action, Input, Move, Output},
    random::RandExtension,
    solver::naive,
};
use std::{cmp::Reverse, time::Duration};

pub(super) fn solve(input: &Input) -> Output {
    let env = Env::new(input.clone(), 5);
    let state = State::new(
        input,
        (0..input.robot_count).collect(),
        (0..input.robot_count).map(|i| i % env.max_group).collect(),
    );
    let (state, stats) = run_annealing::<Neighbors, SimdSelector, 1>(
        &env,
        state,
        1e-2,
        1e-2,
        Duration::from_millis(1900).saturating_sub(input.since.elapsed()),
        42,
    );

    eprintln!("{}", stats);

    state.to_output(input)
}

neighbors! {
    Neighbors,
    Env,
    State,
    [
        AddActionNeigh => 1.0,
        RemoveActionNeigh => 1.0,
        ChangeGroupNeigh => 1.0,
        SwapPermNeigh => 1.0,
        ToggleWall => 0.0,
        SwapActionOrderNeigh => 1.0,
    ]
}

struct Env {
    input: Input,
    max_group: usize,
}

impl Env {
    fn new(input: Input, max_group: usize) -> Self {
        Self { input, max_group }
    }
}

#[derive(Clone, Debug)]
struct State {
    perm: Vec<usize>,
    wall_v: Map2d<bool>,
    wall_h: Map2d<bool>,
    groups: Vec<usize>,
    group_actions: Vec<Move>,
}

impl State {
    fn new(input: &Input, perm: Vec<usize>, groups: Vec<usize>) -> Self {
        Self {
            perm,
            wall_v: input.init_walls_v.clone(),
            wall_h: input.init_walls_h.clone(),
            groups,
            group_actions: vec![],
        }
    }
}

impl State {
    fn to_output(&self, input: &Input) -> Output {
        let mut input = input.clone();
        input.init_walls_v = self.wall_v.clone();
        input.init_walls_h = self.wall_h.clone();
        input.init_graph = build_graph(&input.init_walls_v, &input.init_walls_h);

        for mv in &self.group_actions {
            self.move_group(&mut input, mv);
        }

        let mut output = naive::solve_greedy(&input, &self.perm);

        let mut actions = self
            .group_actions
            .iter()
            .map(|&mv| Action::Group(mv))
            .collect::<Vec<_>>();
        actions.extend(output.actions);
        output.actions = actions;

        output.groups = self.groups.clone();
        output
    }

    fn move_group(&self, input: &mut Input, mv: &Move) {
        let mut robots = (0..input.robot_count)
            .filter(|&i| self.groups[i] == mv.index)
            .collect::<Vec<_>>();

        match mv.direction {
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

            if let Some(new_pos) = input.init_graph[current_pos][mv.direction] {
                if input.robot_maps[new_pos].is_none() {
                    input.robot_maps[current_pos] = None;
                    input.robot_maps[new_pos] = Some(robot_id);
                    input.init_robots[robot_id] = new_pos;
                }
            }
        }
    }
}

impl annealing::State for State {
    type Env = Env;
    type Score = SingleScore;

    fn score(&self, env: &Self::Env) -> Self::Score {
        let mut input = env.input.clone();
        input.init_walls_v = self.wall_v.clone();
        input.init_walls_h = self.wall_h.clone();
        input.init_graph = build_graph(&input.init_walls_v, &input.init_walls_h);

        for mv in &self.group_actions {
            self.move_group(&mut input, mv);
        }

        let score =
            naive::solve_greedy(&input, &self.perm).score() + self.group_actions.len() as u32;

        SingleScore(-(score as f64))
    }
}

fn build_graph(walls_v: &Map2d<bool>, walls_h: &Map2d<bool>) -> Map2d<[Option<Coord>; 4]> {
    let mut graph = Map2d::from_fn(|_| [None; 4], Input::MAP_SIZE);

    for row in 0..Input::MAP_SIZE {
        for col in 0..Input::MAP_SIZE {
            let c = Coord::new(row, col);
            let directions = [
                (
                    U,
                    (row.wrapping_sub(1), col),
                    row > 0 && !walls_h[row - 1][col],
                ),
                (
                    R,
                    (row, col + 1),
                    col < Input::MAP_SIZE - 1 && !walls_v[row][col],
                ),
                (
                    D,
                    (row + 1, col),
                    row < Input::MAP_SIZE - 1 && !walls_h[row][col],
                ),
                (
                    L,
                    (row, col.wrapping_sub(1)),
                    col > 0 && !walls_v[row][col - 1],
                ),
            ];

            for (dir, (new_row, new_col), can_move) in directions {
                if can_move && new_row < Input::MAP_SIZE && new_col < Input::MAP_SIZE {
                    graph[c][dir] = Some(Coord::new(new_row, new_col));
                }
            }
        }
    }

    graph
}

struct AddActionNeigh {
    index: usize,
    mv: Move,
}

impl annealing::Neighbor for AddActionNeigh {
    type Env = Env;
    type State = State;

    fn generate(
        env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        let (index, group_id, dir) =
            rng.fast_gen_range_u16x3(0..=state.group_actions.len(), 0..env.max_group, 0..4);
        let mv = Move::new(group_id, dir);

        Some(Self { index, mv })
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.insert(self.index, self.mv);
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.remove(self.index);
    }
}

struct RemoveActionNeigh {
    index: usize,
    mv: Move,
}

impl annealing::Neighbor for RemoveActionNeigh {
    type Env = Env;
    type State = State;

    fn generate(
        _env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        if state.group_actions.is_empty() {
            return None;
        }

        let index = rng.fast_gen_range_u16x1(0..state.group_actions.len());
        let mv = state.group_actions[index];
        Some(Self { index, mv })
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.remove(self.index);
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.insert(self.index, self.mv);
    }
}

struct ChangeGroupNeigh {
    index: usize,
    old_group: usize,
    new_group: usize,
}

impl annealing::Neighbor for ChangeGroupNeigh {
    type Env = Env;
    type State = State;

    fn generate(
        env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        let index = rng.fast_gen_range_u16x1(0..state.groups.len());
        let old_group = state.groups[index];
        let new_group = loop {
            let new_group = rng.fast_gen_range_u16x1(0..env.max_group);
            if new_group != old_group {
                break new_group;
            }
        };

        Some(Self {
            index,
            old_group,
            new_group,
        })
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.groups[self.index] = self.new_group;
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.groups[self.index] = self.old_group;
    }
}

struct SwapPermNeigh {
    index0: usize,
    index1: usize,
}

impl annealing::Neighbor for SwapPermNeigh {
    type Env = Env;
    type State = State;

    fn generate(
        _env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        loop {
            let (index0, index1) =
                rng.fast_gen_range_u16x2(0..state.perm.len(), 0..state.perm.len());

            if index0 == index1 {
                continue;
            }

            return Some(Self {
                index0: index0 as usize,
                index1: index1 as usize,
            });
        }
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.perm.swap(self.index0, self.index1);
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.perm.swap(self.index0, self.index1);
    }
}

struct ToggleWall {
    coord: Coord,
    is_vertical: bool,
}

impl annealing::Neighbor for ToggleWall {
    type Env = Env;
    type State = State;

    fn generate(
        _env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        loop {
            if rng.fast_gen_range_u16x1(0..2) == 1 {
                let (row, col) =
                    rng.fast_gen_range_u16x2(0..Input::MAP_SIZE, 0..Input::MAP_SIZE - 1);
                let c = Coord::new(row, col);

                if !state.wall_v[c] {
                    return Some(Self {
                        coord: c,
                        is_vertical: true,
                    });
                }
            } else {
                let (row, col) =
                    rng.fast_gen_range_u16x2(0..Input::MAP_SIZE - 1, 0..Input::MAP_SIZE);
                let c = Coord::new(row, col);

                if !state.wall_h[c] {
                    return Some(Self {
                        coord: c,
                        is_vertical: false,
                    });
                }
            }
        }
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        if self.is_vertical {
            state.wall_v[self.coord] ^= true;
        } else {
            state.wall_h[self.coord] ^= true;
        }
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        if self.is_vertical {
            state.wall_v[self.coord] ^= true;
        } else {
            state.wall_h[self.coord] ^= true;
        }
    }
}

struct SwapActionOrderNeigh {
    index0: usize,
    index1: usize,
}

impl annealing::Neighbor for SwapActionOrderNeigh {
    type Env = Env;
    type State = State;

    fn generate(
        _env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        if state.group_actions.len() < 2 {
            return None;
        }

        loop {
            let (index0, index1) = rng
                .fast_gen_range_u16x2(0..state.group_actions.len(), 0..state.group_actions.len());

            if index0 == index1 {
                continue;
            }

            return Some(Self {
                index0: index0 as usize,
                index1: index1 as usize,
            });
        }
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.swap(self.index0, self.index1);
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.group_actions.swap(self.index0, self.index1);
    }
}
