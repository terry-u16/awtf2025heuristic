use crate::{
    annealing::{self, run_annealing, SimdSelector, SingleScore},
    data_structures::IndexSet,
    grid::{Coord, Map2d, D, L, R, U},
    problem::{Action, Input, Move, Output},
    random::RandExtension,
    solver::naive,
};
use rand::Rng;
use std::{cmp::Reverse, time::Duration};

pub(super) fn solve(input: &Input) -> Output {
    let env = Env::new(input.clone(), 5);
    let groups = (0..input.robot_count)
        .map(|i| {
            let dr = input.destinations[i].row() as i32 - input.init_robots[i].row() as i32;
            let dc = input.destinations[i].col() as i32 - input.init_robots[i].col() as i32;

            if dr.abs() > dc.abs() {
                if dr > 0 {
                    D
                } else {
                    U
                }
            } else {
                if dc > 0 {
                    R
                } else {
                    L
                }
            }
        })
        .collect();
    let state = State::new(&env, (0..input.robot_count).collect(), groups);
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
    wall_candidates_v: Vec<Coord>,
    wall_candidates_h: Vec<Coord>,
}

impl Env {
    fn new(input: Input, max_group: usize) -> Self {
        let wall_candidates_v = (0..Input::MAP_SIZE)
            .flat_map(|row| (0..Input::MAP_SIZE - 1).map(move |col| Coord::new(row, col)))
            .filter(|c| !input.init_walls_v[c])
            .collect();
        let wall_candidates_h = (0..Input::MAP_SIZE - 1)
            .flat_map(|row| (0..Input::MAP_SIZE).map(move |col| Coord::new(row, col)))
            .filter(|c| !input.init_walls_h[c])
            .collect();

        Self {
            input,
            max_group,
            wall_candidates_v,
            wall_candidates_h,
        }
    }
}

#[derive(Clone, Debug)]
struct State {
    perm: Vec<usize>,
    wall_v: Map2d<bool>,
    wall_h: Map2d<bool>,
    groups: Vec<usize>,
    group_actions: Vec<Move>,
    unused_walls_v: IndexSet,
    unused_walls_h: IndexSet,
    used_walls_v: IndexSet,
    used_walls_h: IndexSet,
}

impl State {
    fn new(env: &Env, perm: Vec<usize>, groups: Vec<usize>) -> Self {
        let mut unused_walls_v = IndexSet::new(env.wall_candidates_v.len());
        let mut unused_walls_h = IndexSet::new(env.wall_candidates_h.len());
        let used_walls_v = IndexSet::new(env.wall_candidates_v.len());
        let used_walls_h = IndexSet::new(env.wall_candidates_h.len());

        for i in 0..env.wall_candidates_v.len() {
            unused_walls_v.add(i);
        }

        for i in 0..env.wall_candidates_h.len() {
            unused_walls_h.add(i);
        }

        Self {
            perm,
            wall_v: env.input.init_walls_v.clone(),
            wall_h: env.input.init_walls_h.clone(),
            groups,
            group_actions: vec![],
            unused_walls_v: unused_walls_v,
            unused_walls_h: unused_walls_h,
            used_walls_v: used_walls_v,
            used_walls_h: used_walls_h,
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
    is_vertical: bool,
    coord_index: usize,
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
        if rng.gen_bool(0.1) {
            // 壁追加
            if rng.gen_bool(0.5) {
                let slice = state.unused_walls_v.as_slice();

                if slice.is_empty() {
                    return None;
                }

                let index = rng.fast_gen_range_u16x1(0..slice.len());
                return Some(Self {
                    is_vertical: true,
                    coord_index: index,
                });
            } else {
                let slice = state.unused_walls_h.as_slice();

                if slice.is_empty() {
                    return None;
                }

                let index = rng.fast_gen_range_u16x1(0..slice.len());
                return Some(Self {
                    coord_index: index,
                    is_vertical: false,
                });
            }
        } else {
            // 壁削除
            if rng.gen_bool(0.5) {
                let slice = state.used_walls_v.as_slice();

                if slice.is_empty() {
                    return None;
                }

                let index = rng.fast_gen_range_u16x1(0..slice.len());
                return Some(Self {
                    coord_index: index,
                    is_vertical: true,
                });
            } else {
                let slice = state.used_walls_h.as_slice();

                if slice.is_empty() {
                    return None;
                }

                let index = rng.fast_gen_range_u16x1(0..slice.len());
                return Some(Self {
                    coord_index: index,
                    is_vertical: false,
                });
            }
        }
    }

    fn preprocess(&mut self, env: &Self::Env, state: &mut Self::State) {
        if self.is_vertical {
            state.wall_v[env.wall_candidates_v[self.coord_index]] ^= true;
        } else {
            state.wall_h[env.wall_candidates_h[self.coord_index]] ^= true;
        }
    }

    fn postprocess(self, _env: &Self::Env, state: &mut Self::State) {
        if self.is_vertical {
            if state.unused_walls_v.contains(self.coord_index) {
                state.unused_walls_v.remove(self.coord_index);
                state.used_walls_v.add(self.coord_index);
            } else {
                state.used_walls_v.remove(self.coord_index);
                state.unused_walls_v.add(self.coord_index);
            }
        } else {
            if state.unused_walls_h.contains(self.coord_index) {
                state.unused_walls_h.remove(self.coord_index);
                state.used_walls_h.add(self.coord_index);
            } else {
                state.used_walls_h.remove(self.coord_index);
                state.unused_walls_h.add(self.coord_index);
            }
        }
    }

    fn rollback(self, env: &Self::Env, state: &mut Self::State) {
        if self.is_vertical {
            state.wall_v[env.wall_candidates_v[self.coord_index]] ^= true;
        } else {
            state.wall_h[env.wall_candidates_h[self.coord_index]] ^= true;
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
