use crate::{
    annealing::{self, SimdSelector, SingleScore},
    data_structures::IndexSet,
    problem::Input,
    random::RandExtension,
    util::ChangeMinMax,
};
use std::time::Duration;

pub(super) fn make_clusters(input: &Input) -> Vec<Vec<usize>> {
    let mut best_score = f64::MAX;
    let mut best_cnt = 0;
    let mut best_state = State::new(&Env::new(input.clone(), 2));

    for cluster_cnt in 2..=10 {
        let env = Env::new(input.clone(), cluster_cnt);
        let state = State::new(&env);
        let (state, stats) = annealing::run_annealing::<Neighbors, SimdSelector, 1>(
            &env,
            state,
            5e0,
            1e-1,
            Duration::from_millis(50),
            42,
        );

        eprintln!("{}", stats);

        if state.clusters.iter().any(|c| c.iter().count() == 0) {
            continue;
        }

        if best_score.change_min(-stats.final_score) {
            best_state = state;
            best_cnt = cluster_cnt;
        }
    }

    eprintln!("Score: {}, Clusters: {}", best_score as i32, best_cnt);
    best_state
        .clusters
        .iter()
        .map(|c| c.iter().cloned().collect())
        .collect()
}

neighbors! {
    Neighbors,
    Env,
    State,
    [
        MoveCluster => 1.0,
    ]
}

struct Env {
    input: Input,
    cluster_cnt: usize,
}

impl Env {
    fn new(input: Input, cluster_cnt: usize) -> Self {
        Self { input, cluster_cnt }
    }
}

#[derive(Debug, Clone)]
struct State {
    clusters: Vec<IndexSet>,
    cluster_ids: Vec<usize>,
}

impl State {
    fn new(env: &Env) -> Self {
        let mut clusters = vec![IndexSet::new(env.input.robot_count); env.cluster_cnt];
        for i in 0..env.input.robot_count {
            clusters[0].add(i);
        }

        let cluster_ids = vec![0; env.input.robot_count];

        Self {
            clusters,
            cluster_ids,
        }
    }
}

impl annealing::State for State {
    type Env = Env;
    type Score = SingleScore;

    fn score(&self, env: &Self::Env) -> Self::Score {
        let mut score = 0.0;

        for cluster in self.clusters.iter() {
            if cluster.len() == 0 {
                continue;
            }

            let mut dr = 0.0;
            let mut dc = 0.0;

            for robot in cluster.iter() {
                let from = env.input.init_robots[*robot];
                let to = env.input.destinations[*robot];
                dr += to.row() as f64 - from.row() as f64;
                dc += to.col() as f64 - from.col() as f64;
            }

            let dr_avg = dr / cluster.len() as f64;
            let dc_avg = dc / cluster.len() as f64;
            score += dr_avg.abs() + dc_avg.abs();

            for robot in cluster.iter() {
                let from = env.input.init_robots[*robot];
                let to = env.input.destinations[*robot];
                score += (to.row() as f64 - from.row() as f64 - dr_avg).abs();
                score += (to.col() as f64 - from.col() as f64 - dc_avg).abs();
            }
        }

        SingleScore(-score)
    }
}

struct MoveCluster {
    robot: usize,
    from: usize,
    to: usize,
}

impl annealing::Neighbor for MoveCluster {
    type Env = Env;
    type State = State;

    fn generate(
        env: &Self::Env,
        state: &Self::State,
        rng: &mut annealing::AnnealingRng,
        _progress: f64,
    ) -> Option<Self> {
        let robot = rng.fast_gen_range_u16x1(0..env.input.robot_count);
        let from = state.cluster_ids[robot] as usize;

        let to = loop {
            let to = rng.fast_gen_range_u16x1(0..env.cluster_cnt);
            if to != from {
                break to;
            }
        };

        Some(Self { robot, from, to })
    }

    fn preprocess(&mut self, _env: &Self::Env, state: &mut Self::State) {
        state.clusters[self.from].remove(self.robot);
        state.clusters[self.to].add(self.robot);
        state.cluster_ids[self.robot] = self.to;
    }

    fn postprocess(self, _env: &Self::Env, _state: &mut Self::State) {
        // do nothing
    }

    fn rollback(self, _env: &Self::Env, state: &mut Self::State) {
        state.clusters[self.to].remove(self.robot);
        state.clusters[self.from].add(self.robot);
        state.cluster_ids[self.robot] = self.from;
    }
}
