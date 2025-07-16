use crate::grid::{Coord, CoordIndex, Map2d, D, L, R, U};
use itertools::Itertools;
use proconio::{input, marker::Chars};

pub(super) struct Input {
    pub robot_count: usize,
    pub init_robots: Vec<CoordIndex>,
    pub robot_maps: Map2d<Option<usize>>,
    pub destinations: Vec<CoordIndex>,
    pub init_walls_v: Map2d<bool>,
    pub init_walls_h: Map2d<bool>,
    pub init_graph: Map2d<[Option<CoordIndex>; 4]>,
}

impl Input {
    pub const MAP_SIZE: usize = 30;

    pub fn read() -> Self {
        input! {
            n: usize,
            robot_count: usize,
            robots: [(usize, usize, usize, usize); robot_count],
            v: [Chars; n],
            h: [Chars; n - 1],
        }

        let init_robots = robots
            .iter()
            .map(|(r, c, _, _)| Coord::new(*r, *c).to_index(Input::MAP_SIZE))
            .collect_vec();
        let robot_maps = Map2d::with_default(Self::MAP_SIZE);
        let destinations = robots
            .iter()
            .map(|(_, _, r, c)| Coord::new(*r, *c).to_index(Input::MAP_SIZE))
            .collect_vec();
        let mut init_walls_v = Map2d::with_default(Input::MAP_SIZE);
        let mut init_walls_h = Map2d::with_default(Input::MAP_SIZE);
        let mut init_graph = Map2d::from_fn(|_| [None; 4], Input::MAP_SIZE);

        for row in 0..Input::MAP_SIZE {
            for col in 0..Input::MAP_SIZE {
                init_walls_v[row][col] = v[row][col] == '1';
            }
        }

        for row in 0..Input::MAP_SIZE - 1 {
            for col in 0..Input::MAP_SIZE {
                init_walls_h[row][col] = h[row][col] == '1';
            }
        }

        for row in 0..Input::MAP_SIZE {
            for col in 0..Input::MAP_SIZE {
                let c = Coord::new(row, col);

                // Up
                if row > 0 && !init_walls_h[row - 1][col] && !init_walls_v[row][col] {
                    init_graph[c][U] = Some(c.to_index(Input::MAP_SIZE - 1));
                }

                // Right
                if col < Input::MAP_SIZE - 1
                    && !init_walls_v[row][col + 1]
                    && !init_walls_h[row][col]
                {
                    init_graph[c][R] = Some(c.to_index(Input::MAP_SIZE - 1));
                }

                // Down
                if row < Input::MAP_SIZE - 1
                    && !init_walls_h[row][col]
                    && !init_walls_v[row + 1][col]
                {
                    init_graph[c][D] = Some(c.to_index(Input::MAP_SIZE - 1));
                }

                // Left
                if col > 0 && !init_walls_v[row][col - 1] && !init_walls_h[row][col] {
                    init_graph[c][L] = Some(c.to_index(Input::MAP_SIZE - 1));
                }
            }
        }

        Self {
            robot_count,
            init_robots,
            robot_maps,
            destinations,
            init_walls_v,
            init_walls_h,
            init_graph,
        }
    }
}

pub(super) struct Output {
    pub walls_v: Map2d<bool>,
    pub walls_h: Map2d<bool>,
    pub actions: Vec<Action>,
    pub score: u32,
}

impl Output {
    pub(super) fn new(
        walls_v: Map2d<bool>,
        walls_h: Map2d<bool>,
        actions: Vec<Action>,
        score: u32,
    ) -> Self {
        Self {
            walls_v,
            walls_h,
            actions,
            score,
        }
    }
}

pub enum Action {
    /// グループ全体を動かす
    Group(Move),
    /// 個別のロボットを動かす
    Robot(Move),
}

pub struct Move {
    /// index of robot or group
    pub index: usize,
    /// U, R, D, L
    pub direction: usize,
}
