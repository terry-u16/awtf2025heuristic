use crate::grid::{Coord, Map2d, D, L, R, U};
use itertools::Itertools;
use proconio::{input, marker::Chars};
use std::time::Instant;

#[derive(Debug, Clone)]
pub struct Input {
    pub robot_count: usize,
    pub init_robots: Vec<Coord>,
    pub robot_maps: Map2d<Option<usize>>,
    pub destinations: Vec<Coord>,
    pub init_walls_v: Map2d<bool>,
    pub init_walls_h: Map2d<bool>,
    pub init_graph: Map2d<[Option<Coord>; 4]>,
    pub since: Instant,
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

        let since = Instant::now();

        let init_robots = robots
            .iter()
            .map(|(r, c, _, _)| Coord::new(*r, *c))
            .collect_vec();
        let mut robot_maps = Map2d::with_default(Self::MAP_SIZE);
        let destinations = robots
            .iter()
            .map(|(_, _, r, c)| Coord::new(*r, *c))
            .collect_vec();

        for (i, c) in init_robots.iter().enumerate() {
            robot_maps[*c] = Some(i);
        }

        // 垂直壁: [MAP_SIZE][MAP_SIZE-1] (左右の壁)
        let init_walls_v = Map2d::from_fn(
            |coord| {
                let row = coord.row();
                let col = coord.col();
                if col < Input::MAP_SIZE - 1 {
                    v[row][col] == '1'
                } else {
                    false
                }
            },
            Input::MAP_SIZE,
        );

        // 水平壁: [MAP_SIZE-1][MAP_SIZE] (上下の壁)
        let init_walls_h = Map2d::from_fn(
            |coord| {
                let row = coord.row();
                let col = coord.col();
                if row < Input::MAP_SIZE - 1 {
                    h[row][col] == '1'
                } else {
                    false
                }
            },
            Input::MAP_SIZE,
        );

        let mut init_graph = Map2d::from_fn(|_| [None; 4], Input::MAP_SIZE);

        for row in 0..Input::MAP_SIZE {
            for col in 0..Input::MAP_SIZE {
                let c = Coord::new(row, col);

                // Up
                if row > 0 && !init_walls_h[Coord::new(row - 1, col)] {
                    init_graph[c][U] = Some(Coord::new(row - 1, col));
                }

                // Right
                if col < Input::MAP_SIZE - 1 && !init_walls_v[Coord::new(row, col)] {
                    init_graph[c][R] = Some(Coord::new(row, col + 1));
                }

                // Down
                if row < Input::MAP_SIZE - 1 && !init_walls_h[Coord::new(row, col)] {
                    init_graph[c][D] = Some(Coord::new(row + 1, col));
                }

                // Left
                if col > 0 && !init_walls_v[Coord::new(row, col - 1)] {
                    init_graph[c][L] = Some(Coord::new(row, col - 1));
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
            since,
        }
    }
}

pub struct Output {
    pub walls_v: Map2d<bool>,
    pub walls_h: Map2d<bool>,
    pub groups: Vec<usize>,
    pub actions: Vec<Action>,
    pub remaining_dist: u32,
}

impl Output {
    pub fn new(
        walls_v: Map2d<bool>,
        walls_h: Map2d<bool>,
        groups: Vec<usize>,
        actions: Vec<Action>,
        remaining_dist: u32,
    ) -> Self {
        Self {
            walls_v,
            walls_h,
            groups,
            actions,
            remaining_dist,
        }
    }

    pub fn score(&self) -> u32 {
        self.actions.len() as u32 + self.remaining_dist * 100
    }

    pub fn write(&self) {
        // 壁の情報を出力
        for row in 0..Input::MAP_SIZE {
            let mut line = String::new();
            for col in 0..Input::MAP_SIZE - 1 {
                line.push(if self.walls_v[Coord::new(row, col)] {
                    '1'
                } else {
                    '0'
                });
            }
            println!("{}", line);
        }

        for row in 0..Input::MAP_SIZE - 1 {
            let mut line = String::new();
            for col in 0..Input::MAP_SIZE {
                line.push(if self.walls_h[Coord::new(row, col)] {
                    '1'
                } else {
                    '0'
                });
            }
            println!("{}", line);
        }

        // グループの情報を出力
        for (i, &group) in self.groups.iter().enumerate() {
            if i > 0 {
                print!(" ");
            }
            print!("{}", group);
        }
        println!();

        // アクションの情報を出力
        for action in &self.actions {
            match action {
                Action::Group(m) => {
                    let direction = match m.direction {
                        U => "U",
                        R => "R",
                        D => "D",
                        L => "L",
                        _ => panic!("Invalid direction"),
                    };
                    println!("g {} {}", m.index, direction);
                }
                Action::Robot(m) => {
                    let direction = match m.direction {
                        U => "U",
                        R => "R",
                        D => "D",
                        L => "L",
                        _ => panic!("Invalid direction"),
                    };
                    println!("i {} {}", m.index, direction);
                }
            }
        }
    }
}

pub enum Action {
    /// グループ全体を動かす
    Group(Move),
    /// 個別のロボットを動かす
    Robot(Move),
}

#[derive(Debug, Clone, Copy)]
pub struct Move {
    /// index of robot or group
    pub index: usize,
    /// U, R, D, L
    pub direction: usize,
}

impl Move {
    pub fn new(index: usize, direction: usize) -> Self {
        Self { index, direction }
    }
}
