use crate::grid::{Coord, CoordIndex, Map2d, D, L, R, U};
use crate::problem::{Action, Input, Move, Output};
use rand::prelude::*;
use std::collections::VecDeque;
use std::time::Instant;

pub(super) fn solve(input: &Input) -> Output {
    let start_time = Instant::now();
    let time_limit = std::time::Duration::from_millis(1900); // 2秒 - 100ms margin

    let mut best_output = None;
    let mut best_score = u32::MAX;
    let mut rng = rand::thread_rng();

    while start_time.elapsed() < time_limit {
        let output = solve_greedy(input, &mut rng);
        if output.score < best_score {
            best_score = output.score;
            best_output = Some(output);
        }
    }

    best_output.unwrap_or_else(|| {
        // Fallback: return empty solution
        let groups = (0..input.robot_count).collect();
        Output::new(
            input.init_walls_v.clone(),
            input.init_walls_h.clone(),
            groups,
            vec![],
            u32::MAX,
        )
    })
}

fn solve_greedy(input: &Input, rng: &mut impl Rng) -> Output {
    // 壁は追加しない
    let walls_v = input.init_walls_v.clone();
    let walls_h = input.init_walls_h.clone();

    // グラフを構築
    let graph = build_graph(input, &walls_v, &walls_h);

    // ロボットの順番をランダムに決める
    let mut robot_order: Vec<usize> = (0..input.robot_count).collect();
    robot_order.shuffle(rng);

    // 現在のロボット位置
    let mut current_positions = input.init_robots.clone();
    let mut actions = Vec::new();

    // 各ロボットを順番に移動
    for &robot_id in &robot_order {
        let path = find_path(
            current_positions[robot_id],
            input.destinations[robot_id],
            &graph,
            &current_positions,
            robot_id,
        );

        // パスに沿って移動
        for direction in path {
            let new_pos = move_robot(current_positions[robot_id], direction, &graph);
            if let Some(new_pos) = new_pos {
                // 移動先に他のロボットがいるかチェック
                if !current_positions.iter().any(|&pos| pos == new_pos) {
                    current_positions[robot_id] = new_pos;
                    actions.push(Action::Robot(Move {
                        index: robot_id,
                        direction,
                    }));
                }
            }
        }
    }

    // スコア計算
    let mut total_distance = 0;
    for i in 0..input.robot_count {
        let current_coord = current_positions[i].to_coord(Input::MAP_SIZE);
        let dest_coord = input.destinations[i].to_coord(Input::MAP_SIZE);
        total_distance += current_coord.dist(&dest_coord);
    }

    let score = actions.len() as u32 + 100 * total_distance as u32;

    // 各ロボットを独立したグループにする
    let groups = (0..input.robot_count).collect();

    Output::new(walls_v, walls_h, groups, actions, score)
}

fn build_graph(
    _input: &Input,
    walls_v: &Map2d<bool>,
    walls_h: &Map2d<bool>,
) -> Map2d<[Option<CoordIndex>; 4]> {
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
                    graph[c][dir] = Some(Coord::new(new_row, new_col).to_index(Input::MAP_SIZE));
                }
            }
        }
    }

    graph
}

fn find_path(
    start: CoordIndex,
    goal: CoordIndex,
    graph: &Map2d<[Option<CoordIndex>; 4]>,
    current_positions: &[CoordIndex],
    robot_id: usize,
) -> Vec<usize> {
    let mut queue = VecDeque::new();
    let mut visited = Map2d::with_default(Input::MAP_SIZE);
    let mut parent = Map2d::from_fn(|_| None, Input::MAP_SIZE);

    queue.push_back(start);
    visited[start] = true;

    while let Some(current) = queue.pop_front() {
        if current == goal {
            break;
        }

        let current_coord = current.to_coord(Input::MAP_SIZE);
        for direction in 0..4 {
            if let Some(next_pos) = graph[current_coord][direction] {
                if !visited[next_pos] {
                    // 他のロボットがいる場所は通れない
                    let occupied = current_positions
                        .iter()
                        .enumerate()
                        .any(|(id, &pos)| id != robot_id && pos == next_pos);

                    if !occupied {
                        visited[next_pos] = true;
                        parent[next_pos] = Some((current, direction));
                        queue.push_back(next_pos);
                    }
                }
            }
        }
    }

    // パスを復元
    let mut path = Vec::new();
    let mut current = goal;

    while let Some((prev_pos, direction)) = parent[current] {
        path.push(direction);
        current = prev_pos;
    }

    path.reverse();
    path
}

fn move_robot(
    pos: CoordIndex,
    direction: usize,
    graph: &Map2d<[Option<CoordIndex>; 4]>,
) -> Option<CoordIndex> {
    let coord = pos.to_coord(Input::MAP_SIZE);
    graph[coord][direction]
}
