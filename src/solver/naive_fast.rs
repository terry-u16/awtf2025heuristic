use crate::data_structures::{FastClearArray, Queue};
use crate::grid::{Coord, CoordIndex, Map2d, D, L, R, U};
use crate::problem::{Action, Input, Move, Output};
use rand::prelude::*;
use std::collections::VecDeque;
use std::time::Instant;

pub fn solve_greedy(input: &Input, robot_order: &[usize]) -> (u32, u32) {
    // グラフを構築
    let graph = build_graph(input);

    // 現在のロボット位置
    let mut current_positions = input.init_robots.clone();
    let mut current_map = Map2d::with_default(Input::MAP_SIZE);
    let mut queue = Queue::new();
    let mut fast_clear_map = FastClearArray::new(Input::MAP_SIZE * Input::MAP_SIZE);

    for c in current_positions.iter() {
        current_map[*c] = true;
    }

    let mut total_dist = 0;

    // 各ロボットを順番に移動
    while current_positions
        .iter()
        .zip(input.destinations.iter())
        .any(|(pos, dest)| pos != dest)
    {
        let mut found = false;

        for &robot_id in robot_order {
            if current_positions[robot_id] == input.destinations[robot_id] {
                continue; // 目的地に到達しているロボットはスキップ
            }

            if let Some(path) = find_dist(
                current_positions[robot_id].to_index(Input::MAP_SIZE),
                input.destinations[robot_id].to_index(Input::MAP_SIZE),
                &graph,
                &current_map,
                &mut queue,
                &mut fast_clear_map,
            ) {
                found = true;
                current_map[current_positions[robot_id]] = false; // 現在位置を空にする
                current_positions[robot_id] = input.destinations[robot_id]; // 目的地に移動
                current_map[current_positions[robot_id]] = true; // 新しい位置を占有
                total_dist += path as u32;
            }
        }

        if !found {
            // どのロボットも移動できなかった場合、終了
            break;
        }
    }

    // スコア計算
    let mut remaining_dist = 0;
    for i in 0..input.robot_count {
        let current_coord = current_positions[i];
        let dest_coord = input.destinations[i];
        remaining_dist += current_coord.dist(&dest_coord) as u32;
    }

    (total_dist, remaining_dist)
}

fn build_graph(input: &Input) -> Map2d<[Option<CoordIndex>; 4]> {
    let mut graph = Map2d::from_fn(|_| [None; 4], Input::MAP_SIZE);
    let walls_v = &input.init_walls_v;
    let walls_h = &input.init_walls_h;

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

fn find_dist(
    start: CoordIndex,
    goal: CoordIndex,
    graph: &Map2d<[Option<CoordIndex>; 4]>,
    current_map: &Map2d<bool>,
    queue: &mut Queue<(CoordIndex, u32)>,
    visited: &mut FastClearArray,
) -> Option<u32> {
    if current_map[goal] {
        return None;
    }

    queue.clear();
    visited.clear();

    queue.push((start, 0));
    visited.set_true(start.0);

    while let Some(&(pos, dist)) = queue.pop() {
        for direction in 0..4 {
            if let Some(next_pos) = graph[pos][direction] {
                if !visited.get(next_pos.0) && !current_map[next_pos] {
                    if next_pos == goal {
                        return Some(dist + 1);
                    }

                    visited.set_true(next_pos.0);
                    queue.push((next_pos, dist + 1));
                }
            }
        }
    }

    None
}

fn move_robot(pos: Coord, direction: usize, graph: &Map2d<[Option<Coord>; 4]>) -> Option<Coord> {
    graph[pos][direction]
}
