use crate::grid::{Coord, Map2d, D, L, R, U};
use crate::problem::{Action, Input, Move, Output};
use std::collections::VecDeque;

pub fn solve_greedy(input: &Input, robot_order: &[usize]) -> Output {
    // 壁は追加しない
    let walls_v = input.init_walls_v.clone();
    let walls_h = input.init_walls_h.clone();

    // グラフを構築
    let graph = build_graph(input, &walls_v, &walls_h);

    // 現在のロボット位置
    let mut current_positions = input.init_robots.clone();
    let mut current_map = Map2d::with_default(Input::MAP_SIZE);

    for c in current_positions.iter() {
        current_map[*c] = true;
    }

    let mut actions = Vec::new();

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

            let path = find_path(
                current_positions[robot_id],
                input.destinations[robot_id],
                &graph,
                &current_map,
            );

            if let Some(path) = path {
                found = true;
                current_map[current_positions[robot_id]] = false; // 現在位置を空にする
                current_positions[robot_id] = input.destinations[robot_id]; // 目的地に移動
                current_map[current_positions[robot_id]] = true; // 新しい位置を占有

                for dir in path.iter() {
                    // アクションを追加
                    actions.push(Action::Robot(Move {
                        index: robot_id,
                        direction: *dir,
                    }));
                }
            }
        }

        if !found {
            // どのロボットも移動できなかった場合、終了
            break;
        }
    }

    // スコア計算
    let mut total_distance = 0;
    for i in 0..input.robot_count {
        let current_coord = current_positions[i];
        let dest_coord = input.destinations[i];
        total_distance += current_coord.dist(&dest_coord);
    }

    // 各ロボットを独立したグループにする
    let groups = (0..input.robot_count).collect();

    Output::new(walls_v, walls_h, groups, actions, total_distance as u32)
}

fn build_graph(
    _input: &Input,
    walls_v: &Map2d<bool>,
    walls_h: &Map2d<bool>,
) -> Map2d<[Option<Coord>; 4]> {
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

fn find_path(
    start: Coord,
    goal: Coord,
    graph: &Map2d<[Option<Coord>; 4]>,
    current_map: &Map2d<bool>,
) -> Option<Vec<usize>> {
    if current_map[goal] {
        return None;
    }

    let mut queue = VecDeque::new();
    let mut visited = Map2d::with_default(Input::MAP_SIZE);
    let mut parent = Map2d::with_default(Input::MAP_SIZE);

    queue.push_back(start);
    visited[start] = true;

    while let Some(current) = queue.pop_front() {
        if current == goal {
            break;
        }

        let current_coord = current;
        for direction in 0..4 {
            if let Some(next_pos) = graph[current_coord][direction] {
                if !visited[next_pos] && !current_map[next_pos] {
                    visited[next_pos] = true;
                    parent[next_pos] = (current, direction);
                    queue.push_back(next_pos);
                }
            }
        }
    }

    if !visited[goal] {
        return None; // Goal not reachable
    }

    // パスを復元
    let mut path = Vec::new();
    let mut current = goal;

    while current != start {
        let (prev_pos, direction) = parent[current];
        path.push(direction);
        current = prev_pos;
    }

    path.reverse();
    Some(path)
}
