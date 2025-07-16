use crate::data_structures::{FastClearArray, Queue};
use crate::grid::{Coord, CoordIndex, Map2d, D, L, R, U};
use crate::problem::{Action, Input, Move, Output};
use rand::prelude::*;
use std::collections::VecDeque;
use std::time::Instant;

pub fn solve_greedy(input: &Input, robot_order: &[usize]) -> (u32, u32) {
    // 現在のロボット位置
    let mut current_positions = input.init_robots.clone();

    // bit演算用の占有状態マスク
    let mut occupied_bits = [0b1000_0000_0000_0000_0000_0000_0000_0001; Input::MAP_SIZE + 2];

    for c in current_positions.iter() {
        let coord = c;
        occupied_bits[coord.row() + 1] |= 1u32 << (coord.col() + 1);
    }

    occupied_bits[0] = !0; // 上端
    occupied_bits[Input::MAP_SIZE + 1] = !0; // 下端

    let mut wall_bits_v = [0u32; Input::MAP_SIZE + 2];
    let mut wall_bits_h = [0u32; Input::MAP_SIZE + 2];

    for row in 0..Input::MAP_SIZE {
        for col in 0..Input::MAP_SIZE {
            if input.init_walls_h[row][col] {
                wall_bits_h[row + 1] |= 1u32 << (col + 1);
            }
        }
    }

    //wall_bits_h[0] = !0; // 上端の壁
    //wall_bits_h[Input::MAP_SIZE + 1] = !0; // 下端の壁

    for row in 0..Input::MAP_SIZE {
        for col in 0..Input::MAP_SIZE {
            if input.init_walls_v[row][col] {
                wall_bits_v[row + 1] |= 1u32 << (col + 1);
            }
        }

        //wall_bits_v[row + 1] |= 1u32; // 左端の壁
        //wall_bits_v[row + 1] |= 1u32 << (Input::MAP_SIZE + 1); // 右端の壁
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

            if let Some(dist) = find_dist(
                current_positions[robot_id],
                input.destinations[robot_id],
                &occupied_bits,
                &wall_bits_v,
                &wall_bits_h,
            ) {
                found = true;

                // 現在位置のビットをクリア
                let old_coord = current_positions[robot_id];
                occupied_bits[old_coord.row() + 1] &= !(1u32 << (old_coord.col() + 1));

                // 新しい位置にロボットを配置
                current_positions[robot_id] = input.destinations[robot_id];
                let new_coord = current_positions[robot_id];
                occupied_bits[new_coord.row() + 1] |= 1u32 << (new_coord.col() + 1);

                total_dist += dist as u32;
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

fn find_dist(
    start: Coord,
    goal: Coord,
    occupied_bits: &[u32; Input::MAP_SIZE + 2],
    wall_bits_v: &[u32; Input::MAP_SIZE + 2],
    wall_bits_h: &[u32; Input::MAP_SIZE + 2],
) -> Option<u32> {
    if occupied_bits[goal.row() + 1] & (1u32 << (goal.col() + 1)) != 0 {
        return None;
    }

    // bit演算用の訪問済みマスク（各行を32bitで表現）
    let mut visited = [0u32; Input::MAP_SIZE + 2];
    visited[start.row() + 1] |= 1u32 << (start.col() + 1);

    for dist in 1.. {
        let mut visited_updated = visited.clone();
        let mut new_visited = 0;

        // 上方向
        for row in 1..=Input::MAP_SIZE {
            let bit = visited[row] & !wall_bits_h[row - 1] & !occupied_bits[row - 1];
            new_visited |= bit & !visited[row - 1];
            visited_updated[row - 1] |= bit;
        }

        // 下方向
        for row in 1..=Input::MAP_SIZE {
            let bit = visited[row] & !wall_bits_h[row] & !occupied_bits[row + 1];
            new_visited |= bit & !visited[row + 1];
            visited_updated[row + 1] |= bit;
        }

        // 左方向
        for row in 1..=Input::MAP_SIZE {
            let bit = (visited[row] << 1) & !wall_bits_v[row] & !occupied_bits[row];
            new_visited |= bit & !visited[row];
            visited_updated[row] |= bit;
        }

        // 右方向
        for row in 1..=Input::MAP_SIZE {
            let bit = ((visited[row] & !wall_bits_v[row]) >> 1) & !occupied_bits[row];
            new_visited |= bit & !visited[row];
            visited_updated[row] |= bit;
        }

        visited = visited_updated;

        for r in visited.iter() {
            eprintln!("{:032b}", r);
        }

        eprintln!();

        if new_visited == 0 {
            return None;
        } else if visited[goal.row() + 1] & (1u32 << (goal.col() + 1)) != 0 {
            return Some(dist);
        }
    }

    unreachable!() // Should not reach here
}
