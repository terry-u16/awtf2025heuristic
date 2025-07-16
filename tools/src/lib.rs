#![allow(non_snake_case, unused_macros)]

use itertools::Itertools;
use proconio::{input, marker::Chars};
use rand::prelude::*;
use std::ops::RangeBounds;
use svg::node::element::{Circle, Group, Line, Rectangle, Style, Text, Title};

pub trait SetMinMax {
    fn setmin(&mut self, v: Self) -> bool;
    fn setmax(&mut self, v: Self) -> bool;
}
impl<T> SetMinMax for T
where
    T: PartialOrd,
{
    fn setmin(&mut self, v: T) -> bool {
        *self > v && {
            *self = v;
            true
        }
    }
    fn setmax(&mut self, v: T) -> bool {
        *self < v && {
            *self = v;
            true
        }
    }
}

#[macro_export]
macro_rules! mat {
	($($e:expr),*) => { Vec::from(vec![$($e),*]) };
	($($e:expr,)*) => { Vec::from(vec![$($e),*]) };
	($e:expr; $d:expr) => { Vec::from(vec![$e; $d]) };
	($e:expr; $d:expr $(; $ds:expr)+) => { Vec::from(vec![mat![$e $(; $ds)*]; $d]) };
}

#[derive(Clone, Debug)]
pub struct Input {
    pub N: usize,
    pub K: usize,
    pub W: usize,
    pub ss: Vec<(usize, usize)>,
    pub ts: Vec<(usize, usize)>,
    pub wall_v: Vec<Vec<bool>>,
    pub wall_h: Vec<Vec<bool>>,
}

impl std::fmt::Display for Input {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{} {}", self.N, self.K)?;
        for i in 0..self.K {
            writeln!(
                f,
                "{} {} {} {}",
                self.ss[i].0, self.ss[i].1, self.ts[i].0, self.ts[i].1
            )?;
        }
        for i in 0..self.N {
            writeln!(
                f,
                "{}",
                self.wall_v[i]
                    .iter()
                    .map(|&b| if b { '1' } else { '0' })
                    .collect::<String>()
            )?;
        }
        for i in 0..self.N - 1 {
            writeln!(
                f,
                "{}",
                self.wall_h[i]
                    .iter()
                    .map(|&b| if b { '1' } else { '0' })
                    .collect::<String>()
            )?;
        }
        Ok(())
    }
}

pub fn parse_input(f: &str) -> Input {
    let f = proconio::source::once::OnceSource::from(f);
    input! {
        from f,
        N: usize, K: usize,
        st: [(usize, usize, usize, usize); K],
        wall_v: [Chars; N],
        wall_h: [Chars; N - 1],
    }
    let ss = st.iter().map(|&(x, y, _, _)| (x, y)).collect();
    let ts = st.iter().map(|&(_, _, x, y)| (x, y)).collect();
    let wall_v = wall_v
        .into_iter()
        .map(|s| s.into_iter().map(|c| c == '1').collect())
        .collect();
    let wall_h = wall_h
        .into_iter()
        .map(|s| s.into_iter().map(|c| c == '1').collect())
        .collect();
    Input {
        N,
        K,
        W: 0,
        ss,
        ts,
        wall_v,
        wall_h,
    }
}

pub fn read<T: Copy + PartialOrd + std::fmt::Display + std::str::FromStr, R: RangeBounds<T>>(
    token: Option<&str>,
    range: R,
) -> Result<T, String> {
    if let Some(v) = token {
        if let Ok(v) = v.parse::<T>() {
            if !range.contains(&v) {
                Err(format!("Out of range: {}", v))
            } else {
                Ok(v)
            }
        } else {
            Err(format!("Parse error: {}", v))
        }
    } else {
        Err("Unexpected EOF".to_owned())
    }
}

const DIJ: [(usize, usize); 4] = [(!0, 0), (1, 0), (0, !0), (0, 1)];
const DIR: [char; 4] = ['U', 'D', 'L', 'R'];

pub struct Output {
    pub wall_v: Vec<Vec<bool>>,
    pub wall_h: Vec<Vec<bool>>,
    pub group: Vec<usize>,
    pub out: Vec<(char, usize, usize)>,
}

pub fn parse_output(input: &Input, f: &str) -> Result<Output, String> {
    let mut f = f.split_whitespace().peekable();
    let mut wall_v = input.wall_v.clone();
    let mut wall_h = input.wall_h.clone();
    for i in 0..input.N {
        let s = f.next().ok_or("Unexpected EOF")?.chars().collect_vec();
        if s.len() != input.N - 1 {
            return Err("Illegal output format for v".to_string());
        }
        for j in 0..input.N - 1 {
            if s[j] != '0' && s[j] != '1' {
                return Err(format!("Invalid character in v: {}", s[j]));
            }
            wall_v[i][j] |= s[j] == '1';
        }
    }
    for i in 0..input.N - 1 {
        let s = f.next().ok_or("Unexpected EOF")?.chars().collect_vec();
        if s.len() != input.N {
            return Err("Illegal output format for h".to_string());
        }
        for j in 0..input.N {
            if s[j] != '0' && s[j] != '1' {
                return Err(format!("Invalid character in h: {}", s[j]));
            }
            wall_h[i][j] |= s[j] == '1';
        }
    }
    let mut group = vec![];
    for _ in 0..input.K {
        group.push(read(f.next(), 0..input.K)?);
    }
    let mut out = vec![];
    while f.peek().is_some() {
        let a = read(f.next(), 'a'..='z')?;
        if a != 'g' && a != 'i' {
            return Err(format!("Invalid a[t] character: {}", a));
        }
        let b = read(f.next(), 0..input.K)?;
        let dir = read(f.next(), 'A'..='Z')?;
        let Some(d) = DIR.iter().position(|&c| c == dir) else {
            return Err(format!("Invalid direction: {}", dir));
        };
        out.push((a, b, d));
        if out.len() > input.K * input.N * input.N {
            return Err("Too many moves".to_string());
        }
    }
    Ok(Output {
        wall_v,
        wall_h,
        group,
        out,
    })
}

pub fn gen(seed: u64, fix_K: Option<usize>, fix_W: Option<usize>) -> Input {
    let mut rng = rand_chacha::ChaCha20Rng::seed_from_u64(seed ^ 24);
    let N = 30;
    let mut K = rng.gen_range(10i32..=100) as usize;
    if let Some(fix_K) = fix_K {
        K = fix_K;
    }
    if K > N * N {
        panic!("K must be in the range [10, {}], but got {}", N * N, K);
    }
    let mut W = rng.gen_range(0i32..=2) as usize;
    if let Some(fix_W) = fix_W {
        W = fix_W;
    }
    if W > 2 {
        panic!("W must be in the range [0, 2], but got {}", W);
    }
    let mut ps = vec![];
    for i in 0..N {
        for j in 0..N {
            ps.push((i, j));
        }
    }
    ps.shuffle(&mut rng);
    let ss = ps[..K].to_vec();
    ps.shuffle(&mut rng);
    let ts = ps[..K].to_vec();
    let mut wall_v;
    let mut wall_h;
    loop {
        wall_v = mat![false; N; N - 1];
        wall_h = mat![false; N - 1; N];
        let mut used_v = vec![false; N - 1];
        let mut used_h = vec![false; N - 1];
        for _ in 0..W {
            loop {
                let dir = rng.gen_range(0i32..4) as usize;
                let l = rng.gen_range(10i32..=20) as usize;
                if dir / 2 == 0 {
                    // vertical
                    let i = rng.gen_range(5..=N as i32 - 5) as usize;
                    let j = rng.gen_range(4..=N as i32 - 6) as usize;
                    if used_v[j] {
                        continue;
                    }
                    for k in 0..9 {
                        if j - 4 + k < N - 1 {
                            used_v[j - 4 + k] = true;
                        }
                    }
                    for k in 0..l {
                        let i = i + DIJ[dir].0 * k;
                        if i < N {
                            wall_v[i][j] = true;
                        }
                    }
                } else {
                    // horizontal
                    let i = rng.gen_range(4..=N as i32 - 6) as usize;
                    let j = rng.gen_range(5..=N as i32 - 5) as usize;
                    if used_h[i] {
                        continue;
                    }
                    for k in 0..9 {
                        if i - 4 + k < N - 1 {
                            used_h[i - 4 + k] = true;
                        }
                    }
                    for k in 0..l {
                        let j = j + DIJ[dir].1 * k;
                        if j < N {
                            wall_h[i][j] = true;
                        }
                    }
                }
                break;
            }
        }
        let mut visited = mat![false; N; N];
        let mut stack = vec![(0, 0)];
        visited[0][0] = true;
        let mut num = 0;
        while let Some((i, j)) = stack.pop() {
            num += 1;
            for d in 0..4 {
                let (di, dj) = DIJ[d];
                let (i2, j2) = (i + di, j + dj);
                if i2 >= N || j2 >= N || visited[i2][j2] {
                    continue;
                }
                if di == 0 {
                    if wall_v[i][j.min(j2)] {
                        continue;
                    }
                } else {
                    if wall_h[i.min(i2)][j] {
                        continue;
                    }
                }
                visited[i2][j2] = true;
                stack.push((i2, j2));
            }
        }
        if num == N * N {
            break;
        }
    }
    Input {
        N,
        K,
        W,
        ss,
        ts,
        wall_v,
        wall_h,
    }
}

pub fn compute_score(input: &Input, out: &Output) -> (i64, String) {
    let (mut score, err, _) = compute_score_details(input, &out, out.out.len());
    if err.len() > 0 {
        score = 0;
    }
    (score, err)
}

pub fn compute_score_details(
    input: &Input,
    out: &Output,
    t: usize,
) -> (i64, String, Vec<(usize, usize)>) {
    let mut pos = input.ss.clone();
    let mut used = mat![false; input.N; input.N];
    for &(x, y) in &pos {
        used[x][y] = true;
    }
    let mut gs = vec![vec![]; input.K];
    for i in 0..input.K {
        gs[out.group[i]].push(i);
    }
    for &(a, b, d) in &out.out[..t] {
        let mut rs = if a == 'g' {
            gs[b].clone()
        } else {
            vec![b as usize]
        };
        let (di, dj) = DIJ[d];
        rs.sort_by_key(|&i| {
            let (x, y) = pos[i];
            -(x as i32 * di as i32 + y as i32 * dj as i32)
        });
        for r in rs {
            let (x, y) = pos[r];
            let (x2, y2) = (x + di, y + dj);
            if x2 >= input.N || y2 >= input.N || used[x2][y2] {
                continue;
            }
            if di == 0 {
                if out.wall_v[x][y.min(y2)] {
                    continue;
                }
            } else {
                if out.wall_h[x.min(x2)][y] {
                    continue;
                }
            }
            pos[r] = (x2, y2);
            used[x][y] = false;
            used[x2][y2] = true;
        }
    }
    let mut score = t as i64;
    for k in 0..input.K {
        score += pos[k].0.abs_diff(input.ts[k].0) as i64 * 100;
        score += pos[k].1.abs_diff(input.ts[k].1) as i64 * 100;
    }
    (score, String::new(), pos)
}

/// 0 <= val <= 1
pub fn color(mut val: f64) -> String {
    val.setmin(1.0);
    val.setmax(0.0);
    let (r, g, b) = if val < 0.5 {
        let x = val * 2.0;
        (
            30. * (1.0 - x) + 144. * x,
            144. * (1.0 - x) + 255. * x,
            255. * (1.0 - x) + 30. * x,
        )
    } else {
        let x = val * 2.0 - 1.0;
        (
            144. * (1.0 - x) + 255. * x,
            255. * (1.0 - x) + 30. * x,
            30. * (1.0 - x) + 70. * x,
        )
    };
    format!(
        "#{:02x}{:02x}{:02x}",
        r.round() as i32,
        g.round() as i32,
        b.round() as i32
    )
}

pub fn rect(x: usize, y: usize, w: usize, h: usize, fill: &str) -> Rectangle {
    Rectangle::new()
        .set("x", x)
        .set("y", y)
        .set("width", w)
        .set("height", h)
        .set("fill", fill)
}

pub fn group(title: String) -> Group {
    Group::new().add(Title::new(title))
}

pub fn vis_default(input: &Input, out: &Output) -> (i64, String, String) {
    let (mut score, err, svg) = vis(input, &out, out.out.len(), false, false);
    if err.len() > 0 {
        score = 0;
    }
    (score, err, svg)
}

pub fn vis(
    input: &Input,
    out: &Output,
    t: usize,
    show_group: bool,
    show_number: bool,
) -> (i64, String, String) {
    let D = 600 / input.N;
    let W = D * input.N;
    let H = D * input.N;
    let (score, err, pos) = compute_score_details(input, &out, t);
    let mut doc = svg::Document::new()
        .set("id", "vis")
        .set("viewBox", (-5, -5, W + 10, H + 10))
        .set("width", W + 10)
        .set("height", H + 10)
        .set("style", "background-color:white");
    doc = doc.add(Style::new(format!(
        "text {{text-anchor: middle;dominant-baseline: central;}}"
    )));
    for i in 0..input.N {
        for j in 0..input.N {
            let group = group(format!("({}, {})", i, j)).add(rect(j * D, i * D, D, D, "white"));
            doc = doc.add(group);
        }
    }
    for i in 0..=input.N {
        doc = doc.add(
            Line::new()
                .set("x1", 0)
                .set("y1", i * D)
                .set("x2", input.N * D)
                .set("y2", i * D)
                .set("stroke", "lightgray")
                .set("stroke-width", 1),
        );
        doc = doc.add(
            Line::new()
                .set("x1", i * D)
                .set("y1", 0)
                .set("x2", i * D)
                .set("y2", input.N * D)
                .set("stroke", "lightgray")
                .set("stroke-width", 1),
        );
    }
    for i in 0..input.N {
        for j in 0..input.N {
            if j + 1 < input.N {
                if out.wall_v[i][j] {
                    doc = doc.add(
                        Line::new()
                            .set("x1", j * D + D)
                            .set("y1", i * D)
                            .set("x2", j * D + D)
                            .set("y2", (i + 1) * D)
                            .set("stroke", if input.wall_v[i][j] { "black" } else { "gray" })
                            .set("stroke-width", if input.wall_v[i][j] { 4 } else { 3 })
                            .set("stroke-linecap", "round"),
                    );
                }
            }
            if i + 1 < input.N {
                if out.wall_h[i][j] {
                    doc = doc.add(
                        Line::new()
                            .set("x1", j * D)
                            .set("y1", i * D + D)
                            .set("x2", (j + 1) * D)
                            .set("y2", i * D + D)
                            .set("stroke", if input.wall_h[i][j] { "black" } else { "gray" })
                            .set("stroke-width", if input.wall_h[i][j] { 4 } else { 3 })
                            .set("stroke-linecap", "round"),
                    );
                }
            }
        }
    }
    let mut used_g = vec![];
    for i in 0..input.K {
        used_g.push(out.group[i]);
    }
    used_g.sort();
    used_g.dedup();
    for i in 0..input.K {
        let (x1, y1) = pos[i];
        let (x2, y2) = input.ts[i];
        let g = out.group[i];
        let mut group = group(format!(
            "robot {}\ngroup: {}\n({}, {}) → ({}, {})",
            i, g, x1, y1, x2, y2
        ));
        let c = if show_group {
            used_g.iter().position(|&g2| g2 == g).unwrap() as f64 / used_g.len() as f64
        } else {
            i as f64 / input.K as f64
        };
        if show_number {
            group = group.add(
                Text::new(format!("{}", if show_group { g } else { i }))
                    .set("x", y1 * D + D / 2)
                    .set("y", x1 * D + D / 2)
                    .set("fill", "black")
                    .set("font-size", D * 2 / 3),
            );
        } else {
            group = group.add(
                Circle::new()
                    .set("cx", y1 * D + D / 2)
                    .set("cy", x1 * D + D / 2)
                    .set("r", D as f64 / 3.0)
                    .set("fill", color(c)),
            );
        }
        doc = doc.add(group);

        doc = doc.add(
            Line::new()
                .set("x1", y1 * D + D / 2)
                .set("y1", x1 * D + D / 2)
                .set("x2", y2 * D + D / 2)
                .set("y2", x2 * D + D / 2)
                .set("stroke", format!("{}a0", color(c)))
                .set("stroke-width", 1),
        );
    }
    (score, err, doc.to_string())
}
