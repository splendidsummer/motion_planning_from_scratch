"""二维点机器人上的 RRT、RRTConnect、PRM、RRT*。

数学推导见同目录 planning_from_scratch.md。符号与笔记一致：

    q = (x, y)           构型，这里就是平面上的点
    η                    单步最大长度（steer 的步长）
    C_free               不与轴对齐矩形障碍相交的位置
    Cost(q)              从起点沿树走到 q 的路径长度

运行：

    python planners_2d.py
    python planners_2d.py rrt
    python planners_2d.py rrt_connect
    python planners_2d.py prm
    python planners_2d.py search
    python planners_2d.py rrt_star
    python planners_2d.py ompl
"""

from __future__ import annotations

import math
import heapq
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT_DIR = Path(__file__).resolve().parent
WORLD_SIZE = 10.0
START = (1.0, 1.0)
GOAL = (9.0, 9.0)
# 第二次 PRM 查询：路网不重建，只把新的起终点接上去。
START_2 = (1.0, 9.0)
GOAL_2 = (9.0, 1.0)
ETA = 0.5


@dataclass(frozen=True)
class Rect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def area(self) -> float:
        return max(0.0, self.xmax - self.xmin) * max(0.0, self.ymax - self.ymin)


# 两面错开的墙，直线会撞上，但上、下都留了缺口。
OBSTACLES = (
    Rect(3.5, 0.0, 4.5, 7.0),
    Rect(6.0, 3.0, 7.0, 10.0),
)


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_in_rect(q, rect: Rect) -> bool:
    return rect.xmin <= q[0] <= rect.xmax and rect.ymin <= q[1] <= rect.ymax


def segment_hits_rect(a, b, rect: Rect) -> bool:
    """线段 q(t) = a + t (b - a), t ∈ [0, 1] 是否与闭矩形相交。

    把「点落在矩形内」写成四个线性不等式 p_i t ≤ q_i，再用 Liang–Barsky
    求 t 的可行区间。区间非空则相交。推导见笔记第 2 节。
    """
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    p = (-dx, dx, -dy, dy)
    q = (a[0] - rect.xmin, rect.xmax - a[0], a[1] - rect.ymin, rect.ymax - a[1])
    t0, t1 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if pi == 0.0:
            if qi < 0.0:
                return False
            continue
        t = qi / pi
        if pi < 0.0:
            if t > t1:
                return False
            if t > t0:
                t0 = t
        else:
            if t < t0:
                return False
            if t < t1:
                t1 = t
    return t0 <= t1


def collides_point(q) -> bool:
    if not (0.0 <= q[0] <= WORLD_SIZE and 0.0 <= q[1] <= WORLD_SIZE):
        return True
    return any(point_in_rect(q, rect) for rect in OBSTACLES)


def collides_segment(a, b) -> bool:
    if collides_point(a) or collides_point(b):
        return True
    return any(segment_hits_rect(a, b, rect) for rect in OBSTACLES)


def free_area() -> float:
    """μ(C_free)。两块障碍都在 [0, 10]² 内且互不相交。"""
    return WORLD_SIZE * WORLD_SIZE - sum(rect.area() for rect in OBSTACLES)


def sample_uniform(rng: np.random.Generator):
    """在整个构型空间里均匀采样，障碍内部也允许。

    原始 RRT 就是这样：采样点只提供方向，真正加入树的是 steer 之后、
    并且落在 C_free 里的那个点。
    """
    return (float(rng.uniform(0.0, WORLD_SIZE)), float(rng.uniform(0.0, WORLD_SIZE)))


def sample_free(rng: np.random.Generator, limit: int = 10000):
    """拒绝采样：只收下 C_free 中的点。PRM 建路网时用这个。"""
    for _ in range(limit):
        q = sample_uniform(rng)
        if not collides_point(q):
            return q
    raise RuntimeError("拒绝采样失败：自由空间比例太低，或迭代上限太小。")


def steer(q_near, q_rand, eta: float = ETA):
    """把 q_near 朝 q_rand 移动，移动距离不超过 η。"""
    d = dist(q_near, q_rand)
    if d <= eta or d == 0.0:
        return q_rand
    scale = eta / d
    return (
        q_near[0] + scale * (q_rand[0] - q_near[0]),
        q_near[1] + scale * (q_rand[1] - q_near[1]),
    )


def nearest(nodes, q):
    best_i = 0
    best_d = dist(nodes[0], q)
    for i in range(1, len(nodes)):
        d = dist(nodes[i], q)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def rrt_star_radius(n: int, eta: float = ETA) -> float:
    """Karaman–Frazzoli 的连接半径，并被步长 η 截断。

    γ > 2 (1 + 1/d)^{1/d} (μ(C_free) / ζ_d)^{1/d}
    r(n) = min( γ (log n / n)^{1/d} , η )
    d = 2 时 ζ_2 = π。证明只保证「γ 要大于这个阈值」，这里取 1.1 倍。
    """
    dim = 2
    mu = free_area()
    zeta = math.pi
    gamma_star = 2.0 * (1.0 + 1.0 / dim) ** (1.0 / dim) * (mu / zeta) ** (1.0 / dim)
    gamma = 1.1 * gamma_star
    n = max(int(n), 2)
    radius = gamma * math.sqrt(math.log(n) / n)
    return min(radius, eta)


def rrt(rng, max_iter: int = 8000, eta: float = ETA, goal_bias: float = 0.0):
    """找到一条可行路径就停。返回 (nodes, parents, path, iterations)。"""
    nodes = [START]
    parents = [-1]
    for it in range(1, max_iter + 1):
        if goal_bias > 0.0 and rng.random() < goal_bias:
            q_rand = GOAL
        else:
            q_rand = sample_uniform(rng)
        q_near_i = nearest(nodes, q_rand)
        q_new = steer(nodes[q_near_i], q_rand, eta)
        if collides_segment(nodes[q_near_i], q_new):
            continue
        nodes.append(q_new)
        parents.append(q_near_i)
        if dist(q_new, GOAL) <= 1e-12:
            return nodes, parents, trace_path(parents, len(nodes) - 1, nodes), it
        if dist(q_new, GOAL) <= eta and not collides_segment(q_new, GOAL):
            nodes.append(GOAL)
            parents.append(len(nodes) - 2)
            return nodes, parents, trace_path(parents, len(nodes) - 1, nodes), it
    return nodes, parents, None, max_iter


def trace_path(parents, index, nodes):
    path = []
    while index != -1:
        path.append(nodes[index])
        index = parents[index]
    path.reverse()
    return path


def path_length(path) -> float:
    if not path or len(path) < 2:
        return math.inf
    return sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))


class Tree:
    def __init__(self, root):
        self.nodes = [root]
        self.parents = [-1]

    def extend(self, q_rand, eta: float = ETA):
        """EXTEND：Trapped / Advanced / Reached。Reached 表示 q_new 就是 q_rand。"""
        q_near_i = nearest(self.nodes, q_rand)
        q_new = steer(self.nodes[q_near_i], q_rand, eta)
        if collides_segment(self.nodes[q_near_i], q_new):
            return "Trapped", None
        self.nodes.append(q_new)
        self.parents.append(q_near_i)
        new_i = len(self.nodes) - 1
        if dist(q_new, q_rand) <= 1e-12:
            return "Reached", new_i
        return "Advanced", new_i

    def connect(self, q_target, eta: float = ETA):
        """CONNECT：一直朝 q_target 走，直到到达或被挡住。"""
        status = "Advanced"
        last = None
        while status == "Advanced":
            status, last = self.extend(q_target, eta)
        return status, last


def rrt_connect(rng, max_iter: int = 4000, eta: float = ETA):
    """两棵树轮流 EXTEND + CONNECT。返回两棵树和从起点到终点的路径。"""
    tree_a = Tree(START)
    tree_b = Tree(GOAL)
    a_is_start = True
    for it in range(1, max_iter + 1):
        status, new_i = tree_a.extend(sample_uniform(rng), eta)
        if status != "Trapped":
            connect_status, other_i = tree_b.connect(tree_a.nodes[new_i], eta)
            if connect_status == "Reached":
                path_a = trace_path(tree_a.parents, new_i, tree_a.nodes)
                path_b = trace_path(tree_b.parents, other_i, tree_b.nodes)
                if a_is_start:
                    path = path_a + path_b[-2::-1]
                else:
                    path = path_b + path_a[-2::-1]
                return tree_a, tree_b, path, it, a_is_start
        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start
    return tree_a, tree_b, None, max_iter, a_is_start


def prm_build(rng, n_samples: int = 500, k: int = 15):
    """只在 C_free 撒点，每个点连 k 个近邻。路网与起终点无关。"""
    nodes = [sample_free(rng) for _ in range(n_samples)]
    adj = [[] for _ in range(n_samples)]
    for i, q in enumerate(nodes):
        order = sorted(range(n_samples), key=lambda j: dist(q, nodes[j]))
        for j in order[1 : k + 1]:
            if any(v == j for v, _ in adj[i]):
                continue
            if collides_segment(q, nodes[j]):
                continue
            w = dist(q, nodes[j])
            adj[i].append((j, w))
            adj[j].append((i, w))
    return nodes, adj


def prm_attach(nodes, adj, q, k: int = 15):
    """把一个新点接到现成路网上，不重新采样。"""
    if collides_point(q):
        raise ValueError(f"查询点 {q} 在障碍里")
    i = len(nodes)
    nodes.append(q)
    adj.append([])
    order = sorted(range(i), key=lambda j: dist(q, nodes[j]))
    for j in order[:k]:
        if collides_segment(q, nodes[j]):
            continue
        w = dist(q, nodes[j])
        adj[i].append((j, w))
        adj[j].append((i, w))
    return i


def reconstruct(prev, start: int, goal: int):
    if goal not in prev:
        return None
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = prev[node]
    path.append(start)
    path.reverse()
    return path


def graph_search(adj, start: int, goal: int, h_of):
    """按 f(u) = g(u) + h(u) 扩展。h = 0 就是 Dijkstra，h 为到终点的距离就是 A*。

    边权必须非负。h 若可采纳且一致，终点第一次被取出时 g(goal) 就是最短路。
    返回 (path, length, expanded)。expanded 是取出的顺序，用来画搜索波前。
    """
    g_score = {start: 0.0}
    prev = {start: -1}
    counter = 0
    pq = [(h_of(start), 0.0, counter, start)]
    closed = set()
    expanded = []
    while pq:
        _, g_u, _, u = heapq.heappop(pq)
        if g_u > g_score[u] + 1e-9:
            continue
        if u in closed:
            continue
        closed.add(u)
        expanded.append(u)
        if u == goal:
            break
        for v, w in adj[u]:
            tentative = g_u + w
            if tentative < g_score.get(v, math.inf) - 1e-12:
                g_score[v] = tentative
                prev[v] = u
                counter += 1
                heapq.heappush(pq, (tentative + h_of(v), tentative, counter, v))
    if goal not in g_score or goal not in prev:
        return None, math.inf, expanded
    return reconstruct(prev, start, goal), g_score[goal], expanded


def dijkstra(adj, start: int, goal: int):
    """h = 0。边权是欧氏长度。返回 (path_indices, length)。"""
    path, length, _ = graph_search(adj, start, goal, lambda _u: 0.0)
    return path, length


def astar(adj, nodes, start: int, goal: int, weight: float = 1.0):
    """h(u) = weight * ||u - goal||。weight = 1 时可采纳；weight > 1 会少展开，但可能不是最短路。"""
    goal_q = nodes[goal]

    def h_of(u: int) -> float:
        return weight * dist(nodes[u], goal_q)

    return graph_search(adj, start, goal, h_of)


def grid_graph(resolution: float = 0.2):
    """把同一张地图切成八连通网格。对角步不允许从障碍角上切过去。"""
    n = int(round(WORLD_SIZE / resolution))
    free = {}
    centers = []
    for ix in range(n):
        for iy in range(n):
            q = ((ix + 0.5) * resolution, (iy + 0.5) * resolution)
            if collides_point(q):
                continue
            free[(ix, iy)] = len(centers)
            centers.append(q)
    adj = [[] for _ in centers]
    for (ix, iy), i in free.items():
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                j = free.get((ix + dx, iy + dy))
                if j is None or j < i:
                    continue
                if dx != 0 and dy != 0:
                    if (ix + dx, iy) not in free or (ix, iy + dy) not in free:
                        continue
                w = dist(centers[i], centers[j])
                adj[i].append((j, w))
                adj[j].append((i, w))
    return centers, adj, resolution


def rrt_star(rng, time_limit: float = 5.0, eta: float = ETA, goal_bias: float = 0.05):
    """在时间上限内一直优化。返回树、最优路径、代价随时间的记录、最终半径。"""
    nodes = [START]
    parents = [-1]
    cost = [0.0]
    children = [[]]
    goal_parent = None
    goal_cost = math.inf
    history = []
    t0 = time.perf_counter()

    def try_goal(i, radius):
        nonlocal goal_parent, goal_cost
        if dist(nodes[i], GOAL) <= radius and not collides_segment(nodes[i], GOAL):
            c = cost[i] + dist(nodes[i], GOAL)
            if c < goal_cost:
                goal_parent = i
                goal_cost = c

    def rewire_parent(child, new_parent):
        old = parents[child]
        if old != -1:
            children[old].remove(child)
        parents[child] = new_parent
        children[new_parent].append(child)
        cost[child] = cost[new_parent] + dist(nodes[new_parent], nodes[child])
        stack = [child]
        while stack:
            u = stack.pop()
            for v in children[u]:
                cost[v] = cost[u] + dist(nodes[u], nodes[v])
                stack.append(v)

    iterations = 0
    while time.perf_counter() - t0 < time_limit:
        iterations += 1
        if rng.random() < goal_bias:
            q_rand = GOAL
        else:
            q_rand = sample_uniform(rng)
        q_near_i = nearest(nodes, q_rand)
        q_new = steer(nodes[q_near_i], q_rand, eta)
        if collides_segment(nodes[q_near_i], q_new):
            continue

        radius = rrt_star_radius(len(nodes) + 1, eta)
        near = [
            i
            for i, q in enumerate(nodes)
            if dist(q, q_new) <= radius and not collides_segment(q, q_new)
        ]
        parent_i = q_near_i
        best = cost[q_near_i] + dist(nodes[q_near_i], q_new)
        for i in near:
            c = cost[i] + dist(nodes[i], q_new)
            if c < best:
                best = c
                parent_i = i

        new_i = len(nodes)
        nodes.append(q_new)
        parents.append(parent_i)
        cost.append(best)
        children.append([])
        children[parent_i].append(new_i)

        for i in near:
            if i == parent_i:
                continue
            c = cost[new_i] + dist(q_new, nodes[i])
            if c < cost[i] and not collides_segment(q_new, nodes[i]):
                rewire_parent(i, new_i)

        try_goal(new_i, radius)
        for i in near:
            try_goal(i, radius)
        if goal_parent is not None:
            goal_cost = cost[goal_parent] + dist(nodes[goal_parent], GOAL)
        elapsed = time.perf_counter() - t0
        history.append((elapsed, goal_cost if goal_parent is not None else math.inf))

    path = None
    if goal_parent is not None:
        path = trace_path(parents, goal_parent, nodes) + [GOAL]
    radius_final = rrt_star_radius(len(nodes), eta)
    return nodes, parents, path, history, iterations, radius_final


def draw_world(ax):
    ax.set_xlim(0.0, WORLD_SIZE)
    ax.set_ylim(0.0, WORLD_SIZE)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    for rect in OBSTACLES:
        ax.add_patch(
            plt.Rectangle(
                (rect.xmin, rect.ymin),
                rect.xmax - rect.xmin,
                rect.ymax - rect.ymin,
                facecolor="#333333",
                edgecolor="black",
            )
        )


def draw_tree(ax, nodes, parents, color="#9bb7d4", lw=0.6):
    for i, parent in enumerate(parents):
        if parent < 0:
            continue
        a, b = nodes[parent], nodes[i]
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, zorder=1)


def draw_path(ax, path, color="#d62728", lw=2.2):
    if not path:
        return
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, color=color, lw=lw, zorder=3)


def mark_query(ax, start, goal):
    ax.scatter([start[0]], [start[1]], c="#2ca02c", s=40, zorder=4, label="start")
    ax.scatter([goal[0]], [goal[1]], c="#ff7f0e", s=40, zorder=4, label="goal")


def save_fig(fig, name: str):
    path = OUT_DIR / name
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(f"saved {path}")


def demo_rrt(seed: int = 0):
    rng = np.random.default_rng(seed)
    nodes, parents, path, iters = rrt(rng)
    print(f"RRT  iterations={iters}  nodes={len(nodes)}  length={path_length(path):.3f}")
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_world(ax)
    draw_tree(ax, nodes, parents)
    draw_path(ax, path)
    mark_query(ax, START, GOAL)
    ax.set_title(f"RRT   length = {path_length(path):.2f}   iters = {iters}")
    ax.legend(loc="upper left")
    save_fig(fig, "rrt.png")
    return path


def demo_rrt_connect(seed: int = 1):
    rng = np.random.default_rng(seed)
    tree_a, tree_b, path, iters, _ = rrt_connect(rng)
    print(
        f"RRTConnect  iterations={iters}  "
        f"nodes={len(tree_a.nodes) + len(tree_b.nodes)}  length={path_length(path):.3f}"
    )
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_world(ax)
    draw_tree(ax, tree_a.nodes, tree_a.parents, color="#9bb7d4")
    draw_tree(ax, tree_b.nodes, tree_b.parents, color="#e0b0c8")
    draw_path(ax, path)
    mark_query(ax, START, GOAL)
    ax.set_title(f"RRTConnect   length = {path_length(path):.2f}   iters = {iters}")
    ax.legend(loc="upper left")
    save_fig(fig, "rrt_connect.png")
    return path


def demo_prm(seed: int = 2):
    rng = np.random.default_rng(seed)
    nodes, adj = prm_build(rng)
    s1 = prm_attach(nodes, adj, START)
    g1 = prm_attach(nodes, adj, GOAL)
    idx1, length1 = dijkstra(adj, s1, g1)
    path1 = None if idx1 is None else [nodes[i] for i in idx1]
    print(f"PRM query 1  samples={len(nodes)}  length={length1:.3f}")

    s2 = prm_attach(nodes, adj, START_2)
    g2 = prm_attach(nodes, adj, GOAL_2)
    idx2, length2 = dijkstra(adj, s2, g2)
    path2 = None if idx2 is None else [nodes[i] for i in idx2]
    print(f"PRM query 2  samples={len(nodes)}  length={length2:.3f}  (roadmap reused)")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    for ax, path, start, goal, title in (
        (axes[0], path1, START, GOAL, f"PRM query 1   length = {length1:.2f}"),
        (axes[1], path2, START_2, GOAL_2, f"PRM query 2   length = {length2:.2f}"),
    ):
        draw_world(ax)
        for i, nbrs in enumerate(adj):
            for j, _ in nbrs:
                if j < i:
                    continue
                a, b = nodes[i], nodes[j]
                ax.plot([a[0], b[0]], [a[1], b[1]], color="#c5c5c5", lw=0.4, zorder=1)
        draw_path(ax, path)
        mark_query(ax, start, goal)
        ax.set_title(title)
        ax.legend(loc="upper left")
    save_fig(fig, "prm.png")
    return path1, path2


def cost_at(history, t_query: float):
    best = math.inf
    for t, c in history:
        if t > t_query:
            break
        best = c
    return best


def demo_rrt_star(seed: int = 3, time_limit: float = 5.0):
    rng = np.random.default_rng(seed)
    nodes, parents, path, history, iters, radius = rrt_star(rng, time_limit=time_limit)
    c01 = cost_at(history, 0.1)
    c_final = path_length(path)
    first = next(((t, c) for t, c in history if c < math.inf), None)
    first_txt = "none" if first is None else f"{first[1]:.3f} at {first[0]:.3f}s"
    print(
        f"RRT*  iters={iters}  nodes={len(nodes)}  radius={radius:.3f}  "
        f"length@0.1s={c01:.3f}  first={first_txt}  "
        f"length@{time_limit:.1f}s={c_final:.3f}"
    )
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_world(ax)
    draw_tree(ax, nodes, parents)
    draw_path(ax, path)
    mark_query(ax, START, GOAL)
    ax.set_title(f"RRT*   length = {c_final:.2f}   after {time_limit:.1f}s")
    ax.legend(loc="upper left")
    save_fig(fig, "rrt_star.png")

    fig, ax = plt.subplots(figsize=(6, 4))
    ts, cs = [], []
    best = math.inf
    for t, c in history:
        if c < best:
            best = c
            ts.append(t)
            cs.append(c)
    if ts:
        ax.plot(ts, cs, color="#1f77b4")
        ax.scatter([ts[0]], [cs[0]], c="#d62728", zorder=3, label="first feasible")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("best path length")
    ax.set_title("RRT* keeps using the time budget")
    ax.legend(loc="upper right")
    save_fig(fig, "rrt_star_cost.png")
    return path, c01, c_final


def demo_ompl(seed: int = 4, time_limit: float = 1.0):
    """同一张地图上调用 OMPL 的 RRTConnect 和 RRT*。没安装 ompl 就跳过。"""
    try:
        from ompl import base as ob
        from ompl import geometric as og
    except ImportError:
        print("OMPL Python 绑定未安装，跳过这一节。安装后重跑：python planners_2d.py ompl")
        return None

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(WORLD_SIZE)
    space.setBounds(bounds)

    def is_valid(state) -> bool:
        return not collides_point((state[0], state[1]))

    def solve(planner_cls, name: str):
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(is_valid))
        si.setStateValidityCheckingResolution(0.005)
        si.setup()
        start = ob.State(space)
        goal = ob.State(space)
        start[0], start[1] = START
        goal[0], goal[1] = GOAL
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)
        planner = planner_cls(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        if planner_cls is og.RRTstar:
            planner.setRange(ETA)
        elif hasattr(planner, "setRange"):
            planner.setRange(ETA)
        solved = planner.solve(time_limit)
        if not solved:
            print(f"OMPL {name} 在 {time_limit}s 内没有找到路径")
            return None
        path = pdef.getSolutionPath()
        pts = []
        for i in range(path.getStateCount()):
            s = path.getState(i)
            pts.append((s[0], s[1]))
        print(f"OMPL {name}  length={path_length(pts):.3f}  states={len(pts)}")
        return pts

    # OMPL 的 RNG 与 numpy 无关；setSeed 只影响这次进程里的 OMPL 采样。
    ob.RNG.setSeed(seed)
    path_connect = solve(og.RRTConnect, "RRTConnect")
    ob.RNG.setSeed(seed)
    path_star = solve(og.RRTstar, "RRTstar")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    for ax, path, title in (
        (axes[0], path_connect, "OMPL RRTConnect"),
        (axes[1], path_star, "OMPL RRT*"),
    ):
        draw_world(ax)
        draw_path(ax, path)
        mark_query(ax, START, GOAL)
        if path:
            ax.set_title(f"{title}   length = {path_length(path):.2f}")
        else:
            ax.set_title(f"{title}   failed")
        ax.legend(loc="upper left")
    save_fig(fig, "ompl.png")
    return path_connect, path_star


def nearest_index(nodes, q):
    return min(range(len(nodes)), key=lambda i: dist(nodes[i], q))


def draw_expanded(ax, nodes, expanded, resolution=None):
    """按取出顺序涂色。没被取出的点留白。"""
    order = {node: i for i, node in enumerate(expanded)}
    cmap = plt.get_cmap("magma_r")
    denom = max(len(expanded) - 1, 1)
    if resolution is None:
        xs, ys, colors = [], [], []
        for i, q in enumerate(nodes):
            if i not in order:
                xs.append(q[0])
                ys.append(q[1])
                colors.append((0.85, 0.85, 0.85, 1.0))
        ax.scatter(xs, ys, c=colors, s=8, zorder=2)
        xs, ys, colors = [], [], []
        for i, q in enumerate(nodes):
            if i in order:
                xs.append(q[0])
                ys.append(q[1])
                colors.append(cmap(order[i] / denom))
        ax.scatter(xs, ys, c=colors, s=14, zorder=2)
        return
    for i, q in enumerate(nodes):
        color = cmap(order[i] / denom) if i in order else (0.93, 0.93, 0.93, 1.0)
        ax.add_patch(
            plt.Rectangle(
                (q[0] - resolution / 2, q[1] - resolution / 2),
                resolution,
                resolution,
                facecolor=color,
                edgecolor="none",
                zorder=0,
            )
        )


def demo_search(seed: int = 2):
    """同一张图上对比 Dijkstra 和 A*：最短路长度相同，A* 取出的点更少。"""
    centers, grid_adj, resolution = grid_graph()
    grid_start = nearest_index(centers, START)
    grid_goal = nearest_index(centers, GOAL)
    grid_results = []
    for name, weight in (("Dijkstra  h = 0", 0.0), ("A*  h = distance", 1.0)):
        if weight == 0.0:
            path_i, length, expanded = graph_search(
                grid_adj, grid_start, grid_goal, lambda _u: 0.0
            )
        else:
            path_i, length, expanded = astar(grid_adj, centers, grid_start, grid_goal, weight)
        grid_results.append((name, path_i, length, expanded))
        print(
            f"grid {name}  expanded={len(expanded)}/{len(centers)}  "
            f"length={length:.3f}"
        )

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    for ax, (name, path_i, length, expanded) in zip(axes, grid_results):
        draw_expanded(ax, centers, expanded, resolution)
        draw_world(ax)
        draw_path(ax, None if path_i is None else [centers[i] for i in path_i])
        mark_query(ax, centers[grid_start], centers[grid_goal])
        ax.set_title(f"{name}\nlength = {length:.2f}   expanded = {len(expanded)}")
        ax.legend(loc="upper left")
    save_fig(fig, "search_grid.png")

    rng = np.random.default_rng(seed)
    nodes, adj = prm_build(rng)
    s1 = prm_attach(nodes, adj, START)
    g1 = prm_attach(nodes, adj, GOAL)
    prm_results = []
    for name, runner in (
        ("Dijkstra on PRM", lambda: graph_search(adj, s1, g1, lambda _u: 0.0)),
        ("A* on PRM", lambda: astar(adj, nodes, s1, g1, 1.0)),
    ):
        path_i, length, expanded = runner()
        prm_results.append((name, path_i, length, expanded))
        print(f"{name}  expanded={len(expanded)}/{len(nodes)}  length={length:.3f}")

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
    for ax, (name, path_i, length, expanded) in zip(axes, prm_results):
        draw_world(ax)
        for i, nbrs in enumerate(adj):
            for j, _ in nbrs:
                if j < i:
                    continue
                a, b = nodes[i], nodes[j]
                ax.plot([a[0], b[0]], [a[1], b[1]], color="#dddddd", lw=0.3, zorder=1)
        draw_expanded(ax, nodes, expanded)
        draw_path(ax, None if path_i is None else [nodes[i] for i in path_i])
        mark_query(ax, START, GOAL)
        ax.set_title(f"{name}\nlength = {length:.2f}   expanded = {len(expanded)}")
        ax.legend(loc="upper left")
    save_fig(fig, "search_prm.png")

    # h 高估了 A 后面那一步的真实代价，A* 会先锁定那条更长的直连边。
    bad_adj = [[(1, 1.0), (2, 3.0)], [(2, 1.0)], []]
    _, bad_len, bad_exp = graph_search(
        bad_adj, 0, 2, lambda u: 1000.0 if u == 1 else 0.0
    )
    _, good_len, good_exp = graph_search(bad_adj, 0, 2, lambda _u: 0.0)
    print(
        f"inadmissible A*  length={bad_len:.1f}  expanded={bad_exp}  "
        f"(Dijkstra length={good_len:.1f}, expanded={good_exp})"
    )
    return grid_results, prm_results


def self_check():
    assert segment_hits_rect((0.0, 0.0), (1.0, 1.0), Rect(0.4, 0.4, 0.6, 0.6))
    assert not segment_hits_rect((0.0, 0.0), (1.0, 0.0), Rect(2.0, 2.0, 3.0, 3.0))
    assert segment_hits_rect((0.0, 5.0), (10.0, 5.0), Rect(3.0, 4.0, 4.0, 6.0))
    assert not segment_hits_rect((0.0, 0.0), (1.0, 0.0), Rect(0.0, 2.0, 2.0, 3.0))
    assert collides_point((4.0, 3.0))
    assert not collides_point(START)
    assert abs(free_area() - 86.0) < 1e-9
    q = steer((0.0, 0.0), (3.0, 4.0), eta=0.5)
    assert abs(dist((0.0, 0.0), q) - 0.5) < 1e-9
    # S—1—A—1—G 是最短路。把 h(A) 高估成 1000 后，A* 会先收下 S—3—G。
    bad_adj = [[(1, 1.0), (2, 3.0)], [(2, 1.0)], []]
    _, good_len, _ = graph_search(bad_adj, 0, 2, lambda _u: 0.0)
    _, bad_len, _ = graph_search(bad_adj, 0, 2, lambda u: 1000.0 if u == 1 else 0.0)
    assert good_len == 2.0
    assert bad_len == 3.0
    centers, grid_adj, _ = grid_graph()
    s = nearest_index(centers, START)
    g = nearest_index(centers, GOAL)
    _, len_d, exp_d = graph_search(grid_adj, s, g, lambda _u: 0.0)
    _, len_a, exp_a = astar(grid_adj, centers, s, g, 1.0)
    assert abs(len_d - len_a) < 1e-6
    assert len(exp_a) < len(exp_d)


def main(which: str = "all"):
    self_check()
    demos = {
        "rrt": demo_rrt,
        "rrt_connect": demo_rrt_connect,
        "prm": demo_prm,
        "search": demo_search,
        "rrt_star": demo_rrt_star,
        "ompl": demo_ompl,
    }
    if which == "all":
        for name in ("rrt", "rrt_connect", "prm", "search", "rrt_star", "ompl"):
            demos[name]()
    elif which in demos:
        demos[which]()
    else:
        names = ", ".join(["all", *demos])
        raise SystemExit(f"未知演示 {which!r}。可选：{names}")


if __name__ == "__main__":
    import sys

    main(sys.argv[1] if len(sys.argv) > 1 else "all")
