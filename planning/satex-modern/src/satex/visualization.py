"""Static plots and a dependency-free, offline trajectory viewer.

Animations interpolate samples for display only; they do not certify safety
between samples or simulate continuous-time integrator dynamics.
"""

from __future__ import annotations

import json
from html import escape
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from .models import Problem, Region

COLORS = ("#007f86", "#db5b35", "#7061b4", "#c78b1d", "#2471b5", "#c14a85")


def _vertices(region: Region) -> np.ndarray:
    """Enumerate feasible intersections of 2/3D polytope boundary planes."""
    dimension = region.dimension
    vertices = []
    for indices in combinations(range(len(region.b)), dimension):
        matrix = region.A[list(indices)]
        if np.linalg.matrix_rank(matrix) != dimension:
            continue
        vertex = np.linalg.solve(matrix, region.b[list(indices)])
        if np.all(region.A @ vertex <= region.b + 1e-8):
            vertices.append(vertex)
    if not vertices:
        return np.empty((0, dimension))
    points = np.unique(np.round(vertices, decimals=10), axis=0)
    if dimension == 2 and len(points) >= 3:
        try:
            points = points[ConvexHull(points).vertices]
        except QhullError:
            pass
    return points


def _geometry(region: Region, obstacle: bool) -> dict[str, Any]:
    vertices = _vertices(region)
    edges: set[tuple[int, int]] = set()
    faces = []
    if region.dimension == 3 and len(vertices) >= 4:
        try:
            hull = ConvexHull(vertices)
            faces = hull.simplices.tolist()
            for simplex in hull.simplices:
                edges.update(tuple(sorted(map(int, edge))) for edge in combinations(simplex, 2))
        except QhullError:
            pass
    else:
        edges.update((index, (index + 1) % len(vertices)) for index in range(len(vertices)))
    return {
        "name": region.name,
        "vertices": vertices.tolist(),
        "edges": sorted(edges),
        "faces": faces,
        "obstacle": obstacle,
    }


def _payload(problem: Problem, result: Any) -> dict[str, Any]:
    if problem.workspace.dimension not in (2, 3):
        raise ValueError("Trajectory visualization supports 2D and 3D workspaces")
    if result.status != "sat":
        raise ValueError("Only a satisfiable trajectory can be visualized")
    robots = []
    for index, robot in enumerate(problem.robots):
        states = np.asarray(result.states[robot.name], dtype=float)
        positions = states[:, list(robot.dynamics.position_indices)]
        robots.append(
            {
                "name": robot.name,
                "color": COLORS[index % len(COLORS)],
                "positions": positions.tolist(),
                "regions": result.regions[robot.name],
            }
        )
    return {
        "name": problem.name,
        "dimension": problem.workspace.dimension,
        "horizon": result.horizon,
        "mode": result.mode,
        "loop_start": result.loop_start,
        "separation": problem.separation,
        "validation": result.validation,
        "geometry": [_geometry(region, False) for region in problem.workspace.regions]
        + [_geometry(region, True) for region in problem.workspace.obstacles],
        "robots": robots,
    }


def save_plot(problem: Problem, result: Any, path: str | Path) -> Path:
    """Render the sampled trajectory to a publication-friendly PNG."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    data = _payload(problem, result)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    three_d = data["dimension"] == 3
    figure = plt.figure(figsize=(10, 7), facecolor="#fafbf9", constrained_layout=True)
    axes = figure.add_subplot(111, projection="3d" if three_d else None)
    axes.set_facecolor("#fafbf9")
    label_regions = len(data["geometry"]) <= 16
    for shape in data["geometry"]:
        points = np.asarray(shape["vertices"])
        if len(points) == 0:
            continue
        face = "#4c5968" if shape["obstacle"] else "#d9e8e1"
        if three_d and shape["faces"]:
            collection = Poly3DCollection(
                [points[face] for face in shape["faces"]],
                alpha=0.28 if shape["obstacle"] else 0.08,
                facecolor=face,
                edgecolor="#8b9b9b",
                linewidth=0.35,
            )
            axes.add_collection3d(collection)
        elif not three_d:
            axes.add_patch(
                Polygon(
                    points,
                    closed=True,
                    facecolor=face,
                    alpha=0.65 if shape["obstacle"] else 0.5,
                    edgecolor="#a3b7ad",
                    linewidth=0.8,
                )
            )
        if label_regions and not shape["obstacle"]:
            center = points.mean(axis=0)
            if not three_d:
                center[1] = points[:, 1].max() - 0.15 * np.ptp(points[:, 1])
            axes.text(*center, shape["name"], color="#60706b", fontsize=8, ha="center")
    all_points = []
    for robot in data["robots"]:
        positions = np.asarray(robot["positions"])
        all_points.extend(positions)
        axes.plot(
            *positions.T, color=robot["color"], linewidth=2.1, marker=".", markersize=5, label=robot["name"]
        )
        axes.scatter(*positions[0], color=robot["color"], s=55, marker="o", edgecolors="white")
        axes.scatter(*positions[-1], color=robot["color"], s=55, marker="s", edgecolors="white")
        if data["loop_start"] is not None:
            axes.scatter(
                *positions[data["loop_start"]],
                color=robot["color"],
                s=100,
                marker="*",
                edgecolors="#172b32",
                linewidths=0.7,
            )
    axes.set_xlabel("x")
    axes.set_ylabel("y")
    if three_d:
        axes.set_zlabel("z")
        for shape in data["geometry"]:
            all_points.extend(shape["vertices"])
        points = np.asarray(all_points)
        lower, upper = points.min(axis=0), points.max(axis=0)
        span = np.maximum(upper - lower, 0.5)
        axes.set_xlim(lower[0] - 0.05 * span[0], upper[0] + 0.05 * span[0])
        axes.set_ylim(lower[1] - 0.05 * span[1], upper[1] + 0.05 * span[1])
        axes.set_zlim(lower[2] - 0.05 * span[2], upper[2] + 0.05 * span[2])
        axes.set_box_aspect(span)
        axes.view_init(elev=26, azim=-55)
    else:
        axes.autoscale_view()
        axes.set_aspect("equal", adjustable="datalim")
    axes.grid(alpha=0.15)
    axes.legend(loc="upper right", frameon=False)
    suffix = "periodic infinite trace" if result.mode == "lasso" else "finite trace"
    axes.set_title(
        f"{problem.name}  ·  {suffix}\n{result.horizon} labeled samples",
        loc="left",
        color="#172b32",
        fontsize=14,
        pad=15,
    )
    figure.supxlabel(
        "Dots: samples   Circle: start   Square: endpoint   Star: loop start\n"
        "Lines connect samples for display; safety constraints apply at samples only.",
        fontsize=8,
        color="#526069",
    )
    figure.savefig(destination, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return destination


def save_html(problem: Problem, result: Any, path: str | Path) -> Path:
    """Save a portable canvas animation containing all its data and assets."""
    payload = (
        json.dumps(_payload(problem, result), allow_nan=False).replace("<", "\\u003c").replace(">", "\\u003e")
    )
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    html = _HTML.replace("__TITLE__", escape(problem.name)).replace("__DATA__", payload)
    destination.write_text(html, encoding="utf-8")
    return destination


def save_visualizations(problem: Problem, result: Any, directory: str | Path) -> None:
    directory = Path(directory)
    save_plot(problem, result, directory / "trajectory.png")
    save_html(problem, result, directory / "trajectory.html")


_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__ · SatEX trajectory</title><style>
:root{color-scheme:light;font:15px -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:#172b32;background:#eef2ef}
*{box-sizing:border-box}body{margin:0}main{max-width:1200px;margin:auto;padding:36px 32px}header{display:flex;justify-content:space-between;gap:20px;align-items:center;margin-bottom:24px}
.eyebrow{color:#467467;text-transform:uppercase;letter-spacing:.16em;font-size:11px;font-weight:700}h1{font-size:30px;letter-spacing:-.04em;margin:7px 0}p{line-height:1.6;margin:0;color:#5f706b}
.badge{border:1px solid #c7d9d0;border-radius:30px;padding:8px 13px;font-size:12px;white-space:nowrap;background:#f6fbf7}.stage{background:#fbfcfa;border:1px solid #dce5df;border-radius:16px;box-shadow:0 12px 40px #263d2610;overflow:hidden}
.toolbar{display:flex;gap:12px;padding:17px 20px;border-bottom:1px solid #e4ebe6;align-items:center;flex-wrap:wrap}button,select{font:inherit;font-size:13px;border:1px solid #cbd8d0;border-radius:7px;background:white;color:#243e34;padding:8px 12px;cursor:pointer}button.primary{background:#176958;color:white;border-color:#176958}button:focus-visible,select:focus-visible,input:focus-visible{outline:3px solid #e6ae46;outline-offset:2px}.spacer{flex:1}label{font-size:12px;color:#54685e}canvas{width:100%;height:520px;display:block}input[type=range]{accent-color:#176958;cursor:pointer}.scrubber{display:grid;grid-template-columns:auto 1fr auto;align-items:center;gap:15px;border-top:1px solid #e4ebe6;padding:20px}.scrubber input{width:100%}.scrubber output{font-variant-numeric:tabular-nums;min-width:115px;text-align:right;font-size:13px}
.legend{display:flex;gap:22px;flex-wrap:wrap;padding:0 22px 20px}.robot{font-size:13px;display:flex;align-items:center;gap:8px}.dot{width:9px;height:9px;border-radius:50%;display:inline-block}.meta{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-top:18px}.meta>div{padding:16px;border:1px solid #dce5df;border-radius:10px;background:#f6f9f6}.meta small{display:block;color:#667b70;font-size:11px;text-transform:uppercase;letter-spacing:.08em;margin-bottom:6px}.meta strong{font-size:17px;font-weight:600}.note{font-size:12px;line-height:1.65;margin:18px 2px;color:#63736a}.foot{font-size:11px;color:#849188;margin-top:24px}.region{color:#667970;font-size:12px}@media(max-width:600px){main{padding:24px 14px}h1{font-size:24px}header{display:block}.badge{display:inline-block;margin-top:12px}canvas{height:380px}.meta{grid-template-columns:1fr}.scrubber{padding:16px;gap:8px}.scrubber output{min-width:90px}}
</style></head><body><main><header><div><div class="eyebrow">SatEX Modern / motion planning</div><h1>__TITLE__</h1><p id="subtitle"></p></div><span class="badge" id="status">Trajectory explorer</span></header>
<section class="stage" aria-label="Trajectory visualization"><div class="toolbar"><button class="primary" id="play" aria-label="Play trajectory">Play</button><button id="reset">Reset</button><label>Speed <select id="speed"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option></select></label><span class="spacer"></span><label id="viewLabel">View <select id="view"><option value="iso">Isometric</option><option value="top">Top (x–y)</option></select></label><label><input type="checkbox" id="trail" checked> Full path</label></div>
<canvas id="canvas" role="img" aria-label="Robot trajectories and workspace regions"></canvas><div class="scrubber"><label for="time">Sample</label><input id="time" type="range" min="0" max="1" step="0.01" value="0"><output id="sample"></output></div><div class="legend" id="legend"></div></section>
<div class="meta"><div><small>Trace semantics</small><strong id="semantics"></strong></div><div><small>Robot clearance</small><strong id="clearance"></strong></div><div><small>Loop starts at sample</small><strong id="loop"></strong></div></div>
<p class="note">Dots show planned samples. Playback linearly interpolates them for display; it is not a continuous dynamics simulation. Collision and obstacle guarantees apply only at sample instants. Coordinates are in the model's units. Pale regions are free space; dark regions are obstacles.</p><p class="foot">Self-contained offline viewer · Generated by SatEX Modern · No network requests</p></main>
<script type="application/json" id="data">__DATA__</script><script>
"use strict";
const data=JSON.parse(document.getElementById("data").textContent),$=id=>document.getElementById(id);
const canvas=$("canvas"),ctx=canvas.getContext("2d"),slider=$("time"),last=data.robots[0].positions.length-1;
let playing=false,previous=0,time=0;
slider.max=last;$("subtitle").textContent=`${data.robots.length} robot${data.robots.length===1?"":"s"} · ${data.dimension}D workspace · ${data.horizon} labeled samples`;
$("semantics").textContent=data.mode==="lasso"?"Periodic infinite trace":"Finite trace";$("clearance").textContent=`${data.separation} · infinity norm`;
$("loop").textContent=data.loop_start===null?"No loop":data.loop_start;$("status").textContent=data.validation?.valid===true?"Independently validated":"Planned trajectory";
if(data.dimension!==3)$("viewLabel").hidden=true;
for(const robot of data.robots){const item=document.createElement("div");item.className="robot";const dot=document.createElement("span");dot.className="dot";dot.style.background=robot.color;const name=document.createElement("strong");name.textContent=robot.name;const region=document.createElement("span");region.className="region";region.id=`region-${data.robots.indexOf(robot)}`;item.append(dot,name,region);$("legend").append(item)}
function projection(p){if(data.dimension===2||$("view").value==="top")return[p[0],p[1]];return[.866*(p[0]-p[1]),.48*(p[0]+p[1])+p[2]]}
function draw(){const rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);const width=rect.width,height=rect.height;ctx.clearRect(0,0,width,height);
const all=[...data.geometry.flatMap(g=>g.vertices),...data.robots.flatMap(r=>r.positions)].map(projection);let low=[Infinity,Infinity],high=[-Infinity,-Infinity];for(const p of all)for(let k=0;k<2;k++){low[k]=Math.min(low[k],p[k]);high[k]=Math.max(high[k],p[k])}const span=high.map((x,k)=>Math.max(x-low[k],1)),scale=Math.min((width-100)/span[0],(height-100)/span[1]),center=low.map((x,k)=>(x+high[k])/2);const screen=p=>{const q=projection(p);return[width/2+(q[0]-center[0])*scale,height/2-(q[1]-center[1])*scale]};
ctx.font="11px system-ui";ctx.textAlign="center";ctx.lineJoin="round";
for(const g of data.geometry){if(!g.vertices.length)continue;const points=g.vertices.map(screen);ctx.strokeStyle=g.obstacle?"#65716c77":"#a9bdb08a";ctx.fillStyle=g.obstacle?"#65716c55":"#d9e8df66";ctx.lineWidth=1;if(data.dimension===2){ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.fill();ctx.stroke()}else{if(g.obstacle){for(const face of g.faces){ctx.beginPath();face.forEach((i,j)=>j?ctx.lineTo(...points[i]):ctx.moveTo(...points[i]));ctx.closePath();ctx.fill()}}for(const edge of g.edges){ctx.beginPath();ctx.moveTo(...points[edge[0]]);ctx.lineTo(...points[edge[1]]);ctx.stroke()}}
if(data.geometry.length<=16&&!g.obstacle){const center=points.reduce((a,p)=>[a[0]+p[0]/points.length,a[1]+p[1]/points.length],[0,0]);ctx.fillStyle="#708779";ctx.fillText(g.name,center[0],center[1]-12)}}
data.robots.forEach((robot,index)=>{const points=robot.positions.map(screen);ctx.strokeStyle=robot.color;ctx.fillStyle=robot.color;if($("trail").checked){ctx.globalAlpha=.4;ctx.lineWidth=2;ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.stroke();for(const p of points){ctx.beginPath();ctx.arc(...p,2.4,0,Math.PI*2);ctx.fill()}ctx.globalAlpha=1}if(data.loop_start!==null){const q=points[data.loop_start];ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(...q,8,0,Math.PI*2);ctx.stroke()}
const base=Math.min(Math.floor(time),last),next=Math.min(base+1,last),fraction=time-base,p=robot.positions[base].map((x,k)=>x*(1-fraction)+robot.positions[next][k]*fraction),q=screen(p);ctx.beginPath();ctx.arc(...q,7,0,Math.PI*2);ctx.fill();ctx.strokeStyle="#fff";ctx.lineWidth=2;ctx.stroke();ctx.fillStyle=robot.color;ctx.font="bold 12px system-ui";ctx.fillText(robot.name,q[0],q[1]-13);$("region-"+index).textContent=robot.regions[base]??robot.regions[data.loop_start??robot.regions.length-1]});
ctx.font="11px system-ui";ctx.textAlign="left";ctx.fillStyle="#7c8c80";ctx.fillText(data.dimension===3&&$("view").value==="iso"?"Isometric projection · z points upward":"x →    y ↑",18,height-18);$("sample").textContent=`${time.toFixed(2)} / ${last}`;slider.value=time;}
function stop(){playing=false;$("play").textContent="Play";$("play").setAttribute("aria-label","Play trajectory")}
function tick(now){if(!playing)return;if(previous){time+=(now-previous)/1000*Number($("speed").value);if(time>=last){if(data.mode==="lasso"){const cycle=last-data.loop_start;time=data.loop_start+(time-last)%cycle}else{time=last;stop()}}}previous=now;draw();if(playing)requestAnimationFrame(tick)}
$("play").onclick=()=>{if(playing){stop();return}if(time>=last)time=data.loop_start??0;playing=true;previous=0;$("play").textContent="Pause";$("play").setAttribute("aria-label","Pause trajectory");requestAnimationFrame(tick)};
$("reset").onclick=()=>{stop();time=0;draw()};slider.oninput=()=>{stop();time=Number(slider.value);draw()};$("view").onchange=draw;$("trail").onchange=draw;window.addEventListener("resize",draw);draw();
</script></body></html>"""
