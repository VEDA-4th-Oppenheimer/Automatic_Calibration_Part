#!/usr/bin/env python3
"""Convert this project's ASCII point/line PLY into triangle-mesh PLY/OBJ."""

import argparse
import math
from pathlib import Path


def read_ply(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    vertex_count = edge_count = face_count = 0
    header_end = None
    for i, line in enumerate(lines):
        fields = line.split()
        if fields[:2] == ["element", "vertex"]:
            vertex_count = int(fields[2])
        elif fields[:2] == ["element", "edge"]:
            edge_count = int(fields[2])
        elif fields[:2] == ["element", "face"]:
            face_count = int(fields[2])
        elif line == "end_header":
            header_end = i + 1
            break
    if header_end is None or lines[1:2] != ["format ascii 1.0"]:
        raise ValueError("Only ASCII PLY 1.0 is supported")
    vertices = []
    for line in lines[header_end : header_end + vertex_count]:
        fields = line.split()
        color = tuple(map(int, fields[3:6])) if len(fields) >= 6 else (180,) * 3
        vertices.append((tuple(map(float, fields[:3])), color))
    edges = []
    for line in lines[header_end + vertex_count :
                      header_end + vertex_count + edge_count]:
        a, b = map(int, line.split()[:2])
        edges.append((a, b))
    face_start = header_end + vertex_count + edge_count
    faces = [tuple(map(int, line.split()[1:4]))
             for line in lines[face_start : face_start + face_count]]
    return vertices, edges, faces


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def normalized(a):
    length = math.sqrt(sum(value * value for value in a))
    if length <= 1e-12:
        raise ValueError("Zero-length segment")
    return tuple(value / length for value in a)


def write_mesh(stem, vertices, normals, colors, faces, comment):
    if not (len(vertices) == len(normals) == len(colors)) or \
            not vertices or not faces:
        raise ValueError("Mesh must contain matching vertices/normals/colors")
    if min(index for face in faces for index in face) < 0 or \
            max(index for face in faces for index in face) >= len(vertices):
        raise ValueError("Mesh face index is outside the vertex array")
    stem = Path(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    with stem.with_suffix(".ply").open("w", encoding="utf-8") as out:
        out.write("ply\nformat ascii 1.0\n")
        out.write(f"comment {comment}\n")
        out.write(f"element vertex {len(vertices)}\n")
        out.write("property float x\nproperty float y\nproperty float z\n")
        out.write("property float nx\nproperty float ny\nproperty float nz\n")
        out.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        out.write(f"element face {len(faces)}\n")
        out.write("property list uchar int vertex_indices\nend_header\n")
        for point, normal, color in zip(vertices, normals, colors):
            out.write(f"{point[0]:.9g} {point[1]:.9g} {point[2]:.9g} "
                      f"{normal[0]:.9g} {normal[1]:.9g} {normal[2]:.9g} "
                      f"{color[0]} {color[1]} {color[2]}\n")
        for face in faces:
            out.write(f"3 {face[0]} {face[1]} {face[2]}\n")
    with stem.with_suffix(".obj").open("w", encoding="utf-8") as out:
        out.write(f"# {comment}\n")
        for point, normal, color in zip(vertices, normals, colors):
            out.write(f"v {point[0]:.9g} {point[1]:.9g} {point[2]:.9g} "
                      f"{color[0] / 255:.6g} {color[1] / 255:.6g} "
                      f"{color[2] / 255:.6g}\n")
            out.write(f"vn {normal[0]:.9g} {normal[1]:.9g} "
                      f"{normal[2]:.9g}\n")
        for face in faces:
            out.write(f"f {face[0] + 1}//{face[0] + 1} "
                      f"{face[1] + 1}//{face[1] + 1} "
                      f"{face[2] + 1}//{face[2] + 1}\n")


def point_mesh(source, stem, maximum_points, radius):
    points, _, _ = read_ply(source)
    stride = max(1, math.ceil(len(points) / maximum_points))
    points = points[::stride]
    offsets = ((radius, radius, radius), (radius, -radius, -radius),
               (-radius, radius, -radius), (-radius, -radius, radius))
    tetra_faces = ((0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2))
    vertices, normals, colors, faces = [], [], [], []
    for center, color in points:
        base = len(vertices)
        vertices.extend(tuple(center[i] + offset[i] for i in range(3))
                        for offset in offsets)
        normals.extend(normalized(offset) for offset in offsets)
        colors.extend([color] * 4)
        faces.extend(tuple(base + index for index in face)
                     for face in tetra_faces)
    write_mesh(stem, vertices, normals, colors, faces,
               "VS Code 3D Viewer compatible point-cloud preview mesh")


def segment_mesh(source, stem, radius):
    points, edges, _ = read_ply(source)
    if not edges:
        raise ValueError("PLY has no edge elements")
    bar_faces = ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
                 (0, 5, 1), (0, 4, 5), (1, 6, 2), (1, 5, 6),
                 (2, 7, 3), (2, 6, 7), (3, 4, 0), (3, 7, 4))
    vertices, normals, colors, faces = [], [], [], []
    for a_index, b_index in edges:
        a, color = points[a_index]
        b, _ = points[b_index]
        direction = normalized(tuple(b[i] - a[i] for i in range(3)))
        helper = (0, 0, 1) if abs(direction[2]) < 0.9 else (0, 1, 0)
        u = tuple(radius * value for value in normalized(cross(direction, helper)))
        v = tuple(radius * value for value in normalized(cross(direction, u)))
        base = len(vertices)
        vertices.extend(tuple(center[i] + su * u[i] + sv * v[i]
                              for i in range(3))
                        for center in (a, b) for su, sv in
                        ((1, 1), (1, -1), (-1, -1), (-1, 1)))
        normals.extend(normalized(tuple(su * u[i] + sv * v[i]
                                        for i in range(3)))
                       for _center in (a, b) for su, sv in
                       ((1, 1), (1, -1), (-1, -1), (-1, 1)))
        colors.extend([color] * 8)
        faces.extend(tuple(base + index for index in face) for face in bar_faces)
    write_mesh(stem, vertices, normals, colors, faces,
               "VS Code 3D Viewer compatible structural-segment mesh")


def repair_bar_mesh(source, stem):
    points, _, source_faces = read_ply(source)
    if len(points) % 8 or not source_faces:
        raise ValueError("Expected the generated 8-vertex structural bars")
    vertices = [point for point, _ in points]
    colors = [color for _, color in points]
    normals = []
    for base in range(0, len(vertices), 8):
        for endpoint in (vertices[base:base + 4], vertices[base + 4:base + 8]):
            center = tuple(sum(point[i] for point in endpoint) / 4
                           for i in range(3))
            normals.extend(normalized(tuple(point[i] - center[i]
                                            for i in range(3)))
                           for point in endpoint)
    faces = [(face[0], face[2], face[1]) for face in source_faces]
    write_mesh(stem, vertices, normals, colors, faces,
               "VS Code 3D Viewer compatible structural-segment mesh")


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="kind", required=True)
    points = subparsers.add_parser("points")
    points.add_argument("source")
    points.add_argument("stem")
    points.add_argument("--max-points", type=int, default=12000)
    points.add_argument("--radius", type=float, default=25.0)
    segments = subparsers.add_parser("segments")
    segments.add_argument("source")
    segments.add_argument("stem")
    segments.add_argument("--radius", type=float, default=0.005)
    bars = subparsers.add_parser("repair-bars")
    bars.add_argument("source")
    bars.add_argument("stem")
    args = parser.parse_args()
    if args.kind == "points":
        point_mesh(args.source, args.stem, args.max_points, args.radius)
    elif args.kind == "segments":
        segment_mesh(args.source, args.stem, args.radius)
    else:
        repair_bar_mesh(args.source, args.stem)


if __name__ == "__main__":
    main()
