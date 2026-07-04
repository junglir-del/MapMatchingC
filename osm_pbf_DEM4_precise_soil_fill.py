#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import struct
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import osmium
from mapbox_earcut import triangulate_float64
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Polygon, box
from shapely.ops import unary_union

ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963
FLOAT = 5126
UNSIGNED_INT = 5125
TRIANGLES = 4

# RF material properties are frequency/moisture dependent, so the exporter keeps
# material names but leaves dielectric/conductivity values unset.
MATERIAL_ELECTRICAL_PROPERTIES = {
    "brick": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "bricks": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "concrete": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "cement": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "glass": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "wood": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "stone": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "plaster": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "metal": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "steel": {"dielectric_constant": None, "conductivity_s_per_m": None},
}

SURFACE_ELECTRICAL_PROPERTIES = {
    "asphalt": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "fresh_water": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "grass": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "forest": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "sand": {"dielectric_constant": None, "conductivity_s_per_m": None},
    "soil": {"dielectric_constant": None, "conductivity_s_per_m": None},
}

def parse_meters(value):
    if not value:
        return None
    text = str(value).strip().lower().replace(",", ".")
    match = re.search(r"-?\d+(\.\d+)?", text)
    if not match:
        return None
    return float(match.group(0))


def clean_text(value):
    return str(value).replace("\x00", "").strip()


def building_height(tags):
    height = (
        parse_meters(tags.get("height"))
        or parse_meters(tags.get("building:height"))
        or parse_meters(tags.get("roof:height"))
    )
    if height and height > 0:
        return height

    levels = parse_meters(tags.get("building:levels") or tags.get("levels"))
    if levels and levels > 0:
        return levels * 3.2

    return 8.0


def building_name(tags, osm_id):
    return (
        tags.get("name")
        or tags.get("building:name")
        or tags.get("addr:housename")
        or f"building/{osm_id}"
    )


def feature_name(tags, osm_id, feature_type):
    return clean_text(tags.get("name") or f"{feature_type}/{osm_id}")


def actual_building_material(tags):
    material_keys = [
        "building:material",
        "facade:material",
        "material",
        "wall:material",
        "cladding",
        "roof:material",
    ]

    for key in material_keys:
        value = tags.get(key)
        if value:
            return clean_text(value).lower(), key

    return "unknown", None


def infer_building_material(tags):
    building_type = (tags.get("building") or "").lower()
    amenity = (tags.get("amenity") or "").lower()
    name = (tags.get("name") or "").lower()

    inferred = {
        "apartments": "concrete",
        "residential": "concrete",
        "house": "brick",
        "detached": "brick",
        "terrace": "brick",
        "commercial": "glass",
        "retail": "glass",
        "office": "glass",
        "industrial": "metal",
        "warehouse": "metal",
        "garage": "concrete",
        "school": "brick",
        "university": "brick",
        "hospital": "concrete",
        "hotel": "concrete",
    }

    if building_type in inferred:
        return inferred[building_type], f"building:{building_type}"
    if amenity == "fire_station" or "消防" in name:
        return "brick", "amenity:fire_station"

    return "brick", "default_brick"


def resolve_building_material(tags, material_mode):
    actual, source_key = actual_building_material(tags)

    if actual != "unknown":
        return {
            "osm_material": actual,
            "inferred_material": actual,
            "material_inference_source": f"osm:{source_key}",
        }

    if material_mode == "infer":
        inferred, source = infer_building_material(tags)
        return {
            "osm_material": None,
            "inferred_material": inferred,
            "material_inference_source": source,
        }

    return {
        "osm_material": None,
        "inferred_material": "brick",
        "material_inference_source": "default_brick",
    }


def material_electrical_properties(material):
    lookup_material = (material or "unknown").lower()
    if lookup_material not in MATERIAL_ELECTRICAL_PROPERTIES:
        lookup_material = "brick"

    props = MATERIAL_ELECTRICAL_PROPERTIES[lookup_material]
    return {
        "dielectric_constant": props["dielectric_constant"],
        "conductivity_s_per_m": props["conductivity_s_per_m"],
    }


def infer_surface_material(feature_type, tags):
    if feature_type == "road":
        return "asphalt", f"highway:{tags.get('highway')}"
    if feature_type == "water":
        return "fresh_water", "natural:water"
    if feature_type == "grass":
        return "grass", "surface:grass"
    if feature_type == "forest":
        return "forest", "surface:forest"
    if feature_type == "sand":
        return "sand", "surface:sand"
    return "soil", "default_soil"


def surface_electrical_properties(material):
    lookup_material = (material or "soil").lower()
    if lookup_material not in SURFACE_ELECTRICAL_PROPERTIES:
        lookup_material = "soil"

    props = SURFACE_ELECTRICAL_PROPERTIES[lookup_material]
    return {
        "dielectric_constant": props["dielectric_constant"],
        "conductivity_s_per_m": props["conductivity_s_per_m"],
    }



def material_color(material):
    material = (material or "unknown").lower()
    colors = {
        "brick": [0.62, 0.22, 0.14, 1.0],
        "bricks": [0.62, 0.22, 0.14, 1.0],
        "concrete": [0.58, 0.58, 0.55, 1.0],
        "cement": [0.55, 0.55, 0.52, 1.0],
        "glass": [0.45, 0.72, 0.9, 0.55],
        "steel": [0.48, 0.5, 0.52, 1.0],
        "metal": [0.5, 0.5, 0.5, 1.0],
        "wood": [0.55, 0.34, 0.18, 1.0],
        "stone": [0.5, 0.48, 0.42, 1.0],
        "plaster": [0.78, 0.74, 0.66, 1.0],
        "unknown": [0.72, 0.70, 0.64, 1.0],
    }
    return colors.get(material, colors["unknown"])


def feature_color(feature_type):
    colors = {
        "road": [0.44, 0.46, 0.48, 1.0],
        "water": [0.02, 0.32, 0.90, 1.0],
        "grass": [0.20, 0.62, 0.24, 1.0],
        "forest": [0.05, 0.36, 0.12, 1.0],
        "sand": [0.70, 0.64, 0.44, 1.0],
        "land": [0.52, 0.38, 0.24, 1.0],
    }
    return colors.get(feature_type, colors["land"])


def is_water_feature(tags):
    return (
        tags.get("natural") == "water"
        or tags.get("natural") == "bay"
        or tags.get("water") in ("lake", "pond", "reservoir", "basin")
        or tags.get("landuse") == "reservoir"
        or tags.get("waterway") == "riverbank"
    )


def relation_member_is_way(member):
    member_type = getattr(member, "type", None)
    return member_type == "w" or str(member_type).lower() in ("w", "way")


def join_way_segments(segments):
    remaining = [list(segment) for segment in segments if len(segment) >= 2]
    rings = []

    while remaining:
        ring = remaining.pop(0)
        changed = True

        while changed and ring[0] != ring[-1]:
            changed = False

            for i, segment in enumerate(remaining):
                if ring[-1] == segment[0]:
                    ring.extend(segment[1:])
                elif ring[-1] == segment[-1]:
                    ring.extend(reversed(segment[:-1]))
                elif ring[0] == segment[-1]:
                    ring = segment[:-1] + ring
                elif ring[0] == segment[0]:
                    ring = list(reversed(segment[1:])) + ring
                else:
                    continue

                remaining.pop(i)
                changed = True
                break

        if len(ring) >= 4 and ring[0] == ring[-1]:
            rings.append(ring)

    return rings


def landcover_type(tags):
    natural = tags.get("natural")
    landuse = tags.get("landuse")
    leisure = tags.get("leisure")

    if natural in ("wood", "scrub") or landuse == "forest":
        return "forest"
    if natural in ("grassland", "heath") or landuse in ("grass", "meadow"):
        return "grass"
    if natural in ("beach", "sand"):
        return "sand"
    if leisure in ("park", "garden", "pitch"):
        return "grass"
    return None


def road_width_m(tags):
    width = parse_meters(tags.get("width"))
    if width and width > 0:
        return width

    highway = tags.get("highway")
    defaults = {
        "motorway": 14.0,
        "trunk": 12.0,
        "primary": 10.0,
        "secondary": 8.0,
        "tertiary": 7.0,
        "residential": 5.5,
        "service": 3.5,
        "footway": 2.0,
        "path": 1.5,
        "cycleway": 2.0,
    }
    return defaults.get(highway, 4.0)


def mercator_xy(lon, lat, origin_lon, origin_lat):
    radius = 6378137.0
    x = math.radians(lon - origin_lon) * radius * math.cos(math.radians(origin_lat))
    y = math.radians(lat - origin_lat) * radius
    return x, y


def xy_to_lonlat(x, y, origin_lon, origin_lat):
    radius = 6378137.0
    lon = origin_lon + math.degrees(x / (radius * math.cos(math.radians(origin_lat))))
    lat = origin_lat + math.degrees(y / radius)
    return lon, lat


def point_in_polygon(lon, lat, polygon):
    if len(polygon) < 4:
        return False

    inside = False
    j = len(polygon) - 1

    for i, (lon_i, lat_i) in enumerate(polygon):
        lon_j, lat_j = polygon[j]
        intersects = ((lat_i > lat) != (lat_j > lat)) and (
            lon < (lon_j - lon_i) * (lat - lat_i) / ((lat_j - lat_i) or 1e-12) + lon_i
        )
        if intersects:
            inside = not inside
        j = i

    return inside


def polygon_bbox(polygon):
    lons = [lon for lon, _ in polygon]
    lats = [lat for _, lat in polygon]
    return min(lons), min(lats), max(lons), max(lats)


def point_in_lonlat_bbox(lon, lat, bbox):
    min_lon, min_lat, max_lon, max_lat = bbox
    return min_lon <= lon <= max_lon and min_lat <= lat <= max_lat


def point_to_segment_distance_m(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)

    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


def point_near_prepared_road(lon, lat, prepared_road, origin_lon, origin_lat):
    px, py = mercator_xy(lon, lat, origin_lon, origin_lat)
    half_width = prepared_road["half_width"]
    segments = prepared_road["segments"]

    for ax, ay, bx, by in segments:
        if point_to_segment_distance_m(px, py, ax, ay, bx, by) <= half_width:
            return True

    return False


def terrain_cell_excluded(lon, lat, exclusion_polygons, exclusion_roads, origin_lon, origin_lat):
    for polygon in exclusion_polygons:
        if not point_in_lonlat_bbox(lon, lat, polygon["bbox"]):
            continue
        if point_in_polygon(lon, lat, polygon["coords"]):
            return True

    for road in exclusion_roads:
        if not point_in_lonlat_bbox(lon, lat, road["bbox"]):
            continue
        if point_near_prepared_road(lon, lat, road, origin_lon, origin_lat):
            return True

    return False


def prepare_terrain_exclusion_polygons(polygons):
    prepared = []
    for polygon in polygons:
        if len(polygon) < 4:
            continue
        prepared.append({
            "coords": polygon,
            "bbox": polygon_bbox(polygon),
        })
    return prepared


def prepare_terrain_exclusion_roads(roads, origin_lon, origin_lat, extra_margin=0.5):
    prepared = []

    for road in roads:
        coords = road["coords"]
        if len(coords) < 2:
            continue

        half_width = road_width_m(road["tags"]) / 2.0 + extra_margin
        segments = []

        for i in range(len(coords) - 1):
            ax, ay = mercator_xy(coords[i][0], coords[i][1], origin_lon, origin_lat)
            bx, by = mercator_xy(coords[i + 1][0], coords[i + 1][1], origin_lon, origin_lat)
            segments.append((ax, ay, bx, by))

        lons = [lon for lon, _ in coords]
        lats = [lat for _, lat in coords]
        center_lat = sum(lats) / len(lats)
        lat_margin = half_width / 111320.0
        lon_margin = half_width / (111320.0 * max(math.cos(math.radians(center_lat)), 0.01))

        prepared.append({
            "bbox": (
                min(lons) - lon_margin,
                min(lats) - lat_margin,
                max(lons) + lon_margin,
                max(lats) + lat_margin,
            ),
            "half_width": half_width,
            "segments": segments,
        })

    return prepared


def point_in_bbox(lon, lat, bbox):
    min_lon, min_lat, max_lon, max_lat = bbox
    return min_lon <= lon <= max_lon and min_lat <= lat <= max_lat


def any_point_in_bbox(coords, bbox):
    return any(point_in_bbox(lon, lat, bbox) for lon, lat in coords)


def download_opentopography_dem(bbox, dem_type, output_path, api_key):
    if not api_key:
        raise ValueError(
            "OpenTopography API key is required. Use --opentopo-key or OPENTOPO_API_KEY."
        )

    min_lon, min_lat, max_lon, max_lat = bbox
    params = {
        "demtype": dem_type,
        "south": min_lat,
        "north": max_lat,
        "west": min_lon,
        "east": max_lon,
        "outputFormat": "GTiff",
        "API_Key": api_key,
    }
    url = "https://portal.opentopography.org/API/globaldem?" + urllib.parse.urlencode(params)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Downloading DEM from OpenTopography ({dem_type})...")
    with urllib.request.urlopen(url) as response:
        content_type = response.headers.get("Content-Type", "")
        data = response.read()

    if data[:4] not in (b"II*\x00", b"MM\x00*"):
        text = data[:500].decode("utf-8", errors="replace")
        raise RuntimeError(
            "OpenTopography did not return a GeoTIFF. "
            f"Content-Type: {content_type}. Response starts with: {text}"
        )

    output_path.write_bytes(data)
    print(f"Saved DEM GeoTIFF: {output_path}")
    return output_path


class DemSampler:
    def __init__(self, dem_path):
        try:
            import rasterio
        except ImportError as exc:
            raise RuntimeError(
                "DEM support requires rasterio. Install it with: pip install rasterio"
            ) from exc

        self.path = Path(dem_path)
        self.dataset = rasterio.open(self.path)
        self.nodata = self.dataset.nodata

    def sample(self, lon, lat, default=0.0):
        try:
            value = next(self.dataset.sample([(lon, lat)]))[0]
        except Exception:
            return default

        if value is None:
            return default

        value = float(value)
        if math.isnan(value):
            return default
        if self.nodata is not None and value == self.nodata:
            return default
        return value

    def close(self):
        self.dataset.close()


class OsmCollector(osmium.SimpleHandler):
    def __init__(self, bbox, total_items=0):
        super().__init__()
        self.bbox = bbox
        self.total_items = total_items
        self.processed_items = 0
        self.next_progress_percent = 5
        self.buildings = []
        self.roads = []
        self.areas = []
        self.way_geometries = {}

    def update_progress(self):
        if not self.total_items:
            return

        self.processed_items += 1
        percent = int(self.processed_items * 100 / self.total_items)

        while percent >= self.next_progress_percent and self.next_progress_percent <= 100:
            print(f"OSM read progress: {self.next_progress_percent}%")
            self.next_progress_percent += 5

    def way(self, way):
        self.update_progress()
        tags = {tag.k: tag.v for tag in way.tags}

        coords = []
        for node in way.nodes:
            if not node.location.valid():
                return
            coords.append((node.location.lon, node.location.lat))

        if len(coords) < 2:
            return
        if not any_point_in_bbox(coords, self.bbox):
            return

        self.way_geometries[way.id] = coords

        feature = {
            "osm_type": "way",
            "id": way.id,
            "coords": coords,
            "tags": tags,
        }

        is_closed = len(coords) >= 4 and coords[0] == coords[-1]

        if "building" in tags and is_closed:
            self.buildings.append(feature)
        elif "highway" in tags:
            self.roads.append(feature)
        elif is_closed and is_water_feature(tags):
            feature["feature_type"] = "water"
            self.areas.append(feature)
        elif is_closed:
            cover_type = landcover_type(tags)
            if cover_type:
                feature["feature_type"] = cover_type
                self.areas.append(feature)

    def relation(self, relation):
        self.update_progress()
        tags = {tag.k: tag.v for tag in relation.tags}

        if tags.get("type") != "multipolygon":
            return

        if is_water_feature(tags):
            feature_type = "water"
        else:
            feature_type = landcover_type(tags)

        if not feature_type:
            return

        outer_segments = []
        for member in relation.members:
            if not relation_member_is_way(member):
                continue
            if member.role and member.role != "outer":
                continue

            coords = self.way_geometries.get(member.ref)
            if coords:
                outer_segments.append(coords)

        for ring in join_way_segments(outer_segments):
            if not any_point_in_bbox(ring, self.bbox):
                continue

            self.areas.append({
                "osm_type": "relation",
                "id": relation.id,
                "coords": ring,
                "tags": tags,
                "feature_type": feature_type,
            })


class OsmProgressCounter(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.total_items = 0

    def way(self, way):
        self.total_items += 1

    def relation(self, relation):
        self.total_items += 1


class GltfWriter:
    def __init__(self):
        self.bin = bytearray()
        self.buffer_views = []
        self.accessors = []
        self.meshes = []
        self.nodes = []
        self.materials = []

    def align4(self, pad_byte=0):
        while len(self.bin) % 4:
            self.bin.append(pad_byte)

    def add_bytes(self, data, target):
        self.align4()
        offset = len(self.bin)
        self.bin.extend(data)

        view_index = len(self.buffer_views)
        self.buffer_views.append({
            "buffer": 0,
            "byteOffset": offset,
            "byteLength": len(data),
            "target": target,
        })
        return view_index

    def add_positions(self, positions):
        arr = np.asarray(positions, dtype=np.float32)
        view = self.add_bytes(arr.tobytes(), ARRAY_BUFFER)

        self.accessors.append({
            "bufferView": view,
            "byteOffset": 0,
            "componentType": FLOAT,
            "count": int(len(arr)),
            "type": "VEC3",
            "min": arr.min(axis=0).tolist(),
            "max": arr.max(axis=0).tolist(),
        })
        return len(self.accessors) - 1

    def add_normals(self, normals):
        arr = np.asarray(normals, dtype=np.float32)
        view = self.add_bytes(arr.tobytes(), ARRAY_BUFFER)

        self.accessors.append({
            "bufferView": view,
            "byteOffset": 0,
            "componentType": FLOAT,
            "count": int(len(arr)),
            "type": "VEC3",
        })
        return len(self.accessors) - 1

    def add_indices(self, indices):
        arr = np.asarray(indices, dtype=np.uint32)
        view = self.add_bytes(arr.tobytes(), ELEMENT_ARRAY_BUFFER)

        self.accessors.append({
            "bufferView": view,
            "byteOffset": 0,
            "componentType": UNSIGNED_INT,
            "count": int(len(arr)),
            "type": "SCALAR",
            "min": [int(arr.min())],
            "max": [int(arr.max())],
        })
        return len(self.accessors) - 1

    def add_material(self, name, base_color, metallic=0.0, roughness=0.85):
        material = {
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": base_color,
                "metallicFactor": metallic,
                "roughnessFactor": roughness,
            },
            "doubleSided": True,
        }

        if base_color[3] < 1.0:
            material["alphaMode"] = "BLEND"

        self.materials.append(material)
        return len(self.materials) - 1

    def add_mesh_node(self, name, positions, normals, indices, material_index, extras):
        pos_accessor = self.add_positions(positions)
        normal_accessor = self.add_normals(normals)
        idx_accessor = self.add_indices(indices)

        mesh_index = len(self.meshes)
        self.meshes.append({
            "name": name,
            "primitives": [{
                "attributes": {
                    "POSITION": pos_accessor,
                    "NORMAL": normal_accessor,
                },
                "indices": idx_accessor,
                "material": material_index,
                "mode": TRIANGLES,
            }],
        })

        self.nodes.append({
            "name": name,
            "mesh": mesh_index,
            "extras": extras,
        })

    def build_gltf(self, buffer_uri=None):
        self.align4()
        buffer = {"byteLength": len(self.bin)}

        if buffer_uri:
            buffer["uri"] = buffer_uri

        return {
            "asset": {
                "version": "2.0",
                "generator": "osm_pbf_to_buildings_glb.py",
            },
            "scene": 0,
            "scenes": [{"nodes": list(range(len(self.nodes)))}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "buffers": [buffer],
            "bufferViews": self.buffer_views,
            "accessors": self.accessors,
        }

    def save(self, output_path):
        output_path = Path(output_path)

        if output_path.suffix.lower() == ".glb":
            self.save_glb(output_path)
        elif output_path.suffix.lower() == ".gltf":
            self.save_gltf(output_path)
        else:
            raise ValueError("Output file must end with .glb or .gltf")

    def save_gltf(self, gltf_path):
        bin_path = gltf_path.with_suffix(".bin")
        gltf = self.build_gltf(buffer_uri=bin_path.name)

        bin_path.write_bytes(bytes(self.bin))
        gltf_path.write_text(
            json.dumps(gltf, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def save_glb(self, glb_path):
        gltf = self.build_gltf(buffer_uri=None)

        json_bytes = json.dumps(
            gltf,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")

        while len(json_bytes) % 4:
            json_bytes += b" "

        self.align4()
        bin_bytes = bytes(self.bin)

        total_length = 12 + 8 + len(json_bytes) + 8 + len(bin_bytes)

        with glb_path.open("wb") as f:
            f.write(struct.pack("<III", 0x46546C67, 2, total_length))
            f.write(struct.pack("<I4s", len(json_bytes), b"JSON"))
            f.write(json_bytes)
            f.write(struct.pack("<I4s", len(bin_bytes), b"BIN\x00"))
            f.write(bin_bytes)


def triangle_normal(a, b, c):
    ax, ay, az = a
    bx, by, bz = b
    cx, cy, cz = c

    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az

    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx

    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / length, ny / length, nz / length


def compute_vertex_normals(positions, indices):
    normals = [[0.0, 0.0, 0.0] for _ in positions]

    for i in range(0, len(indices), 3):
        ia, ib, ic = indices[i], indices[i + 1], indices[i + 2]

        n = triangle_normal(
            np.array(positions[ia]),
            np.array(positions[ib]),
            np.array(positions[ic]),
        )

        for idx in (ia, ib, ic):
            normals[idx][0] += n[0]
            normals[idx][1] += n[1]
            normals[idx][2] += n[2]

    for normal in normals:
        length = math.sqrt(sum(v * v for v in normal)) or 1.0
        normal[0] /= length
        normal[1] /= length
        normal[2] /= length

    return normals


def build_building_mesh(coords, height, origin_lon, origin_lat, dem=None):
    ring = coords[:-1]
    xy = [mercator_xy(lon, lat, origin_lon, origin_lat) for lon, lat in ring]

    if len(xy) < 3:
        return [], [], []

    vertices_2d = np.array([[x, y] for x, y in xy], dtype=np.float64)
    ring_ends = np.array([len(vertices_2d)], dtype=np.uint32)

    top_tris = triangulate_float64(vertices_2d, ring_ends)
    if len(top_tris) < 3:
        return [], [], []

    positions = []

    base_heights = [
        dem.sample(lon, lat, default=0.0) if dem else 0.0
        for lon, lat in ring
    ]

    for (x, y), base_height in zip(xy, base_heights):
        positions.append([x, base_height, -y])

    for (x, y), base_height in zip(xy, base_heights):
        positions.append([x, base_height + height, -y])

    n = len(xy)
    indices = []

    for i in range(0, len(top_tris), 3):
        a, b, c = [int(v) for v in top_tris[i:i + 3]]

        indices.extend([a + n, b + n, c + n])
        indices.extend([c, b, a])

    for i in range(n):
        j = (i + 1) % n

        indices.extend([i, j, j + n])
        indices.extend([i, j + n, i + n])

    normals = compute_vertex_normals(positions, indices)
    return positions, normals, indices


def build_area_mesh(coords, origin_lon, origin_lat, y=0.01, dem=None, terrain_follow=True):
    ring = coords[:-1]
    xy = [mercator_xy(lon, lat, origin_lon, origin_lat) for lon, lat in ring]

    if len(xy) < 3:
        return [], [], []

    vertices_2d = np.array([[x, z] for x, z in xy], dtype=np.float64)
    ring_ends = np.array([len(vertices_2d)], dtype=np.uint32)

    tris = triangulate_float64(vertices_2d, ring_ends)
    if len(tris) < 3:
        return [], [], []

    if dem and terrain_follow:
        heights = [dem.sample(lon, lat, default=0.0) + y for lon, lat in ring]
    elif dem:
        heights = [dem.sample(lon, lat, default=0.0) + y for lon, lat in ring]
    else:
        heights = [y for _ in ring]

    positions = [[x, height, -z] for (x, z), height in zip(xy, heights)]
    indices = [int(v) for v in tris]
    normals = compute_vertex_normals(positions, indices)
    return positions, normals, indices


def bbox_ring(bbox):
    min_lon, min_lat, max_lon, max_lat = bbox
    return [
        (min_lon, min_lat),
        (max_lon, min_lat),
        (max_lon, max_lat),
        (min_lon, max_lat),
        (min_lon, min_lat),
    ]


def projected_polygon(coords, origin_lon, origin_lat):
    if len(coords) < 4:
        return None

    ring = coords[:-1] if coords[0] == coords[-1] else coords
    if len(ring) < 3:
        return None

    xy = [mercator_xy(lon, lat, origin_lon, origin_lat) for lon, lat in ring]
    polygon = Polygon(xy)
    if polygon.is_empty or polygon.area <= 0:
        return None
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    if polygon.is_empty:
        return None
    return polygon


def projected_bbox_polygon(bbox, origin_lon, origin_lat):
    min_lon, min_lat, max_lon, max_lat = bbox
    x0, y0 = mercator_xy(min_lon, min_lat, origin_lon, origin_lat)
    x1, y1 = mercator_xy(max_lon, max_lat, origin_lon, origin_lat)
    return box(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def projected_coverage_geometry(buildings, areas, roads, origin_lon, origin_lat, road_margin):
    cover_parts = []

    for feature in list(buildings) + list(areas):
        polygon = projected_polygon(feature["coords"], origin_lon, origin_lat)
        if polygon is not None and not polygon.is_empty:
            cover_parts.append(polygon)

    for road in roads:
        coords = road["coords"]
        if len(coords) < 2:
            continue

        xy = [mercator_xy(lon, lat, origin_lon, origin_lat) for lon, lat in coords]
        line = LineString(xy)
        if line.is_empty:
            continue

        half_width = road_width_m(road["tags"]) / 2.0 + max(0.0, road_margin)
        cover_parts.append(line.buffer(half_width, cap_style=2, join_style=2))

    if not cover_parts:
        return GeometryCollection()

    return unary_union(cover_parts)


def iter_polygons(geometry):
    if geometry.is_empty:
        return
    if isinstance(geometry, Polygon):
        yield geometry
    elif isinstance(geometry, MultiPolygon):
        for polygon in geometry.geoms:
            yield polygon
    elif isinstance(geometry, GeometryCollection):
        for child in geometry.geoms:
            yield from iter_polygons(child)


def triangulate_projected_polygon(polygon, origin_lon, origin_lat, dem, z_offset):
    rings = []
    exterior = list(polygon.exterior.coords)[:-1]
    if len(exterior) < 3:
        return [], []

    rings.append(exterior)
    for interior in polygon.interiors:
        hole = list(interior.coords)[:-1]
        if len(hole) >= 3:
            rings.append(hole)

    vertices_2d = []
    ring_ends = []
    for ring in rings:
        vertices_2d.extend(ring)
        ring_ends.append(len(vertices_2d))

    vertices_array = np.array(vertices_2d, dtype=np.float64)
    ring_ends_array = np.array(ring_ends, dtype=np.uint32)
    tris = triangulate_float64(vertices_array, ring_ends_array)
    if len(tris) < 3:
        return [], []

    positions = []
    for x, y in vertices_2d:
        lon, lat = xy_to_lonlat(x, y, origin_lon, origin_lat)
        positions.append([x, dem.sample(lon, lat, default=0.0) + z_offset, -y])

    return positions, [int(v) for v in tris]


def build_precise_soil_fill_mesh(
    bbox,
    origin_lon,
    origin_lat,
    dem,
    buildings,
    areas,
    roads,
    z_offset=0.0,
    road_margin=0.25,
    min_area=0.25,
    simplify=0.0,
):
    bbox_polygon = projected_bbox_polygon(bbox, origin_lon, origin_lat)
    coverage = projected_coverage_geometry(
        buildings,
        areas,
        roads,
        origin_lon,
        origin_lat,
        road_margin,
    )

    soil_geometry = bbox_polygon.difference(coverage)
    if simplify > 0:
        soil_geometry = soil_geometry.simplify(simplify, preserve_topology=True)
    if soil_geometry.is_empty:
        return [], [], [], 0, 0.0

    positions = []
    indices = []
    polygon_count = 0
    soil_area_m2 = 0.0

    for polygon in iter_polygons(soil_geometry):
        if polygon.area < min_area:
            continue

        polygon_positions, polygon_indices = triangulate_projected_polygon(
            polygon,
            origin_lon,
            origin_lat,
            dem,
            z_offset,
        )
        if not polygon_positions or not polygon_indices:
            continue

        base_index = len(positions)
        positions.extend(polygon_positions)
        indices.extend([base_index + index for index in polygon_indices])
        polygon_count += 1
        soil_area_m2 += polygon.area

    if not positions or not indices:
        return [], [], [], polygon_count, soil_area_m2

    normals = compute_vertex_normals(positions, indices)
    return positions, normals, indices, polygon_count, soil_area_m2


def build_terrain_mesh(
    bbox,
    origin_lon,
    origin_lat,
    dem,
    grid_size,
    z_offset=0.0,
    exclusion_polygons=None,
    exclusion_roads=None,
):
    min_lon, min_lat, max_lon, max_lat = bbox
    grid_size = max(2, int(grid_size))
    exclusion_polygons = exclusion_polygons or []
    exclusion_roads = exclusion_roads or []
    positions = []
    indices = []
    vertex_index = {}
    total_cells = grid_size * grid_size
    processed_cells = 0
    next_progress_percent = 5

    print(f"Generating continuous DEM soil terrain mesh: {grid_size} x {grid_size} cells")

    for row in range(grid_size + 1):
        lat = min_lat + (max_lat - min_lat) * row / grid_size
        for col in range(grid_size + 1):
            lon = min_lon + (max_lon - min_lon) * col / grid_size
            x, y = mercator_xy(lon, lat, origin_lon, origin_lat)
            vertex_index[(row, col)] = len(positions)
            positions.append([x, dem.sample(lon, lat, default=0.0) + z_offset, -y])

    for row in range(grid_size):
        for col in range(grid_size):
            processed_cells += 1
            percent = int(processed_cells * 100 / total_cells)
            while percent >= next_progress_percent and next_progress_percent <= 100:
                print(f"DEM soil terrain progress: {next_progress_percent}%")
                next_progress_percent += 5

            lon0 = min_lon + (max_lon - min_lon) * col / grid_size
            lon1 = min_lon + (max_lon - min_lon) * (col + 1) / grid_size
            lat0 = min_lat + (max_lat - min_lat) * row / grid_size
            lat1 = min_lat + (max_lat - min_lat) * (row + 1) / grid_size
            test_points = [
                ((lon0 + lon1) / 2, (lat0 + lat1) / 2),
                (lon0, lat0),
                (lon1, lat0),
                (lon0, lat1),
                (lon1, lat1),
            ]
            excluded = all(
                terrain_cell_excluded(
                    lon,
                    lat,
                    exclusion_polygons,
                    exclusion_roads,
                    origin_lon,
                    origin_lat,
                )
                for lon, lat in test_points
            )

            if excluded:
                continue

            a = vertex_index[(row, col)]
            b = vertex_index[(row, col + 1)]
            c = vertex_index[(row + 1, col)]
            d = vertex_index[(row + 1, col + 1)]
            indices.extend([a, c, b])
            indices.extend([b, c, d])

    if not positions or not indices:
        return [], [], []

    print("DEM soil terrain progress: mesh normals")
    normals = compute_vertex_normals(positions, indices)
    return positions, normals, indices


def build_road_mesh(coords, width, origin_lon, origin_lat, y=0.03, dem=None):
    xy = [mercator_xy(lon, lat, origin_lon, origin_lat) for lon, lat in coords]

    if len(xy) < 2:
        return [], [], []

    half_width = width / 2.0
    positions = []
    indices = []

    for i, (x, z) in enumerate(xy):
        if i == 0:
            dx = xy[1][0] - x
            dz = xy[1][1] - z
        elif i == len(xy) - 1:
            dx = x - xy[i - 1][0]
            dz = z - xy[i - 1][1]
        else:
            dx = xy[i + 1][0] - xy[i - 1][0]
            dz = xy[i + 1][1] - xy[i - 1][1]

        length = math.sqrt(dx * dx + dz * dz) or 1.0
        nx = -dz / length
        nz = dx / length
        height = dem.sample(coords[i][0], coords[i][1], default=0.0) if dem else 0.0

        positions.append([x + nx * half_width, height + y, -(z + nz * half_width)])
        positions.append([x - nx * half_width, height + y, -(z - nz * half_width)])

    for i in range(len(xy) - 1):
        left_a = i * 2
        right_a = left_a + 1
        left_b = left_a + 2
        right_b = left_a + 3
        indices.extend([left_a, right_a, left_b])
        indices.extend([right_a, right_b, left_b])

    normals = [[0.0, 1.0, 0.0] for _ in positions]
    return positions, normals, indices


def make_building_metadata(building, name, height, material_info):
    tags = building["tags"]
    electrical_props = material_electrical_properties(material_info["inferred_material"])

    return {
        "osm_type": "way",
        "osm_id": building["id"],
        "name": name,
        "building": tags.get("building"),
        "height_m": height,
        "height_tag": tags.get("height"),
        "building_levels": tags.get("building:levels"),
        "osm_material": material_info["osm_material"],
        "inferred_material": material_info["inferred_material"],
        "material_inference_source": material_info["material_inference_source"],
        "dielectric_constant": electrical_props["dielectric_constant"],
        "conductivity_s_per_m": electrical_props["conductivity_s_per_m"],
        "source_tags": tags,
    }


def make_surface_metadata(feature, name, feature_type):
    tags = feature["tags"]
    inferred_material, source = infer_surface_material(feature_type, tags)
    electrical_props = surface_electrical_properties(inferred_material)

    return {
        "osm_type": feature.get("osm_type", "way"),
        "osm_id": feature["id"],
        "name": name,
        "feature_type": feature_type,
        "inferred_material": inferred_material,
        "material_inference_source": source,
        "dielectric_constant": electrical_props["dielectric_constant"],
        "conductivity_s_per_m": electrical_props["conductivity_s_per_m"],
        "source_tags": tags,
    }


def normalize_output_path(output_arg, output_format):
    output_path = Path(output_arg)

    if output_path.suffix.lower() in (".glb", ".gltf"):
        return output_path

    return output_path.with_suffix("." + output_format)


def main():
    parser = argparse.ArgumentParser(
        description="Convert OSM PBF buildings, roads, and surface areas inside a bbox to glTF/GLB."
    )

    parser.add_argument("pbf", help="Input .osm.pbf file")
    parser.add_argument("output", help="Output file path. Extension can be omitted.")

    parser.add_argument(
        "--format",
        choices=("glb", "gltf"),
        default="glb",
        help="Output 3D format if output has no extension. Default: glb",
    )

    parser.add_argument(
        "--bbox",
        nargs=4,
        type=float,
        required=True,
        metavar=("MIN_LAT", "MIN_LON", "MAX_LAT", "MAX_LON"),
        help="Bounding box format: minLat minLon maxLat maxLon",
    )

    parser.add_argument(
        "--material-mode",
        choices=("actual", "infer"),
        default="actual",
        help="actual: only use OSM material tags; infer: guess material when OSM has no material",
    )

    parser.add_argument(
        "--dem",
        default=None,
        help="Existing DEM GeoTIFF path. If provided, terrain and features use DEM elevation.",
    )

    parser.add_argument(
        "--download-dem",
        action="store_true",
        help="Download a clipped DEM GeoTIFF for the input bbox from OpenTopography.",
    )

    parser.add_argument(
        "--dem-type",
        default="COP30",
        help="OpenTopography DEM type when --download-dem is used. Default: COP30",
    )

    parser.add_argument(
        "--opentopo-key",
        default=os.environ.get("OPENTOPO_API_KEY"),
        help="OpenTopography API key. Can also be set with OPENTOPO_API_KEY.",
    )

    parser.add_argument(
        "--terrain-grid",
        type=int,
        default=160,
        help="DEM terrain grid resolution per side. Default: 160",
    )

    parser.add_argument(
        "--terrain-offset",
        type=float,
        default=0.0,
        help="Vertical offset in meters applied only to the generated soil fill terrain mesh. Default: 0.0, matching the DEM elevation used by OSM features.",
    )

    parser.add_argument(
        "--generate-dem-terrain",
        action="store_true",
        help="Generate the DEM soil fill terrain. Kept for compatibility; DEM soil fill is generated by default.",
    )

    parser.add_argument(
        "--no-soil-fill",
        action="store_true",
        help="Disable the DEM soil fill layer. If used, black/background areas may remain visible.",
    )

    parser.add_argument(
        "--soil-fill-mode",
        choices=("precise", "underlay"),
        default="precise",
        help="precise: fill only bbox minus buildings, roads, water, vegetation, and other areas using Shapely; underlay: continuous DEM soil surface under all OSM features.",
    )

    parser.add_argument(
        "--soil-road-margin",
        type=float,
        default=0.25,
        help="Extra road buffer margin in meters for precise soil fill clipping. Default: 0.25",
    )

    parser.add_argument(
        "--soil-min-area",
        type=float,
        default=0.25,
        help="Drop precise soil polygons smaller than this area in square meters. Default: 0.25",
    )

    parser.add_argument(
        "--soil-simplify",
        type=float,
        default=0.0,
        help="Simplify precise soil polygons by this tolerance in meters before triangulation. Default: 0.0",
    )

    args = parser.parse_args()

    output_path = normalize_output_path(args.output, args.format)

    min_lat, min_lon, max_lat, max_lon = args.bbox
    min_lon, max_lon = sorted((min_lon, max_lon))
    min_lat, max_lat = sorted((min_lat, max_lat))

    bbox = (min_lon, min_lat, max_lon, max_lat)

    if not (-180 <= min_lon <= 180 and -180 <= max_lon <= 180):
        raise ValueError("Longitude must be between -180 and 180.")
    if not (-90 <= min_lat <= 90 and -90 <= max_lat <= 90):
        raise ValueError("Latitude must be between -90 and 90.")

    origin_lon = (min_lon + max_lon) / 2
    origin_lat = (min_lat + max_lat) / 2

    print(f"Using bbox input format: minLat minLon maxLat maxLon")
    print(f"Using bbox lon/lat: {min_lon} {min_lat} {max_lon} {max_lat}")
    print(f"Output 3D file: {output_path}")
    print(f"Material mode: {args.material_mode}")

    dem_path = Path(args.dem) if args.dem else None
    if args.download_dem:
        dem_path = output_path.with_name(output_path.stem + f"_{args.dem_type.lower()}_dem.tif")
        download_opentopography_dem(
            bbox=bbox,
            dem_type=args.dem_type,
            output_path=dem_path,
            api_key=args.opentopo_key,
        )

    dem = DemSampler(dem_path) if dem_path else None
    if dem:
        print(f"Using DEM GeoTIFF: {dem_path}")

    generate_soil_fill = bool(dem and (args.generate_dem_terrain or not args.no_soil_fill))
    precise_soil_fill = bool(generate_soil_fill and args.soil_fill_mode == "precise")

    print("Counting OSM ways/relations for progress...")
    progress_counter = OsmProgressCounter()
    progress_counter.apply_file(args.pbf, locations=False)

    print("Reading OSM map data...")
    collector = OsmCollector(bbox, total_items=progress_counter.total_items)
    collector.apply_file(args.pbf, locations=True)
    if collector.next_progress_percent <= 100:
        print("OSM read progress: 100%")

    writer = GltfWriter()
    material_indices = {}
    exported_building_count = 0
    exported_road_count = 0
    exported_area_count = 0
    extras_items = []

    for feature_type in ("road", "water", "grass", "forest", "sand", "land"):
        material_indices[feature_type] = writer.add_material(
            feature_type,
            feature_color(feature_type),
        )

    ground_name = "dem_precise_soil_fill" if precise_soil_fill else ("dem_soil_fill" if generate_soil_fill else "bbox_ground")
    ground_feature = {
        "osm_type": "generated",
        "id": ground_name,
        "coords": bbox_ring(bbox),
        "tags": {"generated": ground_name},
    }
    soil_fill_polygon_count = None
    soil_fill_area_m2 = None
    if precise_soil_fill:
        print("Generating precise DEM soil fill from bbox minus buildings, roads, water, vegetation, and other mapped areas")
        positions, normals, indices, soil_fill_polygon_count, soil_fill_area_m2 = build_precise_soil_fill_mesh(
            bbox,
            origin_lon,
            origin_lat,
            dem,
            collector.buildings,
            collector.areas,
            collector.roads,
            z_offset=args.terrain_offset,
            road_margin=args.soil_road_margin,
            min_area=args.soil_min_area,
            simplify=args.soil_simplify,
        )
    elif generate_soil_fill:
        print("Generating unclipped DEM soil surface at the same elevation as OSM features")
        positions, normals, indices = build_terrain_mesh(
            bbox,
            origin_lon,
            origin_lat,
            dem,
            args.terrain_grid,
            z_offset=args.terrain_offset,
        )
    elif not dem:
        positions, normals, indices = build_area_mesh(
            ground_feature["coords"],
            origin_lon,
            origin_lat,
            y=-0.02,
        )
    else:
        positions, normals, indices = [], [], []

    if positions and indices:
        metadata = make_surface_metadata(ground_feature, ground_name, "land")
        if generate_soil_fill:
            metadata["terrain_source"] = "dem"
            metadata["dem_path"] = str(dem_path)
            metadata["terrain_grid"] = args.terrain_grid
            metadata["terrain_offset_m"] = args.terrain_offset
            metadata["soil_fill_mode"] = args.soil_fill_mode
            if precise_soil_fill:
                metadata["terrain_clipping"] = "precise_bbox_difference"
                metadata["fill_strategy"] = "bbox_minus_buildings_roads_water_vegetation_and_mapped_surface_areas"
                metadata["soil_polygon_count"] = soil_fill_polygon_count
                metadata["soil_area_m2"] = soil_fill_area_m2
                metadata["soil_road_margin_m"] = args.soil_road_margin
                metadata["soil_min_area_m2"] = args.soil_min_area
                metadata["soil_simplify_m"] = args.soil_simplify
                metadata["height_rule"] = "soil uses DEM elevation only in uncovered background spaces"
            else:
                metadata["terrain_clipping"] = "none_unclipped_soil_fill"
                metadata["fill_strategy"] = "continuous_dem_soil_surface_under_osm_features"
                metadata["height_rule"] = "soil uses DEM elevation; OSM features use the same DEM elevation plus their small display offset"
        metadata["gltf_node_index"] = len(writer.nodes)
        metadata["vertex_count"] = len(positions)
        metadata["triangle_count"] = len(indices) // 3

        writer.add_mesh_node(
            name=ground_name,
            positions=positions,
            normals=normals,
            indices=indices,
            material_index=material_indices["land"],
            extras=metadata,
        )
        extras_items.append(metadata)

    for building in collector.buildings:
        tags = building["tags"]

        material_info = resolve_building_material(tags, args.material_mode)
        inferred_material = material_info["inferred_material"]

        if inferred_material not in material_indices:
            material_indices[inferred_material] = writer.add_material(
                f"building_{inferred_material}",
                material_color(inferred_material),
            )

        height = building_height(tags)
        name = clean_text(building_name(tags, building["id"]))

        positions, normals, indices = build_building_mesh(
            building["coords"],
            height,
            origin_lon,
            origin_lat,
            dem=dem,
        )

        if not positions or not indices:
            continue

        metadata = make_building_metadata(
            building=building,
            name=name,
            height=height,
            material_info=material_info,
        )

        metadata["gltf_node_index"] = len(writer.nodes)
        metadata["vertex_count"] = len(positions)
        metadata["triangle_count"] = len(indices) // 3

        writer.add_mesh_node(
            name=name,
            positions=positions,
            normals=normals,
            indices=indices,
            material_index=material_indices[inferred_material],
            extras=metadata,
        )

        extras_items.append(metadata)
        exported_building_count += 1

    for road in collector.roads:
        tags = road["tags"]
        name = feature_name(tags, road["id"], "road")
        width = road_width_m(tags)

        positions, normals, indices = build_road_mesh(
            road["coords"],
            width,
            origin_lon,
            origin_lat,
            dem=dem,
        )

        if not positions or not indices:
            continue

        metadata = make_surface_metadata(road, name, "road")
        metadata["highway"] = tags.get("highway")
        metadata["width_m"] = width
        metadata["gltf_node_index"] = len(writer.nodes)
        metadata["vertex_count"] = len(positions)
        metadata["triangle_count"] = len(indices) // 3

        writer.add_mesh_node(
            name=name,
            positions=positions,
            normals=normals,
            indices=indices,
            material_index=material_indices["road"],
            extras=metadata,
        )

        extras_items.append(metadata)
        exported_road_count += 1

    for area in collector.areas:
        feature_type = area["feature_type"]
        name = feature_name(area["tags"], area["id"], feature_type)

        positions, normals, indices = build_area_mesh(
            area["coords"],
            origin_lon,
            origin_lat,
            dem=dem,
            terrain_follow=True,
        )

        if not positions or not indices:
            continue

        metadata = make_surface_metadata(area, name, feature_type)
        metadata["gltf_node_index"] = len(writer.nodes)
        metadata["vertex_count"] = len(positions)
        metadata["triangle_count"] = len(indices) // 3

        writer.add_mesh_node(
            name=name,
            positions=positions,
            normals=normals,
            indices=indices,
            material_index=material_indices.get(feature_type, material_indices["land"]),
            extras=metadata,
        )

        extras_items.append(metadata)
        exported_area_count += 1

    writer.save(output_path)

    extras_path = output_path.with_name(output_path.stem + "_extras.json")
    extras_doc = {
        "input_pbf": args.pbf,
        "output_3d": str(output_path),
        "output_format": output_path.suffix.lower().lstrip("."),
        "material_mode": args.material_mode,
        "dem_enabled": dem is not None,
        "dem_path": str(dem_path) if dem_path else None,
        "dem_type": args.dem_type if args.download_dem else None,
        "dem_terrain_generated": generate_soil_fill,
        "remaining_ground_generated": bool(generate_soil_fill or not dem),
        "remaining_ground_material": "soil",
        "unmodeled_background_material": "soil" if dem and not generate_soil_fill else None,
        "unmodeled_background_note": "No soil fill mesh was generated because --no-soil-fill was used; remaining black/background areas should be treated as inferred soil." if dem and not generate_soil_fill else None,
        "soil_fill_mode": args.soil_fill_mode if generate_soil_fill else None,
        "soil_fill_polygon_count": soil_fill_polygon_count if precise_soil_fill else None,
        "soil_fill_area_m2": soil_fill_area_m2 if precise_soil_fill else None,
        "soil_road_margin_m": args.soil_road_margin if precise_soil_fill else None,
        "soil_min_area_m2": args.soil_min_area if precise_soil_fill else None,
        "soil_simplify_m": args.soil_simplify if precise_soil_fill else None,
        "terrain_grid": args.terrain_grid if generate_soil_fill else None,
        "terrain_offset_m": args.terrain_offset if generate_soil_fill else None,
        "terrain_clipping": ("precise_bbox_difference" if precise_soil_fill else "none_unclipped_soil_fill") if generate_soil_fill else None,
        "soil_fill_strategy": ("bbox_minus_buildings_roads_water_vegetation_and_mapped_surface_areas" if precise_soil_fill else "continuous_dem_soil_surface_under_osm_features") if generate_soil_fill else None,
        "height_rule": ("soil uses DEM elevation only in uncovered background spaces" if precise_soil_fill else "soil uses DEM elevation; buildings, roads, water, grass, forest, and sand use the same DEM elevation baseline") if generate_soil_fill else None,
        "bbox_format": "minLat minLon maxLat maxLon",
        "bbox_input": args.bbox,
        "bbox_lonlat": {
            "min_lon": min_lon,
            "min_lat": min_lat,
            "max_lon": max_lon,
            "max_lat": max_lat,
        },
        "extras_count": len(extras_items),
        "extras": extras_items,
    }
    extras_path.write_text(
        json.dumps(extras_doc, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(f"Saved 3D file: {output_path}")
    print(f"Saved extras JSON: {extras_path}")
    print(f"Buildings read from OSM: {len(collector.buildings)}")
    print(f"Buildings exported to mesh: {exported_building_count}")
    print(f"Roads read from OSM: {len(collector.roads)}")
    print(f"Roads exported to mesh: {exported_road_count}")
    print(f"Surface areas read from OSM: {len(collector.areas)}")
    print(f"Surface areas exported to mesh: {exported_area_count}")

    if dem:
        dem.close()

    if output_path.suffix.lower() == ".gltf":
        print("Note: keep the generated .bin file next to the .gltf file.")


if __name__ == "__main__":
    main()


