#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mapbox/earcut.hpp>
#include <nlohmann/json.hpp>
#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

using json = nlohmann::json;

constexpr int ARRAY_BUFFER = 34962;
constexpr int ELEMENT_ARRAY_BUFFER = 34963;
constexpr int FLOAT = 5126;
constexpr int UNSIGNED_INT = 5125;
constexpr int TRIANGLES = 4;
constexpr double PI = 3.14159265358979323846;

struct LonLat {
    double lon = 0.0;
    double lat = 0.0;
};

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

namespace mapbox {
namespace util {
template <>
struct nth<0, Point2> {
    static double get(const Point2& point) {
        return point.x;
    }
};

template <>
struct nth<1, Point2> {
    static double get(const Point2& point) {
        return point.y;
    }
};
}  // namespace util
}  // namespace mapbox

struct BBox {
    double min_lon = 0.0;
    double min_lat = 0.0;
    double max_lon = 0.0;
    double max_lat = 0.0;
};

struct Feature {
    std::string osm_type = "way";
    std::string id;
    std::vector<LonLat> coords;
    std::map<std::string, std::string> tags;
    std::string feature_type;
};

struct MeshData {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<uint32_t> indices;
};

struct Args {
    std::string pbf;
    std::string output;
    std::string format = "glb";
    std::string material_mode = "actual";
    std::array<double, 4> bbox_input{};
};

std::string clean_text(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), '\0'), value.end());
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::optional<double> parse_meters(const std::string& value) {
    if (value.empty()) {
        return std::nullopt;
    }

    std::string text = lower_copy(value);
    std::replace(text.begin(), text.end(), ',', '.');

    std::smatch match;
    if (!std::regex_search(text, match, std::regex("-?\\d+(\\.\\d+)?"))) {
        return std::nullopt;
    }

    return std::stod(match.str());
}

std::string tag_value(const std::map<std::string, std::string>& tags, const std::string& key) {
    const auto it = tags.find(key);
    return it == tags.end() ? "" : it->second;
}

bool has_tag(const std::map<std::string, std::string>& tags, const std::string& key) {
    return tags.find(key) != tags.end();
}

json tags_to_json(const std::map<std::string, std::string>& tags) {
    json out = json::object();
    for (const auto& [key, value] : tags) {
        out[key] = value;
    }
    return out;
}

double building_height(const std::map<std::string, std::string>& tags) {
    for (const auto& key : {"height", "building:height", "roof:height"}) {
        auto height = parse_meters(tag_value(tags, key));
        if (height && *height > 0.0) {
            return *height;
        }
    }

    auto levels = parse_meters(tag_value(tags, "building:levels"));
    if (!levels) {
        levels = parse_meters(tag_value(tags, "levels"));
    }
    if (levels && *levels > 0.0) {
        return *levels * 3.2;
    }

    return 8.0;
}

std::string building_name(const std::map<std::string, std::string>& tags, const std::string& osm_id) {
    for (const auto& key : {"name", "building:name", "addr:housename"}) {
        const auto value = tag_value(tags, key);
        if (!value.empty()) {
            return value;
        }
    }
    return "building/" + osm_id;
}

std::string feature_name(const std::map<std::string, std::string>& tags,
                         const std::string& osm_id,
                         const std::string& feature_type) {
    const auto value = tag_value(tags, "name");
    return clean_text(value.empty() ? feature_type + "/" + osm_id : value);
}

std::pair<std::string, std::string> actual_building_material(const std::map<std::string, std::string>& tags) {
    for (const auto& key : {"building:material", "facade:material", "material",
                            "wall:material", "cladding", "roof:material"}) {
        const auto value = tag_value(tags, key);
        if (!value.empty()) {
            return {lower_copy(clean_text(value)), key};
        }
    }
    return {"unknown", ""};
}

std::pair<std::string, std::string> infer_building_material(const std::map<std::string, std::string>& tags) {
    const auto building_type = lower_copy(tag_value(tags, "building"));
    const auto amenity = lower_copy(tag_value(tags, "amenity"));
    const auto name = lower_copy(tag_value(tags, "name"));

    const std::map<std::string, std::string> inferred = {
        {"apartments", "concrete"}, {"residential", "concrete"}, {"house", "brick"},
        {"detached", "brick"}, {"terrace", "brick"}, {"commercial", "glass"},
        {"retail", "glass"}, {"office", "glass"}, {"industrial", "metal"},
        {"warehouse", "metal"}, {"garage", "concrete"}, {"school", "brick"},
        {"university", "brick"}, {"hospital", "concrete"}, {"hotel", "concrete"},
    };

    const auto it = inferred.find(building_type);
    if (it != inferred.end()) {
        return {it->second, "building:" + building_type};
    }
    if (amenity == "fire_station") {
        return {"brick", "amenity:fire_station"};
    }
    return {"brick", "default_brick"};
}

json resolve_building_material(const std::map<std::string, std::string>& tags,
                               const std::string& material_mode) {
    const auto [actual, source_key] = actual_building_material(tags);
    if (actual != "unknown") {
        return {
            {"osm_material", actual},
            {"inferred_material", actual},
            {"material_inference_source", "osm:" + source_key},
        };
    }

    if (material_mode == "infer") {
        const auto [inferred, source] = infer_building_material(tags);
        return {
            {"osm_material", nullptr},
            {"inferred_material", inferred},
            {"material_inference_source", source},
        };
    }

    return {
        {"osm_material", nullptr},
        {"inferred_material", "brick"},
        {"material_inference_source", "default_brick"},
    };
}

json unset_electrical_properties() {
    return {
        {"dielectric_constant", nullptr},
        {"conductivity_s_per_m", nullptr},
    };
}

std::pair<std::string, std::string> infer_surface_material(const std::string& feature_type,
                                                           const std::map<std::string, std::string>& tags) {
    if (feature_type == "road") {
        return {"asphalt", "highway:" + tag_value(tags, "highway")};
    }
    if (feature_type == "water") {
        return {"fresh_water", "natural:water"};
    }
    if (feature_type == "grass") {
        return {"grass", "surface:grass"};
    }
    if (feature_type == "forest") {
        return {"forest", "surface:forest"};
    }
    if (feature_type == "sand") {
        return {"sand", "surface:sand"};
    }
    return {"soil", "default_soil"};
}

std::array<float, 4> material_color(const std::string& material) {
    const auto key = lower_copy(material);
    const std::map<std::string, std::array<float, 4>> colors = {
        {"brick", {0.62f, 0.22f, 0.14f, 1.0f}},
        {"bricks", {0.62f, 0.22f, 0.14f, 1.0f}},
        {"concrete", {0.58f, 0.58f, 0.55f, 1.0f}},
        {"cement", {0.55f, 0.55f, 0.52f, 1.0f}},
        {"glass", {0.45f, 0.72f, 0.90f, 0.55f}},
        {"steel", {0.48f, 0.50f, 0.52f, 1.0f}},
        {"metal", {0.50f, 0.50f, 0.50f, 1.0f}},
        {"wood", {0.55f, 0.34f, 0.18f, 1.0f}},
        {"stone", {0.50f, 0.48f, 0.42f, 1.0f}},
        {"plaster", {0.78f, 0.74f, 0.66f, 1.0f}},
        {"unknown", {0.72f, 0.70f, 0.64f, 1.0f}},
    };
    const auto it = colors.find(key);
    return it == colors.end() ? colors.at("unknown") : it->second;
}

std::array<float, 4> feature_color(const std::string& feature_type) {
    const std::map<std::string, std::array<float, 4>> colors = {
        {"road", {0.44f, 0.46f, 0.48f, 1.0f}},
        {"water", {0.02f, 0.32f, 0.90f, 1.0f}},
        {"grass", {0.20f, 0.62f, 0.24f, 1.0f}},
        {"forest", {0.05f, 0.36f, 0.12f, 1.0f}},
        {"sand", {0.70f, 0.64f, 0.44f, 1.0f}},
        {"land", {0.52f, 0.38f, 0.24f, 1.0f}},
    };
    const auto it = colors.find(feature_type);
    return it == colors.end() ? colors.at("land") : it->second;
}

bool is_water_feature(const std::map<std::string, std::string>& tags) {
    const auto natural = tag_value(tags, "natural");
    const auto water = tag_value(tags, "water");
    return natural == "water" || natural == "bay" || water == "lake" || water == "pond" ||
           water == "reservoir" || water == "basin" || tag_value(tags, "landuse") == "reservoir" ||
           tag_value(tags, "waterway") == "riverbank";
}

std::string landcover_type(const std::map<std::string, std::string>& tags) {
    const auto natural = tag_value(tags, "natural");
    const auto landuse = tag_value(tags, "landuse");
    const auto leisure = tag_value(tags, "leisure");

    if (natural == "wood" || natural == "scrub" || landuse == "forest") {
        return "forest";
    }
    if (natural == "grassland" || natural == "heath" || landuse == "grass" || landuse == "meadow") {
        return "grass";
    }
    if (natural == "beach" || natural == "sand") {
        return "sand";
    }
    if (leisure == "park" || leisure == "garden" || leisure == "pitch") {
        return "grass";
    }
    return "";
}

double road_width_m(const std::map<std::string, std::string>& tags) {
    const auto width = parse_meters(tag_value(tags, "width"));
    if (width && *width > 0.0) {
        return *width;
    }

    const std::map<std::string, double> defaults = {
        {"motorway", 14.0}, {"trunk", 12.0}, {"primary", 10.0}, {"secondary", 8.0},
        {"tertiary", 7.0}, {"residential", 5.5}, {"service", 3.5},
        {"footway", 2.0}, {"path", 1.5}, {"cycleway", 2.0},
    };

    const auto it = defaults.find(tag_value(tags, "highway"));
    return it == defaults.end() ? 4.0 : it->second;
}

Point2 mercator_xy(double lon, double lat, double origin_lon, double origin_lat) {
    constexpr double radius = 6378137.0;
    const double x = (lon - origin_lon) * PI / 180.0 * radius * std::cos(origin_lat * PI / 180.0);
    const double y = (lat - origin_lat) * PI / 180.0 * radius;
    return {x, y};
}

bool point_in_bbox(const LonLat& point, const BBox& bbox) {
    return bbox.min_lon <= point.lon && point.lon <= bbox.max_lon &&
           bbox.min_lat <= point.lat && point.lat <= bbox.max_lat;
}

bool any_point_in_bbox(const std::vector<LonLat>& coords, const BBox& bbox) {
    return std::any_of(coords.begin(), coords.end(), [&](const LonLat& point) {
        return point_in_bbox(point, bbox);
    });
}

std::vector<LonLat> bbox_ring(const BBox& bbox) {
    return {
        {bbox.min_lon, bbox.min_lat},
        {bbox.max_lon, bbox.min_lat},
        {bbox.max_lon, bbox.max_lat},
        {bbox.min_lon, bbox.max_lat},
        {bbox.min_lon, bbox.min_lat},
    };
}

bool same_lonlat(const LonLat& a, const LonLat& b) {
    return a.lon == b.lon && a.lat == b.lat;
}

std::vector<std::vector<LonLat>> join_way_segments(const std::vector<std::vector<LonLat>>& segments) {
    std::vector<std::vector<LonLat>> remaining;
    for (const auto& segment : segments) {
        if (segment.size() >= 2) {
            remaining.push_back(segment);
        }
    }

    std::vector<std::vector<LonLat>> rings;
    while (!remaining.empty()) {
        auto ring = remaining.front();
        remaining.erase(remaining.begin());

        bool changed = true;
        while (changed && !same_lonlat(ring.front(), ring.back())) {
            changed = false;
            for (auto it = remaining.begin(); it != remaining.end(); ++it) {
                auto segment = *it;
                if (same_lonlat(ring.back(), segment.front())) {
                    ring.insert(ring.end(), segment.begin() + 1, segment.end());
                } else if (same_lonlat(ring.back(), segment.back())) {
                    std::reverse(segment.begin(), segment.end());
                    ring.insert(ring.end(), segment.begin() + 1, segment.end());
                } else if (same_lonlat(ring.front(), segment.back())) {
                    ring.insert(ring.begin(), segment.begin(), segment.end() - 1);
                } else if (same_lonlat(ring.front(), segment.front())) {
                    std::reverse(segment.begin(), segment.end());
                    ring.insert(ring.begin(), segment.begin(), segment.end() - 1);
                } else {
                    continue;
                }
                remaining.erase(it);
                changed = true;
                break;
            }
        }

        if (ring.size() >= 4 && same_lonlat(ring.front(), ring.back())) {
            rings.push_back(ring);
        }
    }

    return rings;
}

class GltfWriter {
public:
    std::vector<uint8_t> bin;
    json buffer_views = json::array();
    json accessors = json::array();
    json meshes = json::array();
    json nodes = json::array();
    json materials = json::array();

    void align4(uint8_t pad = 0) {
        while (bin.size() % 4 != 0) {
            bin.push_back(pad);
        }
    }

    template <typename T>
    int add_bytes(const std::vector<T>& values, int target) {
        align4();
        const auto offset = bin.size();
        const auto byte_length = values.size() * sizeof(T);
        const auto* ptr = reinterpret_cast<const uint8_t*>(values.data());
        bin.insert(bin.end(), ptr, ptr + byte_length);

        const int view_index = static_cast<int>(buffer_views.size());
        buffer_views.push_back({
            {"buffer", 0},
            {"byteOffset", offset},
            {"byteLength", byte_length},
            {"target", target},
        });
        return view_index;
    }

    int add_positions(const std::vector<Vec3>& positions) {
        const int view = add_bytes(positions, ARRAY_BUFFER);

        std::array<float, 3> minv = {positions[0].x, positions[0].y, positions[0].z};
        std::array<float, 3> maxv = minv;
        for (const auto& p : positions) {
            minv = {std::min(minv[0], p.x), std::min(minv[1], p.y), std::min(minv[2], p.z)};
            maxv = {std::max(maxv[0], p.x), std::max(maxv[1], p.y), std::max(maxv[2], p.z)};
        }

        accessors.push_back({
            {"bufferView", view},
            {"byteOffset", 0},
            {"componentType", FLOAT},
            {"count", positions.size()},
            {"type", "VEC3"},
            {"min", minv},
            {"max", maxv},
        });
        return static_cast<int>(accessors.size()) - 1;
    }

    int add_normals(const std::vector<Vec3>& normals) {
        const int view = add_bytes(normals, ARRAY_BUFFER);
        accessors.push_back({
            {"bufferView", view},
            {"byteOffset", 0},
            {"componentType", FLOAT},
            {"count", normals.size()},
            {"type", "VEC3"},
        });
        return static_cast<int>(accessors.size()) - 1;
    }

    int add_indices(const std::vector<uint32_t>& indices) {
        const int view = add_bytes(indices, ELEMENT_ARRAY_BUFFER);
        const auto [min_it, max_it] = std::minmax_element(indices.begin(), indices.end());
        accessors.push_back({
            {"bufferView", view},
            {"byteOffset", 0},
            {"componentType", UNSIGNED_INT},
            {"count", indices.size()},
            {"type", "SCALAR"},
            {"min", json::array({*min_it})},
            {"max", json::array({*max_it})},
        });
        return static_cast<int>(accessors.size()) - 1;
    }

    int add_material(const std::string& name, const std::array<float, 4>& color) {
        json material = {
            {"name", name},
            {"pbrMetallicRoughness", {
                {"baseColorFactor", color},
                {"metallicFactor", 0.0},
                {"roughnessFactor", 0.85},
            }},
            {"doubleSided", true},
        };
        if (color[3] < 1.0f) {
            material["alphaMode"] = "BLEND";
        }
        materials.push_back(material);
        return static_cast<int>(materials.size()) - 1;
    }

    void add_mesh_node(const std::string& name,
                       const MeshData& mesh,
                       int material_index,
                       const json& extras) {
        const int pos_accessor = add_positions(mesh.positions);
        const int normal_accessor = add_normals(mesh.normals);
        const int idx_accessor = add_indices(mesh.indices);

        const int mesh_index = static_cast<int>(meshes.size());
        meshes.push_back({
            {"name", name},
            {"primitives", json::array({
                {
                    {"attributes", {{"POSITION", pos_accessor}, {"NORMAL", normal_accessor}}},
                    {"indices", idx_accessor},
                    {"material", material_index},
                    {"mode", TRIANGLES},
                }
            })},
        });

        nodes.push_back({
            {"name", name},
            {"mesh", mesh_index},
            {"extras", extras},
        });
    }

    json build_gltf(const std::string& buffer_uri = "") {
        align4();
        json buffer = {{"byteLength", bin.size()}};
        if (!buffer_uri.empty()) {
            buffer["uri"] = buffer_uri;
        }

        json scene_nodes = json::array();
        for (size_t i = 0; i < nodes.size(); ++i) {
            scene_nodes.push_back(i);
        }

        return {
            {"asset", {{"version", "2.0"}, {"generator", "osm_pbf_to_scene_gltf.cpp"}}},
            {"scene", 0},
            {"scenes", json::array({{{"nodes", scene_nodes}}})},
            {"nodes", nodes},
            {"meshes", meshes},
            {"materials", materials},
            {"buffers", json::array({buffer})},
            {"bufferViews", buffer_views},
            {"accessors", accessors},
        };
    }

    void save_gltf(const std::string& gltf_path) {
        const auto dot = gltf_path.find_last_of('.');
        const std::string bin_path = (dot == std::string::npos ? gltf_path : gltf_path.substr(0, dot)) + ".bin";
        const auto slash = bin_path.find_last_of("/\\");
        const std::string bin_name = slash == std::string::npos ? bin_path : bin_path.substr(slash + 1);

        auto gltf = build_gltf(bin_name);
        std::ofstream bout(bin_path, std::ios::binary);
        bout.write(reinterpret_cast<const char*>(bin.data()), static_cast<std::streamsize>(bin.size()));

        std::ofstream gout(gltf_path);
        gout << std::setw(2) << gltf << "\n";
    }

    void save_glb(const std::string& glb_path) {
        auto gltf = build_gltf();
        std::string json_text = gltf.dump();
        while (json_text.size() % 4 != 0) {
            json_text.push_back(' ');
        }

        align4();
        const uint32_t total_length = static_cast<uint32_t>(12 + 8 + json_text.size() + 8 + bin.size());

        std::ofstream out(glb_path, std::ios::binary);
        const uint32_t magic = 0x46546C67;
        const uint32_t version = 2;
        const uint32_t json_length = static_cast<uint32_t>(json_text.size());
        const uint32_t bin_length = static_cast<uint32_t>(bin.size());
        const uint32_t json_type = 0x4E4F534A;
        const uint32_t bin_type = 0x004E4942;

        out.write(reinterpret_cast<const char*>(&magic), 4);
        out.write(reinterpret_cast<const char*>(&version), 4);
        out.write(reinterpret_cast<const char*>(&total_length), 4);
        out.write(reinterpret_cast<const char*>(&json_length), 4);
        out.write(reinterpret_cast<const char*>(&json_type), 4);
        out.write(json_text.data(), static_cast<std::streamsize>(json_text.size()));
        out.write(reinterpret_cast<const char*>(&bin_length), 4);
        out.write(reinterpret_cast<const char*>(&bin_type), 4);
        out.write(reinterpret_cast<const char*>(bin.data()), static_cast<std::streamsize>(bin.size()));
    }

    void save(const std::string& output_path) {
        if (output_path.size() >= 4 && output_path.substr(output_path.size() - 4) == ".glb") {
            save_glb(output_path);
        } else if (output_path.size() >= 5 && output_path.substr(output_path.size() - 5) == ".gltf") {
            save_gltf(output_path);
        } else {
            throw std::runtime_error("Output file must end with .glb or .gltf");
        }
    }
};

Vec3 triangle_normal(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double ux = b.x - a.x;
    const double uy = b.y - a.y;
    const double uz = b.z - a.z;
    const double vx = c.x - a.x;
    const double vy = c.y - a.y;
    const double vz = c.z - a.z;

    double nx = uy * vz - uz * vy;
    double ny = uz * vx - ux * vz;
    double nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len == 0.0) {
        return {0.0f, 1.0f, 0.0f};
    }
    return {static_cast<float>(nx / len), static_cast<float>(ny / len), static_cast<float>(nz / len)};
}

std::vector<Vec3> compute_vertex_normals(const std::vector<Vec3>& positions,
                                         const std::vector<uint32_t>& indices) {
    std::vector<Vec3> normals(positions.size());
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const auto ia = indices[i];
        const auto ib = indices[i + 1];
        const auto ic = indices[i + 2];
        const auto n = triangle_normal(positions[ia], positions[ib], positions[ic]);
        for (const auto idx : {ia, ib, ic}) {
            normals[idx].x += n.x;
            normals[idx].y += n.y;
            normals[idx].z += n.z;
        }
    }

    for (auto& n : normals) {
        const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (len != 0.0) {
            n.x = static_cast<float>(n.x / len);
            n.y = static_cast<float>(n.y / len);
            n.z = static_cast<float>(n.z / len);
        }
    }
    return normals;
}

MeshData build_building_mesh(const std::vector<LonLat>& coords,
                             double height,
                             double origin_lon,
                             double origin_lat) {
    MeshData mesh;
    if (coords.size() < 4) {
        return mesh;
    }

    std::vector<Point2> ring;
    for (size_t i = 0; i + 1 < coords.size(); ++i) {
        ring.push_back(mercator_xy(coords[i].lon, coords[i].lat, origin_lon, origin_lat));
    }
    if (ring.size() < 3) {
        return mesh;
    }

    std::vector<std::vector<Point2>> polygon = {ring};
    const auto top_tris = mapbox::earcut<uint32_t>(polygon);
    if (top_tris.size() < 3) {
        return mesh;
    }

    for (const auto& p : ring) {
        mesh.positions.push_back({static_cast<float>(p.x), 0.0f, static_cast<float>(-p.y)});
    }
    for (const auto& p : ring) {
        mesh.positions.push_back({static_cast<float>(p.x), static_cast<float>(height), static_cast<float>(-p.y)});
    }

    const uint32_t n = static_cast<uint32_t>(ring.size());
    for (size_t i = 0; i + 2 < top_tris.size(); i += 3) {
        const uint32_t a = top_tris[i];
        const uint32_t b = top_tris[i + 1];
        const uint32_t c = top_tris[i + 2];
        mesh.indices.insert(mesh.indices.end(), {a + n, b + n, c + n, c, b, a});
    }

    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t j = (i + 1) % n;
        mesh.indices.insert(mesh.indices.end(), {i, j, j + n, i, j + n, i + n});
    }

    mesh.normals = compute_vertex_normals(mesh.positions, mesh.indices);
    return mesh;
}

MeshData build_area_mesh(const std::vector<LonLat>& coords,
                         double origin_lon,
                         double origin_lat,
                         double y = 0.01) {
    MeshData mesh;
    if (coords.size() < 4) {
        return mesh;
    }

    std::vector<Point2> ring;
    for (size_t i = 0; i + 1 < coords.size(); ++i) {
        ring.push_back(mercator_xy(coords[i].lon, coords[i].lat, origin_lon, origin_lat));
    }
    if (ring.size() < 3) {
        return mesh;
    }

    std::vector<std::vector<Point2>> polygon = {ring};
    mesh.indices = mapbox::earcut<uint32_t>(polygon);
    if (mesh.indices.size() < 3) {
        mesh.indices.clear();
        return mesh;
    }

    for (const auto& p : ring) {
        mesh.positions.push_back({static_cast<float>(p.x), static_cast<float>(y), static_cast<float>(-p.y)});
        mesh.normals.push_back({0.0f, 1.0f, 0.0f});
    }
    return mesh;
}

MeshData build_road_mesh(const std::vector<LonLat>& coords,
                         double width,
                         double origin_lon,
                         double origin_lat,
                         double y = 0.03) {
    MeshData mesh;
    if (coords.size() < 2) {
        return mesh;
    }

    std::vector<Point2> points;
    for (const auto& coord : coords) {
        points.push_back(mercator_xy(coord.lon, coord.lat, origin_lon, origin_lat));
    }

    const double half = width / 2.0;
    for (size_t i = 0; i < points.size(); ++i) {
        double dx = 0.0;
        double dz = 0.0;
        if (i == 0) {
            dx = points[1].x - points[i].x;
            dz = points[1].y - points[i].y;
        } else if (i + 1 == points.size()) {
            dx = points[i].x - points[i - 1].x;
            dz = points[i].y - points[i - 1].y;
        } else {
            dx = points[i + 1].x - points[i - 1].x;
            dz = points[i + 1].y - points[i - 1].y;
        }

        const double len = std::sqrt(dx * dx + dz * dz);
        const double nx = len == 0.0 ? 0.0 : -dz / len;
        const double nz = len == 0.0 ? 0.0 : dx / len;

        mesh.positions.push_back({
            static_cast<float>(points[i].x + nx * half),
            static_cast<float>(y),
            static_cast<float>(-(points[i].y + nz * half)),
        });
        mesh.positions.push_back({
            static_cast<float>(points[i].x - nx * half),
            static_cast<float>(y),
            static_cast<float>(-(points[i].y - nz * half)),
        });
        mesh.normals.push_back({0.0f, 1.0f, 0.0f});
        mesh.normals.push_back({0.0f, 1.0f, 0.0f});
    }

    for (uint32_t i = 0; i + 1 < points.size(); ++i) {
        const uint32_t left_a = i * 2;
        const uint32_t right_a = left_a + 1;
        const uint32_t left_b = left_a + 2;
        const uint32_t right_b = left_a + 3;
        mesh.indices.insert(mesh.indices.end(), {left_a, right_a, left_b, right_a, right_b, left_b});
    }
    return mesh;
}

json make_building_metadata(const Feature& building,
                            const std::string& name,
                            double height,
                            const json& material_info) {
    auto props = unset_electrical_properties();
    return {
        {"osm_type", "way"},
        {"osm_id", std::stoll(building.id)},
        {"name", name},
        {"building", tag_value(building.tags, "building").empty() ? nullptr : json(tag_value(building.tags, "building"))},
        {"height_m", height},
        {"height_tag", tag_value(building.tags, "height").empty() ? nullptr : json(tag_value(building.tags, "height"))},
        {"building_levels", tag_value(building.tags, "building:levels").empty() ? nullptr : json(tag_value(building.tags, "building:levels"))},
        {"osm_material", material_info["osm_material"]},
        {"inferred_material", material_info["inferred_material"]},
        {"material_inference_source", material_info["material_inference_source"]},
        {"dielectric_constant", props["dielectric_constant"]},
        {"conductivity_s_per_m", props["conductivity_s_per_m"]},
        {"source_tags", tags_to_json(building.tags)},
    };
}

json make_surface_metadata(const Feature& feature,
                           const std::string& name,
                           const std::string& feature_type) {
    const auto [material, source] = infer_surface_material(feature_type, feature.tags);
    auto props = unset_electrical_properties();
    json osm_id = feature.id == "bbox_ground" ? json(feature.id) : json(std::stoll(feature.id));
    return {
        {"osm_type", feature.osm_type},
        {"osm_id", osm_id},
        {"name", name},
        {"feature_type", feature_type},
        {"inferred_material", material},
        {"material_inference_source", source},
        {"dielectric_constant", props["dielectric_constant"]},
        {"conductivity_s_per_m", props["conductivity_s_per_m"]},
        {"source_tags", tags_to_json(feature.tags)},
    };
}

class CountHandler : public osmium::handler::Handler {
public:
    uint64_t total_items = 0;

    void way(const osmium::Way&) noexcept {
        ++total_items;
    }

    void relation(const osmium::Relation&) noexcept {
        ++total_items;
    }
};

class Collector : public osmium::handler::Handler {
public:
    explicit Collector(BBox bbox, uint64_t total_items)
        : bbox_(bbox), total_items_(total_items) {}

    std::vector<Feature> buildings;
    std::vector<Feature> roads;
    std::vector<Feature> areas;
    std::unordered_map<osmium::object_id_type, std::vector<LonLat>> way_geometries;

    void way(const osmium::Way& way) {
        update_progress();

        std::map<std::string, std::string> tags;
        for (const auto& tag : way.tags()) {
            tags[tag.key()] = tag.value();
        }

        std::vector<LonLat> coords;
        for (const auto& node_ref : way.nodes()) {
            const auto location = node_ref.location();
            if (!location.valid()) {
                return;
            }
            coords.push_back({location.lon(), location.lat()});
        }

        if (coords.size() < 2 || !any_point_in_bbox(coords, bbox_)) {
            return;
        }

        way_geometries[way.id()] = coords;

        Feature feature;
        feature.osm_type = "way";
        feature.id = std::to_string(way.id());
        feature.coords = coords;
        feature.tags = tags;

        const bool closed = coords.size() >= 4 && same_lonlat(coords.front(), coords.back());
        if (has_tag(tags, "building") && closed) {
            buildings.push_back(feature);
        } else if (has_tag(tags, "highway")) {
            roads.push_back(feature);
        } else if (closed && is_water_feature(tags)) {
            feature.feature_type = "water";
            areas.push_back(feature);
        } else if (closed) {
            const auto cover = landcover_type(tags);
            if (!cover.empty()) {
                feature.feature_type = cover;
                areas.push_back(feature);
            }
        }
    }

    void relation(const osmium::Relation& relation) {
        update_progress();

        std::map<std::string, std::string> tags;
        for (const auto& tag : relation.tags()) {
            tags[tag.key()] = tag.value();
        }

        if (tag_value(tags, "type") != "multipolygon") {
            return;
        }

        std::string feature_type;
        if (is_water_feature(tags)) {
            feature_type = "water";
        } else {
            feature_type = landcover_type(tags);
        }
        if (feature_type.empty()) {
            return;
        }

        std::vector<std::vector<LonLat>> outer_segments;
        for (const auto& member : relation.members()) {
            if (member.type() != osmium::item_type::way) {
                continue;
            }
            const std::string role = member.role();
            if (!role.empty() && role != "outer") {
                continue;
            }

            const auto it = way_geometries.find(member.ref());
            if (it != way_geometries.end()) {
                outer_segments.push_back(it->second);
            }
        }

        for (const auto& ring : join_way_segments(outer_segments)) {
            if (!any_point_in_bbox(ring, bbox_)) {
                continue;
            }

            Feature feature;
            feature.osm_type = "relation";
            feature.id = std::to_string(relation.id());
            feature.coords = ring;
            feature.tags = tags;
            feature.feature_type = feature_type;
            areas.push_back(feature);
        }
    }

private:
    BBox bbox_;
    uint64_t total_items_ = 0;
    uint64_t processed_items_ = 0;
    int next_progress_percent_ = 5;

    void update_progress() {
        if (total_items_ == 0) {
            return;
        }
        ++processed_items_;
        const int percent = static_cast<int>(processed_items_ * 100 / total_items_);
        while (percent >= next_progress_percent_ && next_progress_percent_ <= 100) {
            std::cout << "OSM read progress: " << next_progress_percent_ << "%\n";
            next_progress_percent_ += 5;
        }
    }
};

Args parse_args(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error(
            "Usage: osm_pbf_to_scene_gltf <input.osm.pbf> <output.glb|output.gltf> "
            "--bbox MIN_LAT MIN_LON MAX_LAT MAX_LON [--format glb|gltf] [--material-mode actual|infer]");
    }

    Args args;
    std::vector<std::string> values(argv + 1, argv + argc);
    if (values.size() < 2) {
        throw std::runtime_error("Input PBF and output path are required.");
    }
    args.pbf = values[0];
    args.output = values[1];

    bool bbox_seen = false;
    for (size_t i = 2; i < values.size(); ++i) {
        if (values[i] == "--format" && i + 1 < values.size()) {
            args.format = values[++i];
        } else if (values[i] == "--material-mode" && i + 1 < values.size()) {
            args.material_mode = values[++i];
        } else if (values[i] == "--bbox" && i + 4 < values.size()) {
            args.bbox_input = {
                std::stod(values[++i]),
                std::stod(values[++i]),
                std::stod(values[++i]),
                std::stod(values[++i]),
            };
            bbox_seen = true;
        } else {
            throw std::runtime_error("Unknown or incomplete argument: " + values[i]);
        }
    }

    if (!bbox_seen) {
        throw std::runtime_error("--bbox MIN_LAT MIN_LON MAX_LAT MAX_LON is required.");
    }

    if (args.output.find('.') == std::string::npos) {
        args.output += "." + args.format;
    }
    return args;
}

std::string extras_output_path(const std::string& output_path) {
    const auto dot = output_path.find_last_of('.');
    if (dot == std::string::npos) {
        return output_path + "_extras.json";
    }
    return output_path.substr(0, dot) + "_extras.json";
}

int main(int argc, char** argv) {
    try {
        const auto args = parse_args(argc, argv);

        double min_lat = args.bbox_input[0];
        double min_lon = args.bbox_input[1];
        double max_lat = args.bbox_input[2];
        double max_lon = args.bbox_input[3];
        if (min_lon > max_lon) {
            std::swap(min_lon, max_lon);
        }
        if (min_lat > max_lat) {
            std::swap(min_lat, max_lat);
        }

        if (min_lon < -180.0 || max_lon > 180.0) {
            throw std::runtime_error("Longitude must be between -180 and 180.");
        }
        if (min_lat < -90.0 || max_lat > 90.0) {
            throw std::runtime_error("Latitude must be between -90 and 90.");
        }

        const BBox bbox{min_lon, min_lat, max_lon, max_lat};
        const double origin_lon = (min_lon + max_lon) / 2.0;
        const double origin_lat = (min_lat + max_lat) / 2.0;

        std::cout << "Using bbox input format: minLat minLon maxLat maxLon\n";
        std::cout << "Using bbox lon/lat: " << min_lon << " " << min_lat << " " << max_lon << " " << max_lat << "\n";
        std::cout << "Output 3D file: " << args.output << "\n";
        std::cout << "Material mode: " << args.material_mode << "\n";

        std::cout << "Counting OSM ways/relations for progress...\n";
        CountHandler counter;
        {
            osmium::io::File input_file(args.pbf);
            osmium::io::Reader reader(input_file);
            osmium::apply(reader, counter);
            reader.close();
        }

        std::cout << "Reading OSM map data...\n";
        Collector collector(bbox, counter.total_items);
        {
            using index_type = osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;
            using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;
            index_type index;
            location_handler_type location_handler(index);
            location_handler.ignore_errors();

            osmium::io::File input_file(args.pbf);
            osmium::io::Reader reader(input_file);
            osmium::apply(reader, location_handler, collector);
            reader.close();
        }

        GltfWriter writer;
        std::map<std::string, int> material_indices;
        for (const auto& feature_type : {"road", "water", "grass", "forest", "sand", "land"}) {
            material_indices[feature_type] = writer.add_material(feature_type, feature_color(feature_type));
        }

        uint64_t exported_building_count = 0;
        uint64_t exported_road_count = 0;
        uint64_t exported_area_count = 0;
        json extras_items = json::array();

        Feature ground;
        ground.osm_type = "generated";
        ground.id = "bbox_ground";
        ground.coords = bbox_ring(bbox);
        ground.tags = {{"generated", "bbox_ground"}};
        auto ground_mesh = build_area_mesh(ground.coords, origin_lon, origin_lat, -0.02);
        if (!ground_mesh.positions.empty() && !ground_mesh.indices.empty()) {
            auto metadata = make_surface_metadata(ground, "bbox_ground", "land");
            metadata["gltf_node_index"] = writer.nodes.size();
            metadata["vertex_count"] = ground_mesh.positions.size();
            metadata["triangle_count"] = ground_mesh.indices.size() / 3;
            writer.add_mesh_node("bbox_ground", ground_mesh, material_indices["land"], metadata);
            extras_items.push_back(metadata);
        }

        for (const auto& building : collector.buildings) {
            const auto material_info = resolve_building_material(building.tags, args.material_mode);
            const std::string inferred_material = material_info["inferred_material"];
            if (!material_indices.count(inferred_material)) {
                material_indices[inferred_material] = writer.add_material("building_" + inferred_material,
                                                                          material_color(inferred_material));
            }

            const double height = building_height(building.tags);
            const auto name = clean_text(building_name(building.tags, building.id));
            auto mesh = build_building_mesh(building.coords, height, origin_lon, origin_lat);
            if (mesh.positions.empty() || mesh.indices.empty()) {
                continue;
            }

            auto metadata = make_building_metadata(building, name, height, material_info);
            metadata["gltf_node_index"] = writer.nodes.size();
            metadata["vertex_count"] = mesh.positions.size();
            metadata["triangle_count"] = mesh.indices.size() / 3;
            writer.add_mesh_node(name, mesh, material_indices[inferred_material], metadata);
            extras_items.push_back(metadata);
            ++exported_building_count;
        }

        for (const auto& road : collector.roads) {
            const auto name = feature_name(road.tags, road.id, "road");
            const double width = road_width_m(road.tags);
            auto mesh = build_road_mesh(road.coords, width, origin_lon, origin_lat);
            if (mesh.positions.empty() || mesh.indices.empty()) {
                continue;
            }

            auto metadata = make_surface_metadata(road, name, "road");
            metadata["highway"] = tag_value(road.tags, "highway");
            metadata["width_m"] = width;
            metadata["gltf_node_index"] = writer.nodes.size();
            metadata["vertex_count"] = mesh.positions.size();
            metadata["triangle_count"] = mesh.indices.size() / 3;
            writer.add_mesh_node(name, mesh, material_indices["road"], metadata);
            extras_items.push_back(metadata);
            ++exported_road_count;
        }

        for (const auto& area : collector.areas) {
            const auto feature_type = area.feature_type;
            const auto name = feature_name(area.tags, area.id, feature_type);
            auto mesh = build_area_mesh(area.coords, origin_lon, origin_lat);
            if (mesh.positions.empty() || mesh.indices.empty()) {
                continue;
            }

            auto metadata = make_surface_metadata(area, name, feature_type);
            metadata["gltf_node_index"] = writer.nodes.size();
            metadata["vertex_count"] = mesh.positions.size();
            metadata["triangle_count"] = mesh.indices.size() / 3;
            writer.add_mesh_node(name, mesh, material_indices.count(feature_type) ? material_indices[feature_type] : material_indices["land"], metadata);
            extras_items.push_back(metadata);
            ++exported_area_count;
        }

        writer.save(args.output);

        const auto extras_path = extras_output_path(args.output);
        json extras_doc = {
            {"input_pbf", args.pbf},
            {"output_3d", args.output},
            {"output_format", args.output.substr(args.output.find_last_of('.') + 1)},
            {"material_mode", args.material_mode},
            {"bbox_format", "minLat minLon maxLat maxLon"},
            {"bbox_input", args.bbox_input},
            {"bbox_lonlat", {
                {"min_lon", min_lon},
                {"min_lat", min_lat},
                {"max_lon", max_lon},
                {"max_lat", max_lat},
            }},
            {"extras_count", extras_items.size()},
            {"extras", extras_items},
        };
        std::ofstream extras_out(extras_path);
        extras_out << std::setw(2) << extras_doc << "\n";

        std::cout << "Saved 3D file: " << args.output << "\n";
        std::cout << "Saved extras JSON: " << extras_path << "\n";
        std::cout << "Buildings read from OSM: " << collector.buildings.size() << "\n";
        std::cout << "Buildings exported to mesh: " << exported_building_count << "\n";
        std::cout << "Roads read from OSM: " << collector.roads.size() << "\n";
        std::cout << "Roads exported to mesh: " << exported_road_count << "\n";
        std::cout << "Surface areas read from OSM: " << collector.areas.size() << "\n";
        std::cout << "Surface areas exported to mesh: " << exported_area_count << "\n";

        if (args.output.size() >= 5 && args.output.substr(args.output.size() - 5) == ".gltf") {
            std::cout << "Note: keep the generated .bin file next to the .gltf file.\n";
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
