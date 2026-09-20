#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/SpiralVase.hpp"
#include "libslic3r/GCode/CoolingBuffer.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "test_helpers.hpp"

#include <vector>

using namespace Slic3r;

namespace {

FullPrintConfig spiral_vase_config()
{
    FullPrintConfig config = FullPrintConfig::defaults();
    config.spiral_mode_allow_islands.value = true;
    config.use_relative_e_distances.value = true;
    config.spiral_starting_flow_ratio.value = 1.;
    config.spiral_finishing_flow_ratio.value = 0.25;
    config.spiral_mode_smooth.value = false;
    return config;
}

const std::string two_loop_layer = R"(G1 Z0.4 F600
;_SPIRAL_VASE_BEGIN 0 0.4
G1 X0 Y0 F6000
G1 X10 Y0 E1 F1200
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
;_SPIRAL_VASE_END
G1 E-0.8 F1800
G1 X20 Y0 F6000
G1 E0.8 F1800
G1 X25 Y0 E1 F1200
G1 X25 Y5 E1
G1 X20 Y5 E1
G1 X20 Y0 E1
)";

const std::string primary_with_travel_layer = R"(G1 Z0.4 F600
;_SPIRAL_VASE_BEGIN 0 0.4
G1 E-0.8 F1800
G1 X5 Y0 F6000
G1 E0.8 F1800
G1 X15 Y0 E1 F1200
G1 X15 Y10 E1
G1 X5 Y10 E1
G1 X5 Y0 E1
;_SPIRAL_VASE_END
)";

struct LayerExtrusions {
    std::vector<float> primary_z;
    std::vector<float> island_z;
};

LayerExtrusions extrusion_heights(const std::string &gcode)
{
    LayerExtrusions result;
    GCodeReader reader;
    GCodeConfig reader_config;
    reader_config.use_relative_e_distances.value = true;
    reader.apply_config(reader_config);
    reader.parse_buffer(gcode, [&result](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
        if (!line.extruding(reader) || line.dist_XY(reader) <= 0.f)
            return;
        if (line.new_X(reader) >= 20.f)
            result.island_z.emplace_back(line.new_Z(reader));
        else
            result.primary_z.emplace_back(line.new_Z(reader));
    });
    return result;
}

} // namespace

TEST_CASE("Spiral Vase prints disconnected islands at the completed layer height", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);

    const std::string output = vase.process_layer(two_loop_layer, false);
    const LayerExtrusions heights = extrusion_heights(output);

    REQUIRE(output.find("G1 E-0.8 F1800") != std::string::npos);
    REQUIRE(output.find("G1 X20 Y0 F6000") != std::string::npos);
    REQUIRE(heights.primary_z.size() == 4);
    REQUIRE(heights.island_z.size() == 4);
    CHECK_THAT(heights.primary_z.front(), Catch::Matchers::WithinAbs(0.1f, 0.0001f));
    CHECK_THAT(heights.primary_z.back(), Catch::Matchers::WithinAbs(0.4f, 0.0001f));
    for (float z : heights.island_z)
        CHECK_THAT(z, Catch::Matchers::WithinAbs(0.4f, 0.0001f));
}

TEST_CASE("Spiral Vase does not duplicate islands in the finishing transition", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);

    const LayerExtrusions heights = extrusion_heights(vase.process_layer(two_loop_layer, true));

    CHECK(heights.primary_z.size() == 8);
    CHECK(heights.island_z.size() == 4);
}

TEST_CASE("Spiral Vase finishing moves survive cooling feedrate deduplication", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    GCode generator;
    generator.apply_print_config(config);
    generator.writer().set_extruders({0});
    generator.writer().set_extruder(0);
    CoolingBuffer cooling(generator);

    const std::string output = cooling.process_layer(vase.process_layer(two_loop_layer, true), 1, true);
    const LayerExtrusions heights = extrusion_heights(output);
    REQUIRE(heights.primary_z.size() == 8);
    CHECK(heights.island_z.size() == 4);
    for (size_t i = 4; i < 8; ++i)
        CHECK_THAT(heights.primary_z[i], Catch::Matchers::WithinAbs(0.4f, 0.0001f));
    GCodeReader reader;
    reader.parse_buffer(output, [](GCodeReader&, const GCodeReader::GCodeLine& line) {
        if (line.has_x() || line.has_y() || line.has_z() || line.has_e())
            CHECK(line.cmd_is("G1"));
    });
}

TEST_CASE("Spiral Vase preserves a primary loop handoff", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);

    const std::string output = vase.process_layer(primary_with_travel_layer, false);

    CHECK(output.find("G1 E-0.8 F1800") != std::string::npos);
    CHECK(output.find("G1 X5 Y0 F6000") != std::string::npos);
    CHECK(output.find("G1 E0.8 F1800") != std::string::npos);
}

TEST_CASE("Spiral Vase keeps touching loops separate even without a travel distance", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.4
;_SPIRAL_VASE_BEGIN 0 0.4
G1 X1 Y0 E1
G1 X1 Y1 E1
G1 X0 Y1 E1
G1 X0 Y0 E1
;_SPIRAL_VASE_END
G1 X0 Y0 F6000
G1 X10 Y0 E1
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
)";
    const std::string output = vase.process_layer(layer, false);
    const std::string island = layer.substr(layer.find("G1 X0 Y0 F6000"));
    CHECK(output.find(island) != std::string::npos);
    CHECK(output.find(SpiralVase::primary_begin) == std::string::npos);
    CHECK(output.find(SpiralVase::primary_end) == std::string::npos);
}

TEST_CASE("Spiral Vase uses the complete marked contour across short segments and comments", "[SpiralVase]")
{
    FullPrintConfig config = spiral_vase_config();
    config.spiral_mode_smooth.value = GENERATE(false, true);
    SpiralVase vase(config);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.4
;_SPIRAL_VASE_BEGIN 0 0.4
G1 X0.001 Y0 E0.001 F1200
;WIDTH:0.42
G1 F900
G1 X10 Y0 E0.999
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
;_SPIRAL_VASE_END
G1 X20 Y0 F6000
G1 X20.3 Y0 E0.01
)";
    const LayerExtrusions heights = extrusion_heights(vase.process_layer(layer, true));
    REQUIRE(heights.primary_z.size() == 10);
    CHECK(heights.island_z.size() == 1);
    CHECK(heights.primary_z.front() < 0.001f);
    CHECK_THAT(heights.primary_z[4], Catch::Matchers::WithinAbs(0.4f, 0.0001f));
    for (size_t i = 5; i < 10; ++i)
        CHECK_THAT(heights.primary_z[i], Catch::Matchers::WithinAbs(0.4f, 0.0001f));
}

TEST_CASE("Spiral Vase preserves ordinary motion when no complete primary is marked", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.4
G1 X0.001 Y0 E0.001
G1 E-0.8
G1 Z1
G1 X20 Y0 F6000
G1 Z0.4
G1 E0.8
G1 X25 Y0 E1
)";
    CHECK(vase.process_layer(layer, true) == layer);
}

TEST_CASE("Spiral Vase does not mistake an earlier fragment for the marked primary", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.4
G1 X20 Y0
G1 X20.3 Y0 E0.01
;_SPIRAL_VASE_BEGIN 0 0.4
G1 X0 Y0
G1 X10 Y0 E1
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
;_SPIRAL_VASE_END
)";
    const LayerExtrusions heights = extrusion_heights(vase.process_layer(layer, false));
    REQUIRE(heights.primary_z.size() == 4);
    REQUIRE(heights.island_z.size() == 1);
    CHECK_THAT(heights.primary_z.front(), Catch::Matchers::WithinAbs(0.1f, 0.0001f));
    CHECK_THAT(heights.primary_z.back(), Catch::Matchers::WithinAbs(0.4f, 0.0001f));
    CHECK_THAT(heights.island_z.front(), Catch::Matchers::WithinAbs(0.4f, 0.0001f));
}

TEST_CASE("Spiral Vase declines disconnected marked geometry without deleting travel", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    const std::string moves = "G1 X10 Y0 E1\nG1 X30 Y0\nG1 X0 Y0 E1\n";
    const std::string layer = "G1 Z0.4\n;_SPIRAL_VASE_BEGIN 0 0.4\n" + moves + ";_SPIRAL_VASE_END\n";
    CHECK(vase.process_layer(layer, true) == "G1 Z0.4\n" + moves);
}

TEST_CASE("Spiral Vase preserves lift heights and resumes a primary after island travel", "[SpiralVase]")
{
    const FullPrintConfig config = spiral_vase_config();
    SpiralVase vase(config);
    vase.enable(true);
    vase.process_layer(two_loop_layer, false);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.8
;_SPIRAL_VASE_BEGIN 0.4 0.8
G1 E-0.8
G1 Z1.4
G1 X0 Y0 F6000
G1 Z0.8
G1 E0.8
G1 X10 Y0 E1
G1 X10 Y10 E1
G1 X0 Y10 E1
G1 X0 Y0 E1
;_SPIRAL_VASE_END
G1 E-0.8
G1 Z1.4
G1 X20 Y0 F6000
G1 Z0.8
G1 E0.8
G1 X25 Y0 E1
)";
    const std::string output = vase.process_layer(layer, false);
    CHECK(output.find("G1 Z1.4\nG1 X0 Y0 F6000\nG1 Z0.8") != std::string::npos);
    CHECK(output.find(layer.substr(layer.find("G1 E-0.8", layer.find(SpiralVase::primary_end)))) != std::string::npos);
    const LayerExtrusions heights = extrusion_heights(output);
    REQUIRE(heights.primary_z.size() == 4);
    CHECK_THAT(heights.primary_z.front(), Catch::Matchers::WithinAbs(0.5f, 0.0001f));
    CHECK_THAT(heights.primary_z.back(), Catch::Matchers::WithinAbs(0.8f, 0.0001f));
}

TEST_CASE("Spiral Vase restores absolute extrusion after flow transitions", "[SpiralVase]")
{
    FullPrintConfig config = spiral_vase_config();
    config.use_relative_e_distances.value = false;
    config.spiral_starting_flow_ratio.value = 0.5;
    SpiralVase vase(config);
    vase.enable(true);
    const std::string layer = R"(G1 Z0.4
;_SPIRAL_VASE_BEGIN 0 0.4
G1 X10 Y0 E1
G1 X10 Y10 E2
G1 X0 Y10 E3
G1 X0 Y0 E4
;_SPIRAL_VASE_END
G1 E3.2
G1 X20 Y0
G1 E4
G1 X25 Y0 E5
)";
    const std::string output = vase.process_layer(layer, true);
    GCodeReader reader;
    reader.apply_config(config);
    std::vector<float> extrusions;
    reader.parse_buffer(output, [&](GCodeReader& reader, const GCodeReader::GCodeLine& line) {
        if (line.extruding(reader) && line.dist_XY(reader) > 0.f)
            extrusions.push_back(line.dist_E(reader));
    });
    REQUIRE(extrusions.size() == 9);
    CHECK_THAT(extrusions.front(), Catch::Matchers::WithinAbs(0.625f, 0.00001f));
    CHECK_THAT(extrusions[7], Catch::Matchers::WithinAbs(0.25f, 0.00001f));
    CHECK_THAT(extrusions.back(), Catch::Matchers::WithinAbs(1.f, 0.00001f));
}

TEST_CASE("Spiral Vase leaves island coordinates unchanged during smoothing", "[SpiralVase]")
{
    FullPrintConfig config = spiral_vase_config();
    config.spiral_mode_smooth.value = true;
    SpiralVase vase(config);
    vase.set_max_xy_smoothing(100.f);
    vase.enable(true);
    vase.process_layer(two_loop_layer, false);
    vase.enable(true);
    std::string layer = two_loop_layer;
    layer.replace(layer.find("Z0.4"), 4, "Z0.8");
    layer.replace(layer.find("BEGIN 0 0.4"), 11, "BEGIN 0.4 0.8");
    const std::string output = vase.process_layer(layer, false);
    const std::string island = layer.substr(layer.find("G1 E-0.8"));
    CHECK(output.find(island) != std::string::npos);
    const LayerExtrusions heights = extrusion_heights(output);
    REQUIRE(heights.primary_z.size() == 4);
    CHECK_THAT(heights.primary_z.back(), Catch::Matchers::WithinAbs(0.8f, 0.0001f));
}

TEST_CASE("Spiral Vase keeps the disabled option on the legacy path", "[SpiralVase]")
{
    FullPrintConfig config = spiral_vase_config();
    config.spiral_mode_allow_islands.value = false;
    SpiralVase vase(config);
    vase.enable(true);
    const std::string output = vase.process_layer("G1 Z0.4\nG1 X10 Y0 E1\nG1 X10 Y10 E1\nG1 X0 Y10 E1\nG1 X0 Y0 E1\n", false);
    const LayerExtrusions heights = extrusion_heights(output);
    REQUIRE(heights.primary_z.size() == 4);
    CHECK_THAT(heights.primary_z.front(), Catch::Matchers::WithinAbs(0.1f, 0.0001f));
    CHECK_THAT(heights.primary_z.back(), Catch::Matchers::WithinAbs(0.4f, 0.0001f));
}

TEST_CASE("Spiral Vase emits one helix and a flat island through both wall generators", "[SpiralVase]")
{
    const std::string generator = GENERATE("classic", "arachne");
    const bool relative_e = GENERATE(false, true);
    CAPTURE(generator, relative_e);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"spiral_mode", true}, {"spiral_mode_allow_islands", true},
        {"spiral_mode_smooth", false}, {"use_relative_e_distances", relative_e},
        {"wall_generator", generator}, {"wall_loops", 1},
        {"bottom_shell_layers", 1}, {"bottom_shell_thickness", 0.},
        {"top_shell_layers", 0}, {"sparse_infill_density", "0%"},
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0}, {"brim_type", "no_brim"},
        {"enable_arc_fitting", false}, {"gcode_comments", true}
    });
    TriangleMesh mesh = make_cube(10., 10., 1.);
    TriangleMesh island = make_cube(5., 5., 1.);
    island.translate(20., 0., 0.);
    mesh.merge(island);
    const std::string gcode = Test::slice({mesh}, config);
    CHECK(gcode.find(SpiralVase::primary_begin) == std::string::npos);
    CHECK(gcode.find(SpiralVase::primary_end) == std::string::npos);
    GCodeReader reader;
    reader.apply_config(config);
    size_t layer = 0;
    size_t helix_moves = 0;
    size_t island_moves = 0;
    bool flat_started = false;
    reader.parse_buffer(gcode, [&](GCodeReader& reader, const GCodeReader::GCodeLine& line) {
        if (line.raw() == ";LAYER_CHANGE")
            ++layer;
        // Middle layer: above the base and below the finishing transition.
        if (layer != 3 || !line.extruding(reader) || line.dist_XY(reader) <= 0.f)
            return;
        if (line.dist_Z(reader) > 0.0001f) {
            CHECK_FALSE(flat_started);
            ++helix_moves;
        } else {
            flat_started = true;
            ++island_moves;
            CHECK_THAT(line.new_Z(reader), Catch::Matchers::WithinAbs(0.6f, 0.001f));
        }
    });
    CHECK(helix_moves > 0);
    CHECK(island_moves > 0);
}
